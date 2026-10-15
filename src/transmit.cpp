#include "transmit.h"

#include <WiFi.h>
#include <WebServer.h>

// ------------------------------------------------------------
// The WebServer instance is owned by WebDashboard.cpp.
// We declare it extern here and register our routes on it.
// ------------------------------------------------------------

extern WebServer server;

// ============================================================
// TRANSMIT STATE
// ============================================================

#define TRANSMIT_MAX_CLIENTS 4
#define TRANSMIT_COALESCE_MS 10

// Connected SSE clients.
WiFiClient transmitClients[TRANSMIT_MAX_CLIENTS];

// The latest readings JSON we know about.
String transmitLatestJSON = "{}";

// The last JSON we actually wrote to clients.
// Used to detect "no change, no need to send".
String transmitLastSentJSON = "";

// Pending flag + timestamp for coalescing.
bool transmitPending = false;
unsigned long transmitPendingSince = 0;

// ------------------------------------------------------------
// Send SSE headers to a freshly connected client.
// ------------------------------------------------------------

static void sendSseHeaders(WiFiClient &client)
{
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/event-stream");
    client.println("Cache-Control: no-cache");
    client.println("Connection: keep-alive");
    client.println("Access-Control-Allow-Origin: *");
    client.println();
}

// ------------------------------------------------------------
// Find a free slot in the client array.
// Returns -1 if all slots are taken.
// ------------------------------------------------------------

static int findFreeClientSlot()
{
    for (int i = 0; i < TRANSMIT_MAX_CLIENTS; i++)
    {
        if (!transmitClients[i] ||
            !transmitClients[i].connected())
        {
            return i;
        }
    }

    return -1;
}

// ------------------------------------------------------------
// Prune disconnected clients.
// ------------------------------------------------------------

static void pruneDisconnectedClients()
{
    for (int i = 0; i < TRANSMIT_MAX_CLIENTS; i++)
    {
        if (transmitClients[i] &&
            !transmitClients[i].connected())
        {
            transmitClients[i].stop();
        }
    }
}

// ------------------------------------------------------------
// Write one SSE event to every connected client.
// ------------------------------------------------------------

static void broadcastEvent(const String &json)
{
    for (int i = 0; i < TRANSMIT_MAX_CLIENTS; i++)
    {
        WiFiClient &client = transmitClients[i];

        if (!client || !client.connected())
        {
            continue;
        }

        // SSE frame:
        //   data: <payload>\n\n

        client.print("data: ");
        client.print(json);
        client.print("\n\n");
    }
}

// ============================================================
// ROUTE HANDLERS
// ============================================================

// ------------------------------------------------------------
// GET /stream
//
// Take over the raw socket and hold it open. We send the
// SSE headers, register the client, and return without
// calling server.send(). The WebServer library is happy
// with this: the connection stays open until the client
// disconnects, and we write to it directly.
// ------------------------------------------------------------

static void handleStreamRoute()
{
    WiFiClient client = server.client();

    int slot = findFreeClientSlot();

    if (slot < 0)
    {
        // No free slot. Politely refuse.

        client.println("HTTP/1.1 503 Service Unavailable");
        client.println("Content-Type: text/plain");
        client.println();
        client.println("No free stream slot.");

        return;
    }

    // Send SSE headers manually.

    sendSseHeaders(client);

    // Immediately send the current readings so the client
    // has something to parse right away.

    if (transmitLatestJSON.length() > 0)
    {
        client.print("data: ");
        client.print(transmitLatestJSON);
        client.print("\n\n");
    }

    // Register the client.

    transmitClients[slot] = client;

    Serial.print("[TX] SSE client connected, slot ");
    Serial.println(slot);
}

// ------------------------------------------------------------
// GET /readings
//
// One-shot. Returns the latest readings JSON.
// ------------------------------------------------------------

static void handleReadingsRoute()
{
    server.send(
        200,
        "application/json",
        transmitLatestJSON
    );
}

// ============================================================
// PUBLIC API
// ============================================================

void startTransmitServer()
{
    server.on("/stream",   HTTP_GET, handleStreamRoute);
    server.on("/readings", HTTP_GET, handleReadingsRoute);

    Serial.println();
    Serial.println("================================");
    Serial.println("     TRANSMIT CHANNEL (APP)");
    Serial.println("================================");
    Serial.println("SSE stream:  GET /stream");
    Serial.println("Latest JSON: GET /readings");
    Serial.println("Coalesce:    10 ms");
    Serial.println("Max clients: 4");
    Serial.println("================================");
}

void pushReadingsJSON(const String &json)
{
    // Ignore if identical to last known JSON.

    if (json == transmitLatestJSON)
    {
        return;
    }

    transmitLatestJSON = json;

    // Ignore if identical to last SENT JSON.

    if (json == transmitLastSentJSON)
    {
        return;
    }

    // Mark as pending. handleTransmitClient() will flush
    // after the coalesce window.

    if (!transmitPending)
    {
        transmitPending = true;
        transmitPendingSince = millis();
    }
}

void handleTransmitClient()
{
    // Let the web server deal with any incoming requests.
    // This also gives it a chance to route /stream and
    // /readings to the handlers above.

    // NOTE: server.handleClient() is already called from
    // handleWebDashboard() in the main loop. So we do NOT
    // call it here - we only do the flushing work.

    // --------------------------------------------------------
    // Prune disconnected clients
    // --------------------------------------------------------

    pruneDisconnectedClients();

    // --------------------------------------------------------
    // Flush pending JSON if the coalesce window has passed
    // --------------------------------------------------------

    if (!transmitPending)
    {
        return;
    }

    if (millis() - transmitPendingSince < TRANSMIT_COALESCE_MS)
    {
        return;
    }

    // --------------------------------------------------------
    // If nothing actually changed vs. what was last sent,
    // drop the pending flag and move on.
    // --------------------------------------------------------

    if (transmitLatestJSON == transmitLastSentJSON)
    {
        transmitPending = false;
        return;
    }

    // --------------------------------------------------------
    // Broadcast
    // --------------------------------------------------------

    broadcastEvent(transmitLatestJSON);

    transmitLastSentJSON = transmitLatestJSON;
    transmitPending = false;
}