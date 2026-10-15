#include <Arduino.h>
#include "driver/twai.h"
#include <ArduinoJson.h>
#include <time.h>
#include "WebDashboard.h"
#include "transmit.h"

// ============================================================
// ESP32 TWAI PINS
// ============================================================

#define CAN_RX_PIN 16
#define CAN_TX_PIN 17

// ============================================================
// CAN IDs
// ============================================================

#define OBD_REQUEST_ID  0x7DF
#define ECU_RESPONSE_ID 0x7E8

// ============================================================
// PID TABLE
// ============================================================
//
// This is the full readings set. Sorted ascending.
// Bitmaps (00/20/40/60) and status PIDs (01/03) are NOT
// part of the reading stream - they are discovery / status
// operations handled separately later.
//
// The JSON output contains one entry per PID in this table,
// in the SAME order.
// ============================================================

const uint8_t pids[] =
{
    0x04, // Calculated engine load
    0x05, // Coolant temperature
    0x06, // Short-term fuel trim
    0x07, // Long-term fuel trim
    0x0A, // Fuel pressure
    0x0B, // Intake manifold pressure
    0x0C, // Engine RPM
    0x0D, // Vehicle speed
    0x0E, // Timing advance
    0x0F, // Intake air temperature
    0x10, // Mass air flow
    0x11, // Throttle position
    0x1C, // OBD standard
    0x1F, // Engine runtime
    0x21, // Distance with MIL
    0x2F, // Fuel level
    0x31, // Distance since DTC cleared
    0x33, // Barometric pressure
    0x42, // Control module voltage
    0x46, // Ambient air temperature
    0x51, // Fuel type
    0x5C, // Engine oil temperature
    0x5E  // Engine fuel rate
};

const uint8_t PID_COUNT =
    sizeof(pids) / sizeof(pids[0]);

// ============================================================
// VEHICLE / JSON
// ============================================================

const char *VEHICLE_NAME = "Dacia Sandero Stepway 2017";

struct PidReading
{
    uint8_t pid;
    uint8_t data[8];
    uint8_t dlc;
    bool valid;
};

PidReading pidReadings[PID_COUNT];

// ------------------------------------------------------------
// RAW CAN FRAME FEED
// ------------------------------------------------------------
//
// Rolling buffer of the last CAN_FEED_SIZE frames seen on the
// bus (both our requests and the ECU's responses).
// Rendered into the DIAGNOSTIC EVENTS zone.

#define CAN_FEED_SIZE 40
#define CAN_FEED_LINE_LENGTH 48

char canFeed[CAN_FEED_SIZE][CAN_FEED_LINE_LENGTH];
uint8_t canFeedHead = 0;    // next write position
uint8_t canFeedCount = 0;   // how many valid lines

String activeDtcs[4];
uint8_t activeDtcCount = 0;

// Synthetic timestamp baseline (2026-09-12T15:00:00Z).
// Incremented by 1 second per JSON reading.

uint32_t syntheticUnixTime = 1789225200;

// ============================================================
// DASHBOARD STATE
// ============================================================

String lastRawCAN = "Waiting for CAN frames...";
String lastDiagnosticEvent = "Dashboard started.";

String lastDtcPrediction =
    "No DTC analysis yet.\n"
    "Press READ DTC to start predictive analysis.";

bool dtcPredictionActive = false;
unsigned long dtcPredictionStartMillis = 0;

const unsigned long DTC_MONITOR_WINDOW_MS =
    30UL * 60UL * 1000UL;

// ============================================================
// DASHBOARD REQUEST STATE
// ============================================================

// ------------------------------------------------------------
// Poll interval (ms).
//
// 0 = run PID polling back-to-back for a continuous stream.
// Increase to slow the poll rate if the bus gets crowded.
// ------------------------------------------------------------

const unsigned long LIVE_POLL_INTERVAL = 0;
unsigned long lastLivePollTime = 0;

String lastVin = "Not read yet";
uint8_t lastDtcStatus = 0;

// ------------------------------------------------------------
// The most recent complete readings JSON.
// Updated live as PIDs arrive. Displayed in the JSON zone.
// ------------------------------------------------------------

String latestReadingsJSON = "{}";

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
//
// updateDashboard() is defined further down the file, but it
// is called by pollAllPIDs(), which appears earlier. PlatformIO
// compiles as plain C++, so a prototype must be declared here.

void updateDashboard();

// ============================================================
// SYNTHETIC TIMESTAMP
// ============================================================

String formatSyntheticTimestamp()
{
    syntheticUnixTime++;

    time_t rawTime = (time_t)syntheticUnixTime;

    struct tm *timeInfo = gmtime(&rawTime);

    char buffer[25];

    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", timeInfo);

    return String(buffer);
}

// ============================================================
// COMPACT HEX STRING
// ============================================================

String hexBytesToString(const uint8_t *data, uint8_t offset, uint8_t length)
{
    String result = "";

    for (uint8_t i = 0; i < length; i++)
    {
        char buf[3];

        snprintf(buf, sizeof(buf), "%02X", data[offset + i]);

        result += buf;
    }

    return result;
}

// ============================================================
// RAW CAN FEED
// ============================================================
//
// Appends one line to the rolling feed.
// Old lines are overwritten once the buffer fills.

void pushCanFeed(const char *direction, const twai_message_t &message)
{
    char line[CAN_FEED_LINE_LENGTH];

    int written = snprintf(
        line,
        sizeof(line),
        "%s ID:%03X [%d] ",
        direction,
        message.identifier,
        message.data_length_code
    );

    for (int i = 0;
         i < message.data_length_code &&
         written < (int)sizeof(line) - 4;
         i++)
    {
        written += snprintf(
            line + written,
            sizeof(line) - written,
            "%02X ",
            message.data[i]
        );
    }

    strncpy(
        canFeed[canFeedHead],
        line,
        CAN_FEED_LINE_LENGTH - 1
    );

    canFeed[canFeedHead][CAN_FEED_LINE_LENGTH - 1] = '\0';

    canFeedHead = (canFeedHead + 1) % CAN_FEED_SIZE;

    if (canFeedCount < CAN_FEED_SIZE)
    {
        canFeedCount++;
    }

    // Keep a single-line summary for the events zone.
    lastRawCAN = String(line);
}

String getCanFeedString()
{
    String out;

    out.reserve(CAN_FEED_SIZE * CAN_FEED_LINE_LENGTH);

    // Oldest first.
    uint8_t start =
        (canFeedCount < CAN_FEED_SIZE)
            ? 0
            : canFeedHead;

    for (uint8_t i = 0; i < canFeedCount; i++)
    {
        uint8_t idx = (start + i) % CAN_FEED_SIZE;

        out += canFeed[idx];
        out += "\n";
    }

    if (out.length() == 0)
    {
        out = "Waiting for CAN frames...\n";
    }

    return out;
}

// ============================================================
// JSON EMITTER
// ============================================================
//
// Produces exactly:
//
// {
//   "vehicle": "...",
//   "readings": [
//     { "ecu": "7E8", "pid": "04", "response": "" },
//     ...
//   ]
// }
//
// - PIDs in the same order as pids[].
// - Unread PIDs appear with an empty response.
// - response is the full OBD payload (41 <pid> <data>) as
//   uppercase hex with no spaces.
//
// Called every time a PID arrives (streaming).

String buildReadingsJSON()
{
    JsonDocument doc;

    doc["vehicle"] = VEHICLE_NAME;

    JsonArray readings = doc["readings"].to<JsonArray>();

    for (uint8_t i = 0; i < PID_COUNT; i++)
    {
        JsonObject entry = readings.add<JsonObject>();

        entry["ecu"] = "7E8";

        char pidHex[3];
        snprintf(pidHex, sizeof(pidHex), "%02X", pidReadings[i].pid);

        entry["pid"] = pidHex;

        if (pidReadings[i].valid)
        {
            // ISO-TP Single Frame: byte 0 holds the length nibble.
            // Actual OBD payload begins at byte 1.
            uint8_t payloadLength = pidReadings[i].data[0] & 0x0F;

            entry["response"] = hexBytesToString(
                pidReadings[i].data,
                1,
                payloadLength
            );
        }
        else
        {
            entry["response"] = "";
        }
    }

    String out;

    serializeJson(doc, out);

    return out;
}

// ============================================================
// SEND OBD-II REQUEST
// ============================================================

void sendOBDRequest(uint8_t pid)
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x02;
    message.data[1] = 0x01;
    message.data[2] = pid;

    for (uint8_t i = 3; i < 8; i++)
    {
        message.data[i] = 0x00;
    }

    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(100));

    if (result == ESP_OK)
    {
        pushCanFeed("TX", message);
    }
    else
    {
        Serial.println("ERROR: CAN transmission failed.");
    }
}

// ============================================================
// PRINT CAN FRAME
// ============================================================

void printCANFrame(const twai_message_t &message)
{
    pushCanFeed("RX", message);

    Serial.print("ID: 0x");

    if (message.identifier < 0x100)
        Serial.print("0");

    Serial.print(message.identifier, HEX);

    Serial.print(" DATA: ");

    for (int i = 0; i < message.data_length_code; i++)
    {
        if (message.data[i] < 0x10)
            Serial.print("0");

        Serial.print(message.data[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
}

// ============================================================
// RECEIVE ECU RESPONSE
// ============================================================
//
// Response timeout is short (30 ms) because the ECU answers
// within a few ms. This keeps the poll cycle snappy.

bool receiveECUResponse(uint8_t requestedPID, PidReading &outReading)
{
    outReading.pid = requestedPID;
    outReading.valid = false;

    twai_message_t message;

    unsigned long startTime = millis();

    while (millis() - startTime < 30)
    {
        esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(20));

        if (result != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier != ECU_RESPONSE_ID)
        {
            printCANFrame(message);
            continue;
        }

        if (message.data_length_code < 3)
        {
            continue;
        }

        if (message.data[1] != 0x41)
        {
            continue;
        }

        uint8_t responsePID = message.data[2];

        if (responsePID != requestedPID)
        {
            continue;
        }

        // ----------------------------------------------------
        // Valid response for this PID
        // ----------------------------------------------------

        printCANFrame(message);

        outReading.dlc = message.data_length_code;

        memcpy(outReading.data, message.data, message.data_length_code);

        outReading.valid = true;

        return true;
    }

    Serial.print("RESPONSE <- TIMEOUT PID 0x");

    if (requestedPID < 0x10)
        Serial.print("0");

    Serial.println(requestedPID, HEX);

    return false;
}

// ============================================================
// POLL ALL PIDS
// ============================================================

void pollAllPIDs()
{
    for (uint8_t i = 0; i < PID_COUNT; i++)
    {
        sendOBDRequest(pids[i]);

        receiveECUResponse(pids[i], pidReadings[i]);

        // ----------------------------------------------------
        // Streaming: after every PID, regenerate the JSON and
        // push it to the dashboard. This way the JSON zone
        // updates live as the poll cycle progresses.
        // ----------------------------------------------------

        latestReadingsJSON = buildReadingsJSON();

        updateDashboardJSON(latestReadingsJSON);

        pushReadingsJSON(latestReadingsJSON);

        updateDashboard();

        delay(2);

        handleWebDashboard();
    }

    // Final pass so the JSON reflects the whole cycle.
    latestReadingsJSON = buildReadingsJSON();

    updateDashboardJSON(latestReadingsJSON);

    pushReadingsJSON(latestReadingsJSON);

    updateDashboard();
}

// ============================================================
// WAIT FOR ISO-TP FLOW CONTROL
// ============================================================

bool waitForIsoTpFlowControl()
{
    twai_message_t message;

    unsigned long startTime = millis();

    Serial.println();
    Serial.println("--------------------------------");
    Serial.println("WAITING FOR ISO-TP FLOW CONTROL");
    Serial.println("--------------------------------");

    while (millis() - startTime < 1000)
    {
        esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(50));

        if (result != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier != ECU_RESPONSE_ID)
        {
            printCANFrame(message);
            continue;
        }

        if (message.data_length_code < 3)
            continue;

        uint8_t frameType = (message.data[0] >> 4) & 0x0F;

        if (frameType != 0x03)
            continue;

        uint8_t flowStatus = message.data[0] & 0x0F;
        uint8_t blockSize  = message.data[1];
        uint8_t stMin      = message.data[2];

        printCANFrame(message);

        Serial.print("FC STATUS: ");

        if (flowStatus == 0x00)
        {
            Serial.println("CONTINUE TO SEND");
        }
        else if (flowStatus == 0x01)
        {
            Serial.println("WAIT");
            continue;
        }
        else if (flowStatus == 0x02)
        {
            Serial.println("OVERFLOW / ABORT");
            return false;
        }
        else
        {
            Serial.println("INVALID");
            return false;
        }

        Serial.print("BLOCK SIZE: ");
        Serial.println(blockSize);

        Serial.print("STmin: ");
        Serial.println(stMin);

        Serial.println("FC ACCEPTED.");
        Serial.println("--------------------------------");

        return true;
    }

    Serial.println("ERROR: ISO-TP Flow Control timeout.");

    return false;
}

// ============================================================
// SEND ISO-TP FIRST FRAME
// ============================================================

bool sendIsoTpFirstFrame()
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x10;
    message.data[1] = 0x0C;

    message.data[2] = 0x22;
    message.data[3] = 0x11;
    message.data[4] = 0x22;
    message.data[5] = 0x33;
    message.data[6] = 0x44;
    message.data[7] = 0x55;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        Serial.println("ERROR: First Frame transmission failed.");
        return false;
    }

    pushCanFeed("TX", message);

    Serial.println();
    Serial.println("================================");
    Serial.println("ISO-TP FIRST FRAME TEST");
    Serial.println("================================");

    return true;
}

// ============================================================
// SEND ISO-TP CONSECUTIVE FRAME #1
// ============================================================

bool sendIsoTpConsecutiveFrame()
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x21;
    message.data[1] = 0x66;
    message.data[2] = 0x77;
    message.data[3] = 0x88;
    message.data[4] = 0x99;
    message.data[5] = 0xAA;
    message.data[6] = 0xBB;
    message.data[7] = 0x00;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        Serial.println("ERROR: Consecutive Frame transmission failed.");
        return false;
    }

    pushCanFeed("TX", message);

    Serial.println();
    Serial.println("================================");
    Serial.println("ISO-TP CONSECUTIVE FRAME TEST");
    Serial.println("================================");

    return true;
}

// ============================================================
// UDS SERVICE 0x10 - DIAGNOSTIC SESSION CONTROL
// ============================================================

bool sendUdsDiagnosticSessionControl()
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x02;
    message.data[1] = 0x10;
    message.data[2] = 0x01;

    for (uint8_t i = 3; i < 8; i++)
        message.data[i] = 0x00;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        Serial.println("ERROR: UDS request transmission failed.");
        return false;
    }

    pushCanFeed("TX", message);

    Serial.println();
    Serial.println("================================");
    Serial.println("UDS DIAGNOSTIC SESSION CONTROL");
    Serial.println("================================");
    Serial.println("UDS SERVICE: 0x10");
    Serial.println("SUB-FUNCTION: 0x01 (Default Session)");

    return true;
}

// ============================================================
// RECEIVE UDS 0x10 RESPONSE
// ============================================================

bool receiveUdsDiagnosticSessionResponse()
{
    twai_message_t message;

    unsigned long startTime = millis();

    while (millis() - startTime < 1000)
    {
        esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(50));

        if (result != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier != ECU_RESPONSE_ID)
        {
            printCANFrame(message);
            continue;
        }

        if (message.data_length_code < 3)
            continue;

        if (message.data[0] != 0x02)
            continue;

        if (message.data[1] == 0x7F)
        {
            Serial.println("UDS NEGATIVE RESPONSE");
            printCANFrame(message);
            return false;
        }

        if (message.data[1] != 0x50)
            continue;

        if (message.data[2] != 0x01)
            continue;

        printCANFrame(message);

        Serial.println("UDS 0x10 DEFAULT SESSION: SUCCESS");

        return true;
    }

    Serial.println("ERROR: UDS response timeout.");

    return false;
}

// ============================================================
// UDS 0x22 - VIN
// ============================================================

bool sendUdsReadVinRequest()
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x03;
    message.data[1] = 0x22;
    message.data[2] = 0xF1;
    message.data[3] = 0x90;

    for (uint8_t i = 4; i < 8; i++)
        message.data[i] = 0x00;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        Serial.println("ERROR: VIN request transmission failed.");
        return false;
    }

    pushCanFeed("TX", message);

    Serial.println();
    Serial.println("================================");
    Serial.println("UDS READ VIN REQUEST");
    Serial.println("================================");

    return true;
}

bool sendVinFlowControl()
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x30;
    message.data[1] = 0x00;
    message.data[2] = 0x00;

    for (uint8_t i = 3; i < 8; i++)
        message.data[i] = 0x00;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        Serial.println("ERROR: VIN Flow Control transmission failed.");
        return false;
    }

    pushCanFeed("TX", message);

    return true;
}

bool receiveUdsVinResponse()
{
    twai_message_t message;

    uint8_t vinPayload[20] = {};

    uint16_t totalLength = 0;
    uint16_t receivedLength = 0;

    uint8_t expectedSequenceNumber = 1;

    unsigned long startTime = millis();

    // ----- STEP 1: First Frame -----

    while (millis() - startTime < 1000)
    {
        esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(50));

        if (result != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier != ECU_RESPONSE_ID)
        {
            printCANFrame(message);
            continue;
        }

        if (message.data_length_code < 2)
            continue;

        uint8_t frameType = (message.data[0] >> 4) & 0x0F;

        if (frameType != 0x01)
            continue;

        totalLength = ((message.data[0] & 0x0F) << 8) | message.data[1];

        if (totalLength != 20)
        {
            Serial.println("ERROR: Unexpected VIN payload length.");
            return false;
        }

        if (message.data_length_code < 8)
        {
            Serial.println("ERROR: VIN First Frame DLC too short.");
            return false;
        }

        for (uint8_t i = 0; i < 6; i++)
        {
            vinPayload[receivedLength++] = message.data[2 + i];
        }

        printCANFrame(message);

        if (!sendVinFlowControl())
            return false;

        break;
    }

    if (receivedLength == 0 || totalLength == 0)
    {
        Serial.println("ERROR: VIN First Frame timeout.");
        return false;
    }

    // ----- STEP 3: Consecutive Frames -----

    startTime = millis();

    while (receivedLength < totalLength)
    {
        if (millis() - startTime >= 1000)
        {
            Serial.println("ERROR: VIN Consecutive Frame timeout.");
            return false;
        }

        esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(50));

        if (result != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier != ECU_RESPONSE_ID)
        {
            printCANFrame(message);
            continue;
        }

        if (message.data_length_code < 2)
            continue;

        uint8_t frameType = (message.data[0] >> 4) & 0x0F;

        if (frameType != 0x02)
            continue;

        uint8_t sequenceNumber = message.data[0] & 0x0F;

        if (sequenceNumber != expectedSequenceNumber)
        {
            Serial.println("ERROR: Wrong VIN sequence number.");
            return false;
        }

        printCANFrame(message);

        for (uint8_t i = 1;
             i < message.data_length_code;
             i++)
        {
            if (receivedLength >= totalLength)
                break;

            vinPayload[receivedLength++] = message.data[i];
        }

        expectedSequenceNumber = (expectedSequenceNumber + 1) & 0x0F;

        startTime = millis();
    }

    // ----- STEP 4: Validate -----

    if (vinPayload[0] != 0x62 ||
        vinPayload[1] != 0xF1 ||
        vinPayload[2] != 0x90)
    {
        Serial.println("ERROR: Invalid UDS VIN response header.");
        return false;
    }

    // ----- STEP 5: Extract -----

    lastVin = "";

    for (uint8_t i = 3; i < 20; i++)
    {
        lastVin += (char)vinPayload[i];
    }

    Serial.print("VIN: ");
    Serial.println(lastVin);

    return true;
}

// ============================================================
// UDS 0x19 - READ DTC
// ============================================================

bool sendUdsReadDtcRequest()
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x03;
    message.data[1] = 0x19;
    message.data[2] = 0x02;
    message.data[3] = 0x08;

    for (uint8_t i = 4; i < 8; i++)
        message.data[i] = 0x00;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        Serial.println("ERROR: DTC request transmission failed.");
        return false;
    }

    pushCanFeed("TX", message);

    Serial.println();
    Serial.println("================================");
    Serial.println("UDS READ DTC INFORMATION REQUEST");
    Serial.println("================================");

    return true;
}

bool sendDtcFlowControl()
{
    twai_message_t message = {};

    message.identifier = OBD_REQUEST_ID;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x30;
    message.data[1] = 0x00;
    message.data[2] = 0x00;

    for (uint8_t i = 3; i < 8; i++)
        message.data[i] = 0x00;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
        return false;

    pushCanFeed("TX", message);

    return true;
}

bool receiveUdsDtcResponse()
{
    twai_message_t message;

    unsigned long startTime = millis();

    while (millis() - startTime < 2000)
    {
        if (twai_receive(&message, pdMS_TO_TICKS(100)) != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier != ECU_RESPONSE_ID ||
            message.data_length_code < 8)
        {
            continue;
        }

        if ((message.data[0] & 0xF0) != 0x10)
            continue;

        if (message.data[2] != 0x59 ||
            message.data[3] != 0x02)
        {
            continue;
        }

        uint16_t payloadLength =
            ((message.data[0] & 0x0F) << 8) | message.data[1];

        if (payloadLength != 9)
            return false;

        uint8_t payload[9];

        for (uint8_t i = 0; i < 6; i++)
            payload[i] = message.data[i + 2];

        printCANFrame(message);

        if (!sendDtcFlowControl())
            return false;

        startTime = millis();

        while (millis() - startTime < 2000)
        {
            if (twai_receive(&message, pdMS_TO_TICKS(100)) != ESP_OK)
            {
                handleWebDashboard();
                continue;
            }

            handleWebDashboard();

            if (message.identifier != ECU_RESPONSE_ID ||
                message.data_length_code < 5)
            {
                continue;
            }

            if (message.data[0] != 0x21)
                continue;

            printCANFrame(message);

            payload[6] = message.data[1];
            payload[7] = message.data[2];
            payload[8] = message.data[3];

            lastDtcStatus = payload[8];

            if ((payload[8] & 0x09) != 0 &&
                activeDtcCount < 4)
            {
                activeDtcs[activeDtcCount++] = "P0118";
            }

            return true;
        }

        return false;
    }

    return false;
}

// ============================================================
// PREDICTIVE DTC DISPLAY
// ============================================================

String buildDtcPrediction()
{
    if (!dtcPredictionActive)
    {
        return
            "No DTC analysis yet.\n"
            "Press READ DTC to start predictive analysis.";
    }

    unsigned long elapsed = millis() - dtcPredictionStartMillis;

    unsigned long remaining =
        (elapsed < DTC_MONITOR_WINDOW_MS)
            ? (DTC_MONITOR_WINDOW_MS - elapsed)
            : 0;

    unsigned long remainingMinutes = remaining / 60000UL;
    unsigned long remainingSeconds = (remaining % 60000UL) / 1000UL;

    String statusText;

    if ((lastDtcStatus & 0x01) != 0)
        statusText = "ACTIVE";
    else if ((lastDtcStatus & 0x08) != 0)
        statusText = "STORED";
    else
        statusText = "REPORTED";

    String result;

    result += "CURRENT FAULT\n";
    result += "P0118 - Engine Coolant Temperature Sensor\n";
    result += "Signal High\n";
    result += "Status: ";
    result += statusText;
    result += "\n\n";

    result += "DRIVE STATUS\n";
    result += "Vehicle may remain operational temporarily.\n";
    result += "Coolant-temperature signal is unreliable.\n";
    result += "\n";

    result += "PREDICTIVE ANALYSIS\n";
    result += "Persistent P0118 may lead to additional\n";
    result += "temperature-management or correlation faults.\n";
    result += "\n";

    result += "Potential secondary conditions:\n";
    result += "- P011B - ECT / IAT correlation\n";
    result += "- P0128 - Coolant temperature regulation\n";
    result += "\n";

    if (remaining > 0)
    {
        result += "MONITORING WINDOW\n";
        result += "Approx. ";
        result += String(remainingMinutes);
        result += " min ";

        if (remainingSeconds < 10)
            result += "0";

        result += String(remainingSeconds);
        result += " remaining\n";
        result += "Prototype estimation only.\n";
        result += "\n";

        result += "ESCALATION IF CONDITION WORSENS\n";
        result += "STOP VEHICLE SAFELY\n";
        result += "TURN OFF ENGINE\n";
        result += "Call +213 782 773 051";
    }
    else
    {
        result += "CRITICAL ESCALATION\n";
        result += "Monitoring window exceeded.\n";
        result += "STOP VEHICLE SAFELY\n";
        result += "TURN OFF ENGINE\n";
        result += "Call +213 782 773 051";
    }

    result += "\n\nPrototype estimation - not a certified";
    result += "\nsafe-driving limit.";

    return result;
}

// ============================================================
// DECODE ONE PID FOR THE LIVE ZONE
// ============================================================
//
// Returns the decoded physical value (as float).
// Units are decided on the JS side.
//
// If decoding is impossible, returns 0.

float decodePidValue(const PidReading &r)
{
    if (!r.valid)
        return 0.0f;

    // data[0] = SF length, data[1] = 0x41, data[2] = pid,
    // data[3..] = payload bytes.

    uint8_t A = (r.dlc >= 4) ? r.data[3] : 0;
    uint8_t B = (r.dlc >= 5) ? r.data[4] : 0;

    switch (r.pid)
    {
        case 0x04: return A * 100.0f / 255.0f;
        case 0x05: return (float)A - 40.0f;
        case 0x06: return ((float)A - 128.0f) * 100.0f / 128.0f;
        case 0x07: return ((float)A - 128.0f) * 100.0f / 128.0f;
        case 0x0A: return A * 3.0f;
        case 0x0B: return (float)A;
        case 0x0C: return (((float)A * 256.0f) + B) / 4.0f;
        case 0x0D: return (float)A;
        case 0x0E: return ((float)A / 2.0f) - 64.0f;
        case 0x0F: return (float)A - 40.0f;
        case 0x10: return (((float)A * 256.0f) + B) / 100.0f;
        case 0x11: return A * 100.0f / 255.0f;
        case 0x1C: return (float)A;
        case 0x1F: return ((float)A * 256.0f) + B;
        case 0x21: return ((float)A * 256.0f) + B;
        case 0x2F: return A * 100.0f / 255.0f;
        case 0x31: return ((float)A * 256.0f) + B;
        case 0x33: return (float)A;
        case 0x42: return (((float)A * 256.0f) + B) / 1000.0f;
        case 0x46: return (float)A - 40.0f;
        case 0x51: return (float)A;
        case 0x5C: return (float)A - 40.0f;
        case 0x5E: return (((float)A * 256.0f) + B) / 20.0f;
    }

    return 0.0f;
}

// ============================================================
// BUILD LIVE JSON
// ============================================================
//
// {"04": 15.2, "05": 110, "0C": 3000, ...}
//
// Order = same as pids[].

String buildLiveJSON()
{
    JsonDocument doc;

    for (uint8_t i = 0; i < PID_COUNT; i++)
    {
        char key[3];
        snprintf(key, sizeof(key), "%02X", pidReadings[i].pid);

        if (pidReadings[i].valid)
        {
            doc[key] = decodePidValue(pidReadings[i]);
        }
        else
        {
            doc[key] = nullptr;
        }
    }

    String out;
    serializeJson(doc, out);

    return out;
}

// ============================================================
// UPDATE DASHBOARD
// ============================================================

void updateDashboard()
{
    // ----------------------------------------------
    // Decode + package the LIVE zone as a JSON object
    // ----------------------------------------------

    String liveJSON = buildLiveJSON();

    // ----------------------------------------------
    // Diagnostic event text
    // ----------------------------------------------

    lastDiagnosticEvent = "PID polling complete.";

    if (activeDtcCount > 0)
    {
        lastDiagnosticEvent += "\nDTC: ";

        for (uint8_t i = 0; i < activeDtcCount; i++)
        {
            if (i > 0)
                lastDiagnosticEvent += ", ";

            lastDiagnosticEvent += activeDtcs[i];
        }
    }

    // ----------------------------------------------
    // Push everything to WebDashboard.cpp
    // ----------------------------------------------

    updateDashboardData(
        liveJSON,
        latestReadingsJSON,
        buildDtcPrediction(),
        getCanFeedString(),
        lastDiagnosticEvent
    );
}

// ============================================================
// DASHBOARD REQUEST HELPERS
// ============================================================

String getPidName(uint8_t pid)
{
    switch (pid)
    {
        case 0x04: return "Engine load";
        case 0x05: return "Coolant";
        case 0x06: return "STFT";
        case 0x07: return "LTFT";
        case 0x0A: return "Fuel pressure";
        case 0x0B: return "MAP";
        case 0x0C: return "RPM";
        case 0x0D: return "Speed";
        case 0x0E: return "Timing advance";
        case 0x0F: return "Intake temp";
        case 0x10: return "MAF";
        case 0x11: return "Throttle";
        case 0x1C: return "OBD standard";
        case 0x1F: return "Engine runtime";
        case 0x21: return "Distance with MIL";
        case 0x2F: return "Fuel level";
        case 0x31: return "Distance since clear";
        case 0x33: return "Barometric";
        case 0x42: return "Module voltage";
        case 0x46: return "Ambient temp";
        case 0x51: return "Fuel type";
        case 0x5C: return "Oil temp";
        case 0x5E: return "Fuel rate";
    }

    return "Unknown PID";
}

String getPidResultText(uint8_t pid, const PidReading &reading)
{
    if (!reading.valid)
        return "No valid ECU response.";

    float value = decodePidValue(reading);

    switch (pid)
    {
        case 0x0C: return "RPM = " + String(value, 0);
        case 0x0D: return "Speed = " + String(value, 0) + " km/h";
        case 0x05: return "Coolant = " + String(value, 0) + " C";
        case 0x11: return "Throttle = " + String(value, 1) + " %";
    }

    return getPidName(pid) + " = " + String(value, 2);
}

bool performDashboardPidRequest(uint8_t pid)
{
    int index = -1;

    for (uint8_t i = 0; i < PID_COUNT; i++)
    {
        if (pids[i] == pid)
        {
            index = i;
            break;
        }
    }

    if (index < 0)
        return false;

    Serial.println();
    Serial.println("================================");
    Serial.print("DASHBOARD REQUEST: READ ");
    Serial.println(getPidName(pid));
    Serial.println("================================");

    sendOBDRequest(pid);

    bool success =
        receiveECUResponse(pid, pidReadings[index]);

    if (!success)
    {
        updateDashboardRequestResult(
            getPidName(pid),
            "OBD-II request failed or timed out."
        );

        return false;
    }

    // Refresh the readings JSON too, so the JSON zone
    // reflects this new value immediately.
    latestReadingsJSON = buildReadingsJSON();
    updateDashboardJSON(latestReadingsJSON);
    pushReadingsJSON(latestReadingsJSON);

    String resultText = getPidResultText(pid, pidReadings[index]);

    updateDashboard();

    updateDashboardRequestResult(
        getPidName(pid),
        resultText
    );

    return true;
}

bool performDashboardVinRequest()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("DASHBOARD REQUEST: READ VIN");
    Serial.println("================================");

    if (!sendUdsReadVinRequest())
    {
        updateDashboardRequestResult(
            "READ VIN",
            "UDS 0x22 VIN request transmission failed."
        );
        return false;
    }

    if (!receiveUdsVinResponse())
    {
        updateDashboardRequestResult(
            "READ VIN",
            "UDS 0x22 VIN response failed or timed out."
        );
        return false;
    }

    String result = "VIN: " + lastVin + "\nUDS: 0x22\nDID: F190";

    updateDashboardRequestResult("READ VIN", result);

    return true;
}

bool performDashboardDtcRequest()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("DASHBOARD REQUEST: READ DTC");
    Serial.println("================================");

    activeDtcCount = 0;
    lastDtcStatus = 0;

    if (!sendUdsReadDtcRequest() ||
        !receiveUdsDtcResponse())
    {
        updateDashboardRequestResult(
            "READ DTC",
            "UDS 0x19 DTC request failed or timed out."
        );
        return false;
    }

    bool p0118Found = false;

    for (uint8_t i = 0; i < activeDtcCount; i++)
    {
        if (activeDtcs[i] == "P0118")
        {
            p0118Found = true;
            break;
        }
    }

    if (p0118Found)
    {
        dtcPredictionActive = true;
        dtcPredictionStartMillis = millis();

        lastDtcPrediction = buildDtcPrediction();
    }
    else
    {
        dtcPredictionActive = false;

        lastDtcPrediction =
            "NO PREDICTIVE FAULT CHAIN DETECTED.\n"
            "No P0118 was reported by the ECU.";
    }

    updateDashboard();

    String result = "DTC response received.\n";

    if (activeDtcCount > 0)
    {
        result += "DTC: ";

        for (uint8_t i = 0; i < activeDtcCount; i++)
        {
            if (i > 0)
                result += ", ";

            result += activeDtcs[i];
        }
    }
    else
    {
        result += "No reported DTC.";
    }

    char statusText[8];
    snprintf(statusText, sizeof(statusText), "%02X", lastDtcStatus);

    result += "\nStatus: 0x";
    result += statusText;

    updateDashboardRequestResult("READ DTC", result);

    return true;
}

bool performDashboardSessionRequest()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("DASHBOARD REQUEST: DEFAULT SESSION");
    Serial.println("================================");

    if (!sendUdsDiagnosticSessionControl())
    {
        updateDashboardRequestResult(
            "DEFAULT SESSION",
            "UDS 0x10 request transmission failed."
        );
        return false;
    }

    if (!receiveUdsDiagnosticSessionResponse())
    {
        updateDashboardRequestResult(
            "DEFAULT SESSION",
            "UDS 0x10 response failed or timed out."
        );
        return false;
    }

    updateDashboardRequestResult(
        "DEFAULT SESSION",
        "UDS 0x10\nResponse: 50 01\nDefault Diagnostic Session confirmed."
    );

    return true;
}

void processDashboardRequest()
{
    String request = getPendingDashboardRequest();

    if (request.length() == 0)
        return;

    if (request == "rpm")
        performDashboardPidRequest(0x0C);
    else if (request == "speed")
        performDashboardPidRequest(0x0D);
    else if (request == "coolant")
        performDashboardPidRequest(0x05);
    else if (request == "throttle")
        performDashboardPidRequest(0x11);
    else if (request == "vin")
        performDashboardVinRequest();
    else if (request == "dtc")
        performDashboardDtcRequest();
    else if (request == "session")
        performDashboardSessionRequest();
    else
    {
        Serial.print("Unknown dashboard request: ");
        Serial.println(request);

        updateDashboardRequestResult(
            request,
            "Unknown dashboard request."
        );
    }
}

// ============================================================
// WAIT WITH DASHBOARD
// ============================================================

void waitWithDashboard(unsigned long duration)
{
    unsigned long startTime = millis();

    while (millis() - startTime < duration)
    {
        handleWebDashboard();
        delay(10);
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("      OBD-II CAN SNIFFER");
    Serial.println("================================");
    Serial.println("CAN bitrate: 500 kbps");
    Serial.println("RX pin: GPIO16");
    Serial.println("TX pin: GPIO17");

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN,
            (gpio_num_t)CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result =
        twai_driver_install(&g_config, &t_config, &f_config);

    if (result != ESP_OK)
    {
        Serial.println("ERROR: TWAI driver installation failed.");
        return;
    }

    Serial.println("TWAI driver installed.");

    result = twai_start();

    if (result != ESP_OK)
    {
        Serial.println("ERROR: TWAI start failed.");
        return;
    }

    Serial.println("CAN controller started.");

    startWebDashboard();
    startTransmitServer();

    Serial.println();
    Serial.println("================================");
    Serial.println("       OBD-II TESTER READY");
    Serial.println("================================");

    // ------------------------------------------------
    // Startup tests
    // ------------------------------------------------

    if (!sendUdsReadVinRequest())
    {
        Serial.println("UDS VIN REQUEST FAILED.");
    }
    else if (!receiveUdsVinResponse())
    {
        Serial.println("UDS VIN TEST FAILED.");
    }

    waitWithDashboard(500);

    if (!sendUdsReadDtcRequest() ||
        !receiveUdsDtcResponse())
    {
        Serial.println("UDS 0x19 TEST FAILED.");
    }

    waitWithDashboard(500);

    if (!sendIsoTpFirstFrame())
    {
        Serial.println("ISO-TP FF FAILED.");
    }
    else if (!waitForIsoTpFlowControl())
    {
        Serial.println("ISO-TP FC FAILED.");
    }
    else if (!sendIsoTpConsecutiveFrame())
    {
        Serial.println("ISO-TP CF FAILED.");
    }

    waitWithDashboard(500);

    if (!sendUdsDiagnosticSessionControl())
    {
        Serial.println("UDS 0x10 REQUEST FAILED.");
    }
    else if (!receiveUdsDiagnosticSessionResponse())
    {
        Serial.println("UDS 0x10 TEST FAILED.");
    }

    Serial.println();
    Serial.println("ALL STARTUP TESTS COMPLETE");

    // Kick off the live poll timer immediately.
    lastLivePollTime = millis() - LIVE_POLL_INTERVAL;
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    handleWebDashboard();

    handleTransmitClient();

    processDashboardRequest();

    if (millis() - lastLivePollTime >= LIVE_POLL_INTERVAL)
    {
        lastLivePollTime = millis();

        pollAllPIDs();
    }

    handleWebDashboard();

    handleTransmitClient();

    delay(2);
}