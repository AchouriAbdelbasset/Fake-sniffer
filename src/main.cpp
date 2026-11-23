#include <Arduino.h>
#include "driver/twai.h"
#include <ArduinoJson.h>
#include <time.h>
#include "WebDashboard.h"
#include "transmit.h"
#include "gsm_call.h"
#include "GPS.h"
// i cancelled the sim work to test the ps, in case i forgot idid that the ia responsable to test will see this and declare !
// ============================================================
// ESP32 TWAI PINS
// ============================================================
#define CAN_RX_PIN 16
#define CAN_TX_PIN 17

#define OBD_REQUEST_ID  0x7DF
#define ECU_RESPONSE_ID 0x7E8

// ============================================================
// PID TABLE
// ============================================================
const uint8_t pids[] =
{
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
    0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
    0x1C, 0x1F, 0x21,
    0x22, 0x23,
    0x2C, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x32, 0x33,
    0x3C,
    0x42, 0x43, 0x45, 0x46,
    0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C,
    0x4D, 0x4E,
    0x51,
    0x5C, 0x5D, 0x5E
};
const uint8_t PID_COUNT = sizeof(pids) / sizeof(pids[0]);

const char *VEHICLE_NAME = "VW Golf 8 (2023)";

struct PidReading
{
    uint8_t pid;
    uint8_t data[8];
    uint8_t dlc;
    bool valid;
};

PidReading pidReadings[PID_COUNT];

// ============================================================
// DISTANCE TRACKING STATE (For the 100m UI)
// ============================================================
float obdDistMeters = 0.0f;   // Integrated via Speed PID
float udsDistMeters = -1.0f;  // Read from UDS Odometer
unsigned long lastSpeedTime = 0;

// ============================================================
// DIAGNOSTIC CAN FEED
// ============================================================
#define CAN_FEED_SIZE 40
#define CAN_FEED_LINE_LENGTH 48
char canFeed[CAN_FEED_SIZE][CAN_FEED_LINE_LENGTH];
uint8_t canFeedHead = 0;
uint8_t canFeedCount = 0;

// ============================================================
// ACTION LISTENER
// ============================================================
#define ACTION_FEED_SIZE 60
#define ACTION_LINE_LENGTH 64
#define ACTION_MIN_INTERVAL_MS 500
#define CAN_TRACKER_SIZE 64

char actionFeed[ACTION_FEED_SIZE][ACTION_LINE_LENGTH];
uint8_t actionFeedHead = 0;
uint8_t actionFeedCount = 0;

struct CanTracker
{
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    bool active;
    unsigned long lastPush;
};
CanTracker canTrackers[CAN_TRACKER_SIZE] = {};

String activeDtcs[4];
uint8_t activeDtcCount = 0;

uint32_t syntheticUnixTime = 1789225200;

// ============================================================
// DASHBOARD STATE
// ============================================================
String lastDiagnosticEvent = "Dashboard started.";
const unsigned long LIVE_POLL_INTERVAL = 0;
unsigned long lastLivePollTime = 0;

String lastVin = "Not read yet";
uint8_t lastDtcStatus = 0;
String latestReadingsJSON = "{}";

void updateDashboard();

// ============================================================
// HELPERS
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
// ACTION FEED
// ============================================================
void pushActionFeed(const char *tag, const twai_message_t &msg)
{
    char line[ACTION_LINE_LENGTH];
    int n = snprintf(line, sizeof(line), "%s %03X [%d] ",
                     tag, (unsigned)msg.identifier, msg.data_length_code);
    for (int i = 0; i < msg.data_length_code && n < (int)sizeof(line) - 4; i++)
        n += snprintf(line + n, sizeof(line) - n, "%02X ", msg.data[i]);
    strncpy(actionFeed[actionFeedHead], line, ACTION_LINE_LENGTH - 1);
    actionFeed[actionFeedHead][ACTION_LINE_LENGTH - 1] = '\0';
    actionFeedHead = (actionFeedHead + 1) % ACTION_FEED_SIZE;
    if (actionFeedCount < ACTION_FEED_SIZE) actionFeedCount++;
}

void trackActionFrame(const twai_message_t &msg)
{
    unsigned long now = millis();
    int slot = -1;
    for (int i = 0; i < CAN_TRACKER_SIZE; i++)
    {
        if (canTrackers[i].active && canTrackers[i].id == msg.identifier)
        {
            slot = i;
            break;
        }
    }

    if (slot >= 0)
    {
        bool changed = (canTrackers[slot].dlc != msg.data_length_code);
        if (!changed)
        {
            for (int j = 0; j < msg.data_length_code; j++)
            {
                if (canTrackers[slot].data[j] != msg.data[j]) { changed = true; break; }
            }
        }
        if (changed && (now - canTrackers[slot].lastPush) >= ACTION_MIN_INTERVAL_MS)
        {
            pushActionFeed("CHG", msg);
            canTrackers[slot].lastPush = now;
        }
        canTrackers[slot].dlc = msg.data_length_code;
        memcpy(canTrackers[slot].data, msg.data, msg.data_length_code);
    }
    else
    {
        int freeSlot = -1;
        for (int i = 0; i < CAN_TRACKER_SIZE; i++)
        {
            if (!canTrackers[i].active) { freeSlot = i; break; }
        }
        if (freeSlot < 0)
        {
            freeSlot = 0;
            for (int i = 1; i < CAN_TRACKER_SIZE; i++)
                if (canTrackers[i].lastPush < canTrackers[freeSlot].lastPush)
                    freeSlot = i;
        }
        canTrackers[freeSlot].active = true;
        canTrackers[freeSlot].id = msg.identifier;
        canTrackers[freeSlot].dlc = msg.data_length_code;
        memcpy(canTrackers[freeSlot].data, msg.data, msg.data_length_code);
        canTrackers[freeSlot].lastPush = now;
        pushActionFeed("NEW", msg);
    }
}

String getActionFeedString()
{
    String out;
    out.reserve(ACTION_FEED_SIZE * ACTION_LINE_LENGTH);
    uint8_t start = (actionFeedCount < ACTION_FEED_SIZE) ? 0 : actionFeedHead;
    for (uint8_t i = 0; i < actionFeedCount; i++)
    {
        uint8_t idx = (start + i) % ACTION_FEED_SIZE;
        out += actionFeed[idx];
        out += "\n";
    }
    if (out.length() == 0) out = "Waiting for CAN activity...\n";
    return out;
}

// ============================================================
// DIAGNOSTIC FEED
// ============================================================
void pushCanFeed(const char *direction, const twai_message_t &message)
{
    char line[CAN_FEED_LINE_LENGTH];
    int written = snprintf(line, sizeof(line), "%s ID:%03X [%d] ",
                           direction, message.identifier, message.data_length_code);
    for (int i = 0; i < message.data_length_code && written < (int)sizeof(line) - 4; i++)
        written += snprintf(line + written, sizeof(line) - written, "%02X ", message.data[i]);

    strncpy(canFeed[canFeedHead], line, CAN_FEED_LINE_LENGTH - 1);
    canFeed[canFeedHead][CAN_FEED_LINE_LENGTH - 1] = '\0';
    canFeedHead = (canFeedHead + 1) % CAN_FEED_SIZE;
    if (canFeedCount < CAN_FEED_SIZE) canFeedCount++;

    if (direction[0] == 'R') trackActionFrame(message);
}

String getCanFeedString()
{
    String out;
    out.reserve(CAN_FEED_SIZE * CAN_FEED_LINE_LENGTH);
    uint8_t start = (canFeedCount < CAN_FEED_SIZE) ? 0 : canFeedHead;
    for (uint8_t i = 0; i < canFeedCount; i++)
    {
        uint8_t idx = (start + i) % CAN_FEED_SIZE;
        out += canFeed[idx];
        out += "\n";
    }
    if (out.length() == 0) out = "Waiting for CAN frames...\n";
    return out;
}

// ============================================================
// JSON EMITTER
// ============================================================
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
            uint8_t payloadLength = pidReadings[i].data[0] & 0x0F;
            entry["response"] = hexBytesToString(pidReadings[i].data, 1, payloadLength);
        }
        else entry["response"] = "";
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
    message.data_length_code = 8;
    message.data[0] = 0x02;
    message.data[1] = 0x01;
    message.data[2] = pid;
    for (uint8_t i = 3; i < 8; i++) message.data[i] = 0x00;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK)
        pushCanFeed("TX", message);
}

void printCANFrame(const twai_message_t &message)
{
    pushCanFeed("RX", message);
    Serial.print("ID: 0x");
    if (message.identifier < 0x100) Serial.print("0");
    Serial.print(message.identifier, HEX);
    Serial.print(" DATA: ");
    for (int i = 0; i < message.data_length_code; i++)
    {
        if (message.data[i] < 0x10) Serial.print("0");
        Serial.print(message.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

// ============================================================
// RECEIVE ECU RESPONSE
// ============================================================
bool receiveECUResponse(uint8_t requestedPID, PidReading &outReading)
{
    outReading.pid = requestedPID;
    outReading.valid = false;

    twai_message_t message;
    unsigned long startTime = millis();

    while (millis() - startTime < 30)
    {
        if (twai_receive(&message, pdMS_TO_TICKS(20)) != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }
        handleWebDashboard();

        if (message.identifier != ECU_RESPONSE_ID) { printCANFrame(message); continue; }
        if (message.data_length_code < 3) continue;
        if (message.data[1] != 0x41) continue;
        if (message.data[2] != requestedPID) continue;

        printCANFrame(message);
        outReading.dlc = message.data_length_code;
        memcpy(outReading.data, message.data, message.data_length_code);
        outReading.valid = true;
        return true;
    }
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

        latestReadingsJSON = buildReadingsJSON();
        updateDashboardJSON(latestReadingsJSON);
        pushReadingsJSON(latestReadingsJSON);
        updateDashboard();

        delay(2);
        handleWebDashboard();
    }

    latestReadingsJSON = buildReadingsJSON();
    updateDashboardJSON(latestReadingsJSON);
    pushReadingsJSON(latestReadingsJSON);
    updateDashboard();
}

// ============================================================
// UDS ODOMETER / SGW / PID HELPERS
// ============================================================
bool waitForIsoTpFlowControl()
{
    twai_message_t message;
    unsigned long startTime = millis();
    while (millis() - startTime < 1000)
    {
        if (twai_receive(&message, pdMS_TO_TICKS(50)) != ESP_OK) { handleWebDashboard(); continue; }
        handleWebDashboard();
        if (message.identifier != ECU_RESPONSE_ID) { printCANFrame(message); continue; }
        if (message.data_length_code < 3) continue;
        if (((message.data[0] >> 4) & 0x0F) != 0x03) continue;
        printCANFrame(message);
        return true;
    }
    return false;
}

bool sendUdsReadVinRequest()
{
    twai_message_t m = {};
    m.identifier = OBD_REQUEST_ID; m.data_length_code = 8;
    m.data[0] = 0x03; m.data[1] = 0x22; m.data[2] = 0xF1; m.data[3] = 0x90;
    if (twai_transmit(&m, pdMS_TO_TICKS(100)) != ESP_OK) return false;
    pushCanFeed("TX", m);
    return true;
}

bool sendVinFlowControl()
{
    twai_message_t m = {};
    m.identifier = OBD_REQUEST_ID; m.data_length_code = 8; m.data[0] = 0x30;
    if (twai_transmit(&m, pdMS_TO_TICKS(100)) != ESP_OK) return false;
    pushCanFeed("TX", m);
    return true;
}

bool receiveUdsVinResponse()
{
    twai_message_t message;
    uint8_t vinPayload[20] = {};
    uint16_t totalLength = 0, receivedLength = 0;
    uint8_t expectedSequenceNumber = 1;
    unsigned long startTime = millis();

    while (millis() - startTime < 1000)
    {
        if (twai_receive(&message, pdMS_TO_TICKS(50)) != ESP_OK) { handleWebDashboard(); continue; }
        handleWebDashboard();
        if (message.identifier != ECU_RESPONSE_ID) { printCANFrame(message); continue; }
        if (message.data_length_code < 2) continue;
        if (((message.data[0] >> 4) & 0x0F) != 0x01) continue;
        totalLength = ((message.data[0] & 0x0F) << 8) | message.data[1];
        if (totalLength != 20) return false;
        for (uint8_t i = 0; i < 6; i++) vinPayload[receivedLength++] = message.data[2 + i];
        printCANFrame(message);
        if (!sendVinFlowControl()) return false;
        break;
    }
    if (receivedLength == 0 || totalLength == 0) return false;

    startTime = millis();
    while (receivedLength < totalLength)
    {
        if (millis() - startTime >= 1000) return false;
        if (twai_receive(&message, pdMS_TO_TICKS(50)) != ESP_OK) { handleWebDashboard(); continue; }
        handleWebDashboard();
        if (message.identifier != ECU_RESPONSE_ID) { printCANFrame(message); continue; }
        if (message.data_length_code < 2) continue;
        if (((message.data[0] >> 4) & 0x0F) != 0x02) continue;
        if ((message.data[0] & 0x0F) != expectedSequenceNumber) return false;
        printCANFrame(message);
        for (uint8_t i = 1; i < message.data_length_code; i++)
        {
            if (receivedLength >= totalLength) break;
            vinPayload[receivedLength++] = message.data[i];
        }
        expectedSequenceNumber = (expectedSequenceNumber + 1) & 0x0F;
        startTime = millis();
    }
    if (vinPayload[0] != 0x62 || vinPayload[1] != 0xF1 || vinPayload[2] != 0x90) return false;

    lastVin = "";
    for (uint8_t i = 3; i < 20; i++) lastVin += (char)vinPayload[i];
    return true;
}

bool sendUdsReadDtcRequest()
{
    twai_message_t m = {};
    m.identifier = OBD_REQUEST_ID; m.data_length_code = 8;
    m.data[0] = 0x03; m.data[1] = 0x19; m.data[2] = 0x02; m.data[3] = 0x08;
    if (twai_transmit(&m, pdMS_TO_TICKS(100)) != ESP_OK) return false;
    pushCanFeed("TX", m);
    return true;
}

bool sendDtcFlowControl()
{
    twai_message_t m = {};
    m.identifier = OBD_REQUEST_ID; m.data_length_code = 8; m.data[0] = 0x30;
    if (twai_transmit(&m, pdMS_TO_TICKS(100)) != ESP_OK) return false;
    pushCanFeed("TX", m);
    return true;
}

bool receiveUdsDtcResponse()
{
    twai_message_t message;
    unsigned long startTime = millis();
    while (millis() - startTime < 2000)
    {
        if (twai_receive(&message, pdMS_TO_TICKS(100)) != ESP_OK) { handleWebDashboard(); continue; }
        handleWebDashboard();
        if (message.identifier != ECU_RESPONSE_ID || message.data_length_code < 8) continue;
        if ((message.data[0] & 0xF0) != 0x10) continue;
        if (message.data[2] != 0x59 || message.data[3] != 0x02) continue;

        uint8_t payload[9];
        for (uint8_t i = 0; i < 6; i++) payload[i] = message.data[i + 2];
        printCANFrame(message);
        if (!sendDtcFlowControl()) return false;

        startTime = millis();
        while (millis() - startTime < 2000)
        {
            if (twai_receive(&message, pdMS_TO_TICKS(100)) != ESP_OK) { handleWebDashboard(); continue; }
            handleWebDashboard();
            if (message.identifier != ECU_RESPONSE_ID || message.data_length_code < 5) continue;
            if (message.data[0] != 0x21) continue;
            printCANFrame(message);
            payload[6] = message.data[1]; payload[7] = message.data[2]; payload[8] = message.data[3];
            lastDtcStatus = payload[8];
            if ((payload[8] & 0x09) != 0 && activeDtcCount < 4)
                activeDtcs[activeDtcCount++] = "P0118";
            return true;
        }
        return false;
    }
    return false;
}

// ============================================================
// PID DECODE
// ============================================================
float decodePidValue(const PidReading &r)
{
    if (!r.valid) return 0.0f;
    uint8_t A = (r.dlc >= 4) ? r.data[3] : 0;
    uint8_t B = (r.dlc >= 5) ? r.data[4] : 0;

    switch (r.pid)
    {
        case 0x04: return A * 100.0f / 255.0f;
        case 0x05: return (float)A - 40.0f;
        case 0x06: return ((float)A - 128.0f) * 100.0f / 128.0f;
        case 0x07: return ((float)A - 128.0f) * 100.0f / 128.0f;
        case 0x08: return ((float)A - 128.0f) * 100.0f / 128.0f;
        case 0x09: return ((float)A - 128.0f) * 100.0f / 128.0f;
        case 0x0A: return A * 3.0f;
        case 0x0B: return (float)A;
        case 0x0C: return (((float)A * 256.0f) + B) / 4.0f;
        case 0x0D: return (float)A; // Speed
        case 0x0E: return ((float)A / 2.0f) - 64.0f;
        case 0x0F: return (float)A - 40.0f;
        case 0x10: return (((float)A * 256.0f) + B) / 100.0f;
        case 0x11: return A * 100.0f / 255.0f;
        case 0x1C: return (float)A;
        case 0x1F: return ((float)A * 256.0f) + B;
        case 0x21: return ((float)A * 256.0f) + B;
        case 0x22: return (((float)A * 256.0f) + B) * 0.079f;
        case 0x23: return (((float)A * 256.0f) + B) * 10.0f;
        case 0x2C: return A * 100.0f / 255.0f;
        case 0x2D: return ((float)A - 128.0f) * 100.0f / 128.0f;
        case 0x2E: return A * 100.0f / 255.0f;
        case 0x2F: return A * 100.0f / 255.0f;
        case 0x30: return (float)A;
        case 0x31: return ((float)A * 256.0f) + B;
        case 0x32: { int16_t v = (int16_t)((A << 8) | B); return v / 4.0f; }
        case 0x33: return (float)A;
        case 0x3C: return (((float)A * 256.0f) + B) / 10.0f - 40.0f;
        case 0x42: return (((float)A * 256.0f) + B) / 1000.0f;
        case 0x43: return (((float)A * 256.0f) + B) * 100.0f / 255.0f;
        case 0x45: return A * 100.0f / 255.0f;
        case 0x46: return (float)A - 40.0f;
        case 0x47: return A * 100.0f / 255.0f;
        case 0x48: return A * 100.0f / 255.0f;
        case 0x49: return A * 100.0f / 255.0f;
        case 0x4A: return A * 100.0f / 255.0f;
        case 0x4B: return A * 100.0f / 255.0f;
        case 0x4C: return A * 100.0f / 255.0f;
        case 0x4D: return ((float)A * 256.0f) + B;
        case 0x4E: return ((float)A * 256.0f) + B;
        case 0x51: return (float)A;
        case 0x5C: return (float)A - 40.0f;
        case 0x5D: return ((((float)A * 256.0f) + B) / 128.0f) - 210.0f;
        case 0x5E: return (((float)A * 256.0f) + B) * 0.05f;
    }
    return 0.0f;
}

String buildLiveJSON()
{
    JsonDocument doc;
    for (uint8_t i = 0; i < PID_COUNT; i++)
    {
        char key[3];
        snprintf(key, sizeof(key), "%02X", pidReadings[i].pid);
        if (pidReadings[i].valid)
            doc[key] = decodePidValue(pidReadings[i]);
        else
            doc[key] = nullptr;
    }
    String out;
    serializeJson(doc, out);
    return out;
}

// ============================================================
// DISTANCE TRACKER (Calculates 100m from OBD Speed)
// ============================================================
void updateDistanceTrackers()
{
    unsigned long now = millis();
    if (lastSpeedTime == 0) lastSpeedTime = now;
    
    float dt = (now - lastSpeedTime) / 1000.0f; // in seconds
    lastSpeedTime = now;

    // OBD Distance is integrated from Speed (PID 0D)
    int idxSpeed = -1;
    for (int i = 0; i < PID_COUNT; i++) {
        if (pids[i] == 0x0D) { idxSpeed = i; break; }
    }

    if (idxSpeed >= 0 && pidReadings[idxSpeed].valid)
    {
        float speedKmh = decodePidValue(pidReadings[idxSpeed]);
        if (speedKmh > 0.5f) {
            // Convert km/h to m/s, multiply by time
            obdDistMeters += (speedKmh / 3.6f) * dt;
        }
    }
}

// ============================================================
// UPDATE DASHBOARD
// ============================================================
void updateDashboard()
{
    updateDistanceTrackers();
    
    String liveJSON = buildLiveJSON();
    lastDiagnosticEvent = "PID polling complete.";

    if (activeDtcCount > 0)
    {
        lastDiagnosticEvent += "\nDTC: ";
        for (uint8_t i = 0; i < activeDtcCount; i++)
        {
            if (i > 0) lastDiagnosticEvent += ", ";
            lastDiagnosticEvent += activeDtcs[i];
        }
    }

    updateDashboardData(
        liveJSON,
        latestReadingsJSON,
        getCanFeedString(),
        getActionFeedString(),
        lastDiagnosticEvent,
        obdDistMeters,
        udsDistMeters
    );
}

// ============================================================
// PID NAME / RESULT HELPERS
// ============================================================
String getPidName(uint8_t pid)
{
    switch (pid)
    {
        case 0x04: return "Engine load";
        case 0x05: return "Coolant";
        case 0x06: return "STFT B1";
        case 0x07: return "LTFT B1";
        case 0x08: return "STFT B2";
        case 0x09: return "LTFT B2";
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
        case 0x21: return "Dist w/ MIL";
        case 0x22: return "Fuel rail";
        case 0x23: return "Fuel rail DI";
        case 0x2C: return "Commanded EGR";
        case 0x2D: return "EGR error";
        case 0x2E: return "Evap purge";
        case 0x2F: return "Fuel level";
        case 0x30: return "Warmups";
        case 0x31: return "Dist since clear";
        case 0x32: return "Evap vapor";
        case 0x33: return "Barometric";
        case 0x3C: return "Cat temp B1S1";
        case 0x42: return "Module voltage";
        case 0x43: return "Abs load";
        case 0x45: return "Rel throttle";
        case 0x46: return "Ambient temp";
        case 0x47: return "Throttle B";
        case 0x48: return "Throttle C";
        case 0x49: return "Pedal D";
        case 0x4A: return "Pedal E";
        case 0x4B: return "Pedal F";
        case 0x4C: return "Cmd throttle";
        case 0x4D: return "MIL time";
        case 0x4E: return "Time since clr";
        case 0x51: return "Fuel type";
        case 0x5C: return "Oil temp";
        case 0x5D: return "Inj timing";
        case 0x5E: return "Fuel rate";
    }
    return "Unknown PID";
}

String getPidResultText(uint8_t pid, const PidReading &reading)
{
    if (!reading.valid) return "No valid ECU response.";
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
        if (pids[i] == pid) { index = i; break; }
    if (index < 0) return false;

    sendOBDRequest(pid);
    bool success = receiveECUResponse(pid, pidReadings[index]);

    if (!success)
    {
        updateDashboardRequestResult(getPidName(pid), "OBD-II request failed or timed out.");
        return false;
    }

    latestReadingsJSON = buildReadingsJSON();
    updateDashboardJSON(latestReadingsJSON);
    pushReadingsJSON(latestReadingsJSON);
    updateDashboard();

    updateDashboardRequestResult(getPidName(pid), getPidResultText(pid, pidReadings[index]));
    return true;
}

bool performDashboardVinRequest()
{
    if (!sendUdsReadVinRequest())
    {
        updateDashboardRequestResult("READ VIN", "UDS 0x22 VIN request transmission failed.");
        return false;
    }
    if (!receiveUdsVinResponse())
    {
        updateDashboardRequestResult("READ VIN", "UDS 0x22 VIN response failed or timed out.");
        return false;
    }
    updateDashboardRequestResult("READ VIN", "VIN: " + lastVin + "\nUDS: 0x22\nDID: F190");
    return true;
}

bool performDashboardDtcRequest()
{
    activeDtcCount = 0;
    lastDtcStatus = 0;

    if (!sendUdsReadDtcRequest() || !receiveUdsDtcResponse())
    {
        updateDashboardRequestResult("READ DTC", "UDS 0x19 DTC request failed or timed out.");
        return false;
    }

    String result = "DTC response received.\n";
    if (activeDtcCount > 0)
    {
        result += "DTC: ";
        for (uint8_t i = 0; i < activeDtcCount; i++)
        {
            if (i > 0) result += ", ";
            result += activeDtcs[i];
        }
    }
    else result += "No reported DTC.";

    char statusText[8];
    snprintf(statusText, sizeof(statusText), "%02X", lastDtcStatus);
    result += "\nStatus: 0x" + String(statusText);

    updateDashboard();
    updateDashboardRequestResult("READ DTC", result);
    return true;
}

bool performDashboardSessionRequest()
{
    twai_message_t m = {};
    m.identifier = OBD_REQUEST_ID; m.data_length_code = 8;
    m.data[0] = 0x02; m.data[1] = 0x10; m.data[2] = 0x01;
    
    if (twai_transmit(&m, pdMS_TO_TICKS(100)) != ESP_OK) return false;
    pushCanFeed("TX", m);
    
    updateDashboardRequestResult("DEFAULT SESSION", "UDS 0x10 Request sent.");
    return true;
}

// ============================================================
// STAGE II : SGW AVAILABILITY TEST
// ============================================================
void performSgwTest()
{
    String log = "SGW AVAILABILITY TEST\n=======================\n";

    twai_message_t m = {};
    m.identifier = 0x7E0; m.data_length_code = 8;
    m.data[0] = 0x03; m.data[1] = 0x22; m.data[2] = 0xF1; m.data[3] = 0x90;

    if (twai_transmit(&m, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        updateDashboardRequestResult("SGW TEST", "TX failed.");
        return;
    }
    pushCanFeed("TX", m);
    log += "TX -> 7E0: 03 22 F1 90 (Read VIN, DID F190)\n\n";

    uint8_t vinPayload[32] = {};
    uint16_t totalLen = 0, receivedLen = 0;
    uint8_t expectedSN = 1;
    bool gotPositive = false, gotNegative = false, sawFF = false;
    uint8_t nrc = 0;
    unsigned long start = millis();

    while (millis() - start < 2000)
    {
        handleWebDashboard();
        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK) continue;
        handleWebDashboard();

        if (rx.identifier != 0x7E8) { printCANFrame(rx); continue; }
        printCANFrame(rx);
        if (rx.data_length_code < 2) continue;

        uint8_t frameType = (rx.data[0] >> 4) & 0x0F;
        uint8_t sfLen = rx.data[0] & 0x0F;

        if (frameType == 0x00)
        {
            if (sfLen >= 3 && rx.data[1] == 0x7F && rx.data[2] == 0x22)
            {
                nrc = rx.data[3];
                gotNegative = true;
                char buf[80];
                snprintf(buf, sizeof(buf), "RX <- 7E8: 7F 22 %02X (NEGATIVE)\n", nrc);
                log += buf;
                break;
            }
            else if (sfLen >= 3 && rx.data[1] == 0x62)
            {
                log += "RX <- 7E8: positive single-frame\n";
                gotPositive = true;
                break;
            }
        }
        else if (frameType == 0x01)
        {
            sawFF = true;
            totalLen = ((rx.data[0] & 0x0F) << 8) | rx.data[1];
            char buf[80];
            snprintf(buf, sizeof(buf), "RX <- 7E8: FIRST FRAME, total %d bytes\n", totalLen);
            log += buf;
            for (int i = 0; i < 6 && receivedLen < totalLen && receivedLen < 32; i++)
                vinPayload[receivedLen++] = rx.data[2 + i];

            twai_message_t fc = {};
            fc.identifier = 0x7E0; fc.data_length_code = 8; fc.data[0] = 0x30;
            twai_transmit(&fc, pdMS_TO_TICKS(100));
            pushCanFeed("TX", fc);
            log += "TX -> 7E0: 30 00 00 (Flow Control)\n";
            start = millis();
            continue;
        }
        else if (frameType == 0x02)
        {
            uint8_t sn = rx.data[0] & 0x0F;
            if (sn != expectedSN) continue;
            for (int i = 1; i < rx.data_length_code && receivedLen < totalLen && receivedLen < 32; i++)
                vinPayload[receivedLen++] = rx.data[i];
            expectedSN = (expectedSN + 1) & 0x0F;
            if (receivedLen >= totalLen) { gotPositive = true; break; }
            start = millis();
        }
    }

    log += "\nINTERPRETATION:\n";
    if (gotNegative)
    {
        char buf[80]; snprintf(buf, sizeof(buf), "NRC = 0x%02X\n", nrc); log += buf;
        switch (nrc)
        {
            case 0x11: log += "Service Not Supported\n"; break;
            case 0x12: log += "Sub-function Not Supported\n"; break;
            case 0x13: log += "Incorrect Message Length\n"; break;
            case 0x22: log += "Conditions Not Correct\n"; break;
            case 0x31: log += "Request Out Of Range\n"; break;
            case 0x33: log += ">>> SGW / SFD IS BLOCKING <<<\n>>> Need 0x27 unlock first.\n"; break;
            case 0x35: log += "Invalid Key\n"; break;
            case 0x78: log += "Response Pending (retry in a moment)\n"; break;
            case 0x7E: log += "Sub-function Not Supported in Active Session\n"; break;
            case 0x7F: log += "Service Not Supported in Active Session\n"; break;
            default: log += "Unknown NRC\n"; break;
        }
    }
    else if (gotPositive && receivedLen >= 20)
    {
        if (vinPayload[0] == 0x62 && vinPayload[1] == 0xF1 && vinPayload[2] == 0x90)
        {
            String vin = "";
            for (int i = 3; i < 20; i++) vin += (char)vinPayload[i];
            log += "VIN = " + vin + "\n>>> SGW is PASSING diagnostic traffic <<<\n";
            lastVin = vin;
        }
        else log += "Unexpected positive payload.\n";
    }
    else if (sawFF) log += "Partial multi-frame response (incomplete).\n>>> SGW may be passing but timing/interruption issue.\n";
    else
    {
        log += "No response from 0x7E8.\n";
        log += ">>> Possible SGW block, no ECU at 0x01,\n";
        log += ">>> wrong wiring/bitrate, or SGW silently dropping.\n";
    }

    updateDashboardRequestResult("SGW TEST", log);
}

// ============================================================
// STAGE II : MANUAL OBD PID
// ============================================================
String decodePid00Bitmap(uint8_t A, uint8_t B, uint8_t C, uint8_t D)
{
    String result = "";
    uint8_t bytes[4] = {A, B, C, D};
    int lineLen = 0;
    for (uint8_t pid = 0x01; pid <= 0x20; pid++)
    {
        uint8_t byteIdx = (pid - 1) / 8;
        uint8_t bitIdx = 7 - ((pid - 1) % 8);
        if ((bytes[byteIdx] >> bitIdx) & 1)
        {
            char buf[8]; snprintf(buf, sizeof(buf), "%02X ", pid); result += buf;
            if (++lineLen % 8 == 0) result += "\n";
        }
    }
    if (lineLen == 0) result = "(none reported)";
    return result;
}

void performManualPidRequest(uint8_t pid)
{
    sendOBDRequest(pid);
    PidReading temp;
    bool success = receiveECUResponse(pid, temp);

    char nameBuf[16]; snprintf(nameBuf, sizeof(nameBuf), "PID %02X", pid);

    if (!success) { updateDashboardRequestResult(nameBuf, "No response / timeout."); return; }

    char reqBuf[40]; snprintf(reqBuf, sizeof(reqBuf), "02 01 %02X 00 00 00 00 00", pid);
    char respBuf[48] = ""; int n = 0;
    for (int i = 0; i < temp.dlc && n < (int)sizeof(respBuf) - 4; i++)
        n += snprintf(respBuf + n, sizeof(respBuf) - n, "%02X ", temp.data[i]);

    String result = "Request: " + String(reqBuf) + "\nResponse: " + String(respBuf) + "\n\n";
    uint8_t payloadLen = temp.data[0] & 0x0F;
    
    if (payloadLen >= 3 && temp.data[1] == 0x41 && temp.data[2] == pid)
    {
        if (pid == 0x00 && payloadLen >= 6)
        {
            result += "Supported PIDs (01-20):\n" + decodePid00Bitmap(temp.data[3], temp.data[4], temp.data[5], temp.data[6]);
        }
        else if (pid == 0x21 && payloadLen >= 4)
        {
            uint16_t km = (temp.data[3] << 8) | temp.data[4];
            result += "Distance with MIL = " + String(km) + " km";
        }
        else
        {
            float v = decodePidValue(temp);
            result += getPidName(pid) + " = " + String(v, 2);
        }
    }
    else result += "Unexpected response format.";

    updateDashboardRequestResult(nameBuf, result);
}

// ============================================================
// STAGE II : ODOMETER SCAN (Red UDS)
// ============================================================
void performOdometerRead()
{
    struct Target { uint32_t tx; uint32_t rx; const char* name; };
    Target targets[] = {
        {0x7E0, 0x7E8, "ECU(0x01)"},
        {0x7E1, 0x7E9, "TCU(0x02)"},
        {0x714, 0x77E, "IC(0x17)"},
        {0x7A4, 0x7CE, "IC-alt(0x17)"},
        {0x7E2, 0x7EA, "ABS(0x03)"},
    };

    struct Did { uint16_t did; float div; const char* mode; };
    Did dids[] = {
        {0x2205, 100.0f,  "wheel-pulse"},
        {0x2203, 1000.0f, "total-dist"},
        {0x40C0, 1000.0f, "NVRAM-gearbox"},
        {0x0101, 100.0f,  "mileage-status"},
    };

    String log = "ODOMETER SCAN\n=============\n";
    int successCount = 0;

    for (auto &t : targets)
    {
        bool targetOk = false;
        for (auto &d : dids)
        {
            twai_message_t m = {};
            m.identifier = t.tx; m.data_length_code = 8;
            m.data[0] = 0x03; m.data[1] = 0x22;
            m.data[2] = (d.did >> 8) & 0xFF; m.data[3] = d.did & 0xFF;

            if (twai_transmit(&m, pdMS_TO_TICKS(50)) != ESP_OK) continue;
            pushCanFeed("TX", m);

            unsigned long start = millis();
            bool got = false;

            while (millis() - start < 200)
            {
                handleWebDashboard();
                twai_message_t rx;
                if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) continue;
                handleWebDashboard();

                if (rx.identifier != t.rx) { printCANFrame(rx); continue; }
                printCANFrame(rx);
                if (rx.data_length_code < 4) continue;

                if (rx.data[1] == 0x7F && rx.data[2] == 0x22)
                {
                    char buf[128]; snprintf(buf, sizeof(buf), "  %s DID %04X: NEG NRC 0x%02X\n", t.name, d.did, rx.data[3]);
                    log += buf;
                    break;
                }
                if (rx.data[1] == 0x62 && rx.data[2] == ((d.did >> 8) & 0xFF) && rx.data[3] == (d.did & 0xFF))
                {
                    uint8_t payloadLen = rx.data[0] & 0x0F;
                    if (payloadLen >= 7)
                    {
                        uint32_t raw = ((uint32_t)rx.data[4] << 24) | ((uint32_t)rx.data[5] << 16) | ((uint32_t)rx.data[6] << 8) | rx.data[7];
                        float km = raw / d.div;
                        
                        // UPDATE THE RED UDS ODOMETER
                        udsDistMeters = km * 1000.0f; 
                        
                        char buf[160]; snprintf(buf, sizeof(buf), "  %s DID %04X [%s]: %.2f km\n", t.name, d.did, d.mode, km);
                        log += buf;
                        got = true; targetOk = true; successCount++;
                        break;
                    }
                }
            }
            if (!got)
            {
                char buf[128]; snprintf(buf, sizeof(buf), "  %s DID %04X: no response\n", t.name, d.did);
                log += buf;
            }
            if (targetOk) break;
        }
    }

    if (successCount == 0) log += "\nNo positive response from any target.\n";
    else log += "\nDone. " + String(successCount) + " positive response(s).\n";
    log += "Red UDS Odometer updated!";

    updateDashboard();
    updateDashboardRequestResult("ODOMETER", log);
}

void processDashboardRequest()
{
    String request = getPendingDashboardRequest();
    if (request.length() == 0) return;

    if      (request == "rpm")        performDashboardPidRequest(0x0C);
    else if (request == "speed")      performDashboardPidRequest(0x0D);
    else if (request == "coolant")    performDashboardPidRequest(0x05);
    else if (request == "throttle")   performDashboardPidRequest(0x11);
    else if (request == "vin")        performDashboardVinRequest();
    else if (request == "dtc")        performDashboardDtcRequest();
    else if (request == "session")    performDashboardSessionRequest();
    else if (request == "sgw")        performSgwTest();
    else if (request == "pid0100")    performManualPidRequest(0x00);
    else if (request == "pid0121")    performManualPidRequest(0x21);
    else if (request == "odometer")   performOdometerRead();
    else updateDashboardRequestResult(request, "Unknown dashboard request.");
}

void waitWithDashboard(unsigned long duration)
{
    unsigned long startTime = millis();
    while (millis() - startTime < duration)
    {
        handleWebDashboard();
        delay(10);
    }
}
// Create SIM800L object
SIM800L sim800l;
// create GPS object
GPS gps;
void setup()
{
    Serial.begin(115200);
    delay(1000);

   
    
    Serial.println("================================");
    Serial.println("      Golf 8 OBD-II SNIFFER");
    Serial.println("================================");
      // Initialize SIM800L
     //sim800l.begin();
     gps.begin();
     Serial.println("GPS BEGIN COMPLETED");
 
    // Wait for GSM network
    //if (sim800l.waitForNetwork(60000)) {

        // Call your phone number for 15 seconds
      //  sim800l.makeCall("+213782773051", 15000);

    //} else {

      //  Serial.println("SIM NETWORK UNAVAILABLE!");
    //}///
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return;
    if (twai_start() != ESP_OK) return;
    
    Serial.println("TWAI started.");

    startWebDashboard();
    startTransmitServer();

    lastLivePollTime = millis() - LIVE_POLL_INTERVAL;
    lastSpeedTime = millis();
    Serial.println("READY.");
}

void loop()
{
    
    handleWebDashboard();
    handleTransmitClient();
    processDashboardRequest();
    gps.update();
    if (millis() - lastLivePollTime >= LIVE_POLL_INTERVAL)
    {
        lastLivePollTime = millis();
        pollAllPIDs();
    }

    handleWebDashboard();
    handleTransmitClient();
    delay(2);
}