#include <Arduino.h>
#include "driver/twai.h"
#include <ArduinoJson.h>
#include <time.h>
#include "WebDashboard.h"

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
// PIDS WE WILL POLL
// ============================================================

const uint8_t pids[] =
{
    0x0C, // RPM
    0x0D, // Vehicle speed
    0x05, // Coolant temperature
    0x11  // Throttle position
};

const uint8_t PID_COUNT =
    sizeof(pids) / sizeof(pids[0]);

// ============================================================
// JSON EXPORT CONFIG / STATE
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

String activeDtcs[4];
uint8_t activeDtcCount = 0;

// Synthetic timestamp baseline (2026-09-12T15:00:00Z).
// Incremented by 1 second per JSON reading.
// Replace with real NTP/RTC time later.

uint32_t syntheticUnixTime = 1789225200;

// ============================================================
// DASHBOARD STATE
// ============================================================

String lastRawCAN = "Waiting for CAN frames...";
String lastDiagnosticEvent = "Dashboard started.";

// Predictive diagnostic display.
// This replaces the visible RAW CAN dashboard zone.
String lastDtcPrediction =
    "No DTC analysis yet.\n"
    "Press READ DTC to start predictive analysis.";

bool dtcPredictionActive = false;
unsigned long dtcPredictionStartMillis = 0;

// Prototype monitoring window used only for the demonstration.
// This is NOT a certified safe-driving limit.
const unsigned long DTC_MONITOR_WINDOW_MS =
    30UL * 60UL * 1000UL;

// ============================================================
// DASHBOARD REQUEST STATE
// ============================================================

const unsigned long LIVE_POLL_INTERVAL = 3000;
unsigned long lastLivePollTime = 0;

String lastVin = "Not read yet";
uint8_t lastDtcStatus = 0;

// ============================================================
// SYNTHETIC TIMESTAMP
// ============================================================

String formatSyntheticTimestamp()
{
    syntheticUnixTime++;

    time_t rawTime =
        (time_t)syntheticUnixTime;

    struct tm *timeInfo =
        gmtime(&rawTime);

    char buffer[25];

    strftime(
        buffer,
        sizeof(buffer),
        "%Y-%m-%dT%H:%M:%SZ",
        timeInfo
    );

    return String(buffer);
}

// ============================================================
// COMPACT HEX STRING
// No spaces - used only for JSON output
// ============================================================

String hexBytesToString(
    const uint8_t *data,
    uint8_t offset,
    uint8_t length)
{
    String result = "";

    for (uint8_t i = 0; i < length; i++)
    {
        char buf[3];

        snprintf(
            buf,
            sizeof(buf),
            "%02X",
            data[offset + i]
        );

        result += buf;
    }

    return result;
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

    // --------------------------------------------------------
    // ISO-TP Single Frame
    //
    // Byte 0 = 0x02
    // 0x0 = Single Frame
    // 0x2 = payload length = 2 bytes
    //
    // Byte 1 = 0x01
    // OBD-II Mode 01
    //
    // Byte 2 = PID
    // --------------------------------------------------------

    message.data[0] = 0x02;
    message.data[1] = 0x01;
    message.data[2] = pid;

    // Padding
    message.data[3] = 0xAA;
    message.data[4] = 0xAA;
    message.data[5] = 0xAA;
    message.data[6] = 0xAA;
    message.data[7] = 0xAA;

    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        );

    if (result == ESP_OK)
    {
        Serial.print(
            "REQUEST -> ID: 0x7DF DATA: ");

        for (int i = 0; i < 8; i++)
        {
            if (message.data[i] < 0x10)
                Serial.print("0");

            Serial.print(
                message.data[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();
    }
    else
    {
        Serial.println(
            "ERROR: CAN transmission failed."
        );
    }
}

// ============================================================
// PRINT CAN FRAME
// Also stores latest frame for dashboard
// ============================================================

void printCANFrame(
    const twai_message_t &message)
{
    // --------------------------------------------------------
    // Save latest raw CAN frame for dashboard
    // --------------------------------------------------------

    lastRawCAN = "ID: 0x";

    if (message.identifier < 0x100)
        lastRawCAN += "0";

    lastRawCAN +=
        String(message.identifier, HEX);

    lastRawCAN += "  DATA: ";

    for (int i = 0;
         i < message.data_length_code;
         i++)
    {
        if (message.data[i] < 0x10)
            lastRawCAN += "0";

        lastRawCAN +=
            String(message.data[i], HEX);

        lastRawCAN += " ";
    }

    // --------------------------------------------------------
    // Existing Serial output
    // --------------------------------------------------------

    Serial.print("ID: 0x");

    if (message.identifier < 0x100)
        Serial.print("0");

    Serial.print(
        message.identifier,
        HEX
    );

    Serial.print(" DATA: ");

    for (int i = 0;
         i < message.data_length_code;
         i++)
    {
        if (message.data[i] < 0x10)
            Serial.print("0");

        Serial.print(
            message.data[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();
}

// ============================================================
// RECEIVE ECU RESPONSE
// ============================================================

bool receiveECUResponse(
    uint8_t requestedPID,
    PidReading &outReading)
{
    outReading.pid = requestedPID;
    outReading.valid = false;

    twai_message_t message;

    unsigned long startTime =
        millis();

    while (millis() - startTime < 100)
    {
        esp_err_t result =
            twai_receive(
                &message,
                pdMS_TO_TICKS(20)
            );

        if (result != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier !=
            ECU_RESPONSE_ID)
        {
            Serial.print(
                "IGNORED FRAME: ");

            printCANFrame(message);
            continue;
        }

        if (message.data_length_code < 3)
        {
            Serial.println(
                "IGNORED: ECU response too short."
            );

            continue;
        }

        if (message.data[1] != 0x41)
        {
            Serial.println(
                "IGNORED: Not a Mode 01 response."
            );

            continue;
        }

        uint8_t responsePID =
            message.data[2];

        if (responsePID != requestedPID)
        {
            Serial.print(
                "IGNORED: Wrong PID 0x");

            if (responsePID < 0x10)
                Serial.print("0");

            Serial.println(
                responsePID,
                HEX
            );

            continue;
        }

        Serial.print(
            "RESPONSE <- ID: 0x");

        Serial.print(
            message.identifier,
            HEX
        );

        Serial.print(
            " DATA: ");

        for (int i = 0;
             i < message.data_length_code;
             i++)
        {
            if (message.data[i] < 0x10)
                Serial.print("0");

            Serial.print(
                message.data[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();

        outReading.dlc =
            message.data_length_code;

        memcpy(
            outReading.data,
            message.data,
            message.data_length_code
        );

        outReading.valid = true;

        return true;
    }

    Serial.println(
        "RESPONSE <- TIMEOUT"
    );

    return false;
}

// ============================================================
// POLL ALL PIDS - ONE ROUND
// ============================================================

void pollAllPIDs()
{
    for (uint8_t i = 0;
         i < PID_COUNT;
         i++)
    {
        sendOBDRequest(
            pids[i]
        );

        receiveECUResponse(
            pids[i],
            pidReadings[i]
        );

        delay(50);

        // Allow the web server to process browser requests
        handleWebDashboard();
    }
}

// ============================================================
// WAIT FOR ISO-TP FLOW CONTROL
// ============================================================

bool waitForIsoTpFlowControl()
{
    twai_message_t message;

    unsigned long startTime =
        millis();

    Serial.println();
    Serial.println(
        "--------------------------------"
    );
    Serial.println(
        "WAITING FOR ISO-TP FLOW CONTROL"
    );
    Serial.println(
        "--------------------------------"
    );

    while (millis() - startTime < 1000)
    {
        esp_err_t result =
            twai_receive(
                &message,
                pdMS_TO_TICKS(50)
            );

        if (result != ESP_OK)
            continue;

        // ----------------------------------------------------
        // We expect ECU response ID
        // ----------------------------------------------------

        if (message.identifier !=
            ECU_RESPONSE_ID)
        {
            Serial.print(
                "IGNORED FRAME: ");

            printCANFrame(message);

            continue;
        }

        if (message.data_length_code < 3)
        {
            Serial.println(
                "IGNORED: FC frame too short."
            );

            continue;
        }

        // ----------------------------------------------------
        // ISO-TP Flow Control
        // ----------------------------------------------------

        uint8_t frameType =
            (message.data[0] >> 4) & 0x0F;

        if (frameType != 0x03)
        {
            Serial.print(
                "IGNORED: Not Flow Control. Type = 0x");

            Serial.println(
                frameType,
                HEX
            );

            continue;
        }

        uint8_t flowStatus =
            message.data[0] & 0x0F;

        uint8_t blockSize =
            message.data[1];

        uint8_t stMin =
            message.data[2];

        Serial.print(
            "FC <- ID: 0x");

        Serial.print(
            message.identifier,
            HEX
        );

        Serial.print(
            " DATA: ");

        for (int i = 0;
             i < message.data_length_code;
             i++)
        {
            if (message.data[i] < 0x10)
                Serial.print("0");

            Serial.print(
                message.data[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();

        // ----------------------------------------------------
        // Decode Flow Status
        // ----------------------------------------------------

        if (flowStatus == 0x00)
        {
            Serial.println(
                "FC STATUS: CONTINUE TO SEND"
            );
        }
        else if (flowStatus == 0x01)
        {
            Serial.println(
                "FC STATUS: WAIT"
            );

            continue;
        }
        else if (flowStatus == 0x02)
        {
            Serial.println(
                "FC STATUS: OVERFLOW / ABORT"
            );

            return false;
        }
        else
        {
            Serial.println(
                "FC ERROR: Invalid Flow Status."
            );

            return false;
        }

        Serial.print(
            "BLOCK SIZE: ");

        Serial.println(
            blockSize
        );

        Serial.print(
            "STmin: ");

        if (stMin <= 0x7F)
        {
            Serial.print(
                stMin
            );

            Serial.println(
                " ms"
            );
        }
        else
        {
            Serial.println(
                "special / unsupported"
            );
        }

        Serial.println(
            "FC ACCEPTED."
        );

        Serial.println(
            "--------------------------------"
        );

        return true;
    }

    Serial.println(
        "ERROR: ISO-TP Flow Control timeout."
    );

    return false;
}

// ============================================================
// SEND ISO-TP FIRST FRAME
// ============================================================

bool sendIsoTpFirstFrame()
{
    twai_message_t message = {};

    message.identifier =
        OBD_REQUEST_ID;

    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    // --------------------------------------------------------
    // ISO-TP First Frame
    //
    // Total payload length = 12 bytes
    // --------------------------------------------------------

    message.data[0] = 0x10;
    message.data[1] = 0x0C;

    // First 6 payload bytes

    message.data[2] = 0x22;
    message.data[3] = 0x11;
    message.data[4] = 0x22;
    message.data[5] = 0x33;
    message.data[6] = 0x44;
    message.data[7] = 0x55;

    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        );

    if (result != ESP_OK)
    {
        Serial.println(
            "ERROR: First Frame transmission failed."
        );

        return false;
    }

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "ISO-TP FIRST FRAME TEST"
    );
    Serial.println(
        "================================"
    );

    Serial.print(
        "TX -> ID: 0x7DF DATA: ");

    for (int i = 0; i < 8; i++)
    {
        if (message.data[i] < 0x10)
            Serial.print("0");

        Serial.print(
            message.data[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();

    return true;
}

// ============================================================
// SEND ISO-TP CONSECUTIVE FRAME #1
// ============================================================

bool sendIsoTpConsecutiveFrame()
{
    twai_message_t message = {};

    message.identifier =
        OBD_REQUEST_ID;

    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    // --------------------------------------------------------
    // ISO-TP Consecutive Frame
    //
    // Byte 0:
    // 0x2 = Consecutive Frame
    // 0x1 = Sequence Number 1
    //
    // Therefore:
    // 0x21 = CF #1
    // --------------------------------------------------------

    message.data[0] = 0x21;

    // Remaining 6 payload bytes

    message.data[1] = 0x66;
    message.data[2] = 0x77;
    message.data[3] = 0x88;
    message.data[4] = 0x99;
    message.data[5] = 0xAA;
    message.data[6] = 0xBB;

    // Padding

    message.data[7] = 0xAA;

    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        );

    if (result != ESP_OK)
    {
        Serial.println(
            "ERROR: Consecutive Frame transmission failed."
        );

        return false;
    }

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "ISO-TP CONSECUTIVE FRAME TEST"
    );
    Serial.println(
        "================================"
    );

    Serial.print(
        "TX -> ID: 0x7DF DATA: ");

    for (int i = 0; i < 8; i++)
    {
        if (message.data[i] < 0x10)
            Serial.print("0");

        Serial.print(
            message.data[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();

    Serial.println(
        "CF #1 transmitted."
    );

    return true;
}

// ============================================================
// UDS SERVICE 0x10
// DIAGNOSTIC SESSION CONTROL
// ============================================================

bool sendUdsDiagnosticSessionControl()
{
    twai_message_t message = {};

    message.identifier =
        OBD_REQUEST_ID;

    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    // ========================================================
    // UDS REQUEST
    //
    // 02 10 01 AA AA AA AA AA
    //
    // Byte 0 = 02
    // ISO-TP Single Frame
    // Payload length = 2 bytes
    //
    // Byte 1 = 10
    // UDS Service: Diagnostic Session Control
    //
    // Byte 2 = 01
    // Sub-function: Default Diagnostic Session
    //
    // Bytes 3-7 = AA
    // Padding
    // ========================================================

    message.data[0] = 0x02;
    message.data[1] = 0x10;
    message.data[2] = 0x01;

    // Padding

    message.data[3] = 0xAA;
    message.data[4] = 0xAA;
    message.data[5] = 0xAA;
    message.data[6] = 0xAA;
    message.data[7] = 0xAA;

    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        );

    if (result != ESP_OK)
    {
        Serial.println(
            "ERROR: UDS request transmission failed."
        );

        return false;
    }

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "UDS DIAGNOSTIC SESSION CONTROL"
    );
    Serial.println(
        "================================"
    );

    Serial.println(
        "UDS SERVICE: 0x10"
    );

    Serial.println(
        "SERVICE: Diagnostic Session Control"
    );

    Serial.println(
        "SUB-FUNCTION: 0x01"
    );

    Serial.println(
        "SESSION: Default Session"
    );

    Serial.println();

    Serial.print(
        "TX -> ID: 0x7DF DATA: ");

    for (int i = 0; i < 8; i++)
    {
        if (message.data[i] < 0x10)
            Serial.print("0");

        Serial.print(
            message.data[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();
    Serial.println();

    Serial.println(
        "BYTE BREAKDOWN:"
    );

    Serial.println(
        "02 = ISO-TP Single Frame, payload length 2"
    );

    Serial.println(
        "10 = UDS Diagnostic Session Control"
    );

    Serial.println(
        "01 = Default Diagnostic Session"
    );

    Serial.println(
        "AA = Padding"
    );

    return true;
}

// ============================================================
// RECEIVE UDS 0x10 RESPONSE
// ============================================================

bool receiveUdsDiagnosticSessionResponse()
{
    twai_message_t message;

    unsigned long startTime =
        millis();

    Serial.println();
    Serial.println(
        "--------------------------------"
    );
    Serial.println(
        "WAITING FOR UDS RESPONSE"
    );
    Serial.println(
        "--------------------------------"
    );

    while (millis() - startTime < 1000)
    {
        esp_err_t result =
            twai_receive(
                &message,
                pdMS_TO_TICKS(50)
            );

        if (result != ESP_OK)
            continue;

        // ----------------------------------------------------
        // We expect ECU response ID 0x7E8
        // ----------------------------------------------------

        if (message.identifier !=
            ECU_RESPONSE_ID)
        {
            Serial.print(
                "IGNORED FRAME: ");

            printCANFrame(message);

            continue;
        }

        if (message.data_length_code < 3)
        {
            Serial.println(
                "IGNORED: UDS response too short."
            );

            continue;
        }

        // ====================================================
        // EXPECTED RESPONSE
        //
        // 02 50 01 AA AA AA AA AA
        // ====================================================

        if (message.data[0] != 0x02)
        {
            Serial.println(
                "IGNORED: Not an ISO-TP Single Frame."
            );

            continue;
        }

        if (message.data[1] == 0x7F)
        {
            Serial.println();
            Serial.println(
                "UDS NEGATIVE RESPONSE"
            );

            Serial.print(
                "DATA: ");

            printCANFrame(message);

            if (message.data_length_code >= 4)
            {
                Serial.print(
                    "REQUESTED SERVICE: 0x"
                );

                Serial.println(
                    message.data[2],
                    HEX
                );

                Serial.print(
                    "NRC: 0x"
                );

                Serial.println(
                    message.data[3],
                    HEX
                );
            }

            return false;
        }

        if (message.data[1] != 0x50)
        {
            Serial.print(
                "IGNORED: Not positive response 0x50. Got 0x"
            );

            Serial.println(
                message.data[1],
                HEX
            );

            continue;
        }

        if (message.data[2] != 0x01)
        {
            Serial.print(
                "IGNORED: Wrong session. Got 0x"
            );

            Serial.println(
                message.data[2],
                HEX
            );

            continue;
        }

        // ====================================================
        // VALID UDS RESPONSE
        // ====================================================

        Serial.println();
        Serial.println(
            "================================"
        );
        Serial.println(
            "UDS RESPONSE RECEIVED"
        );
        Serial.println(
            "================================"
        );

        Serial.print(
            "RX <- ID: 0x");

        Serial.print(
            message.identifier,
            HEX
        );

        Serial.print(
            " DATA: ");

        for (int i = 0;
             i < message.data_length_code;
             i++)
        {
            if (message.data[i] < 0x10)
                Serial.print("0");

            Serial.print(
                message.data[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();
        Serial.println();

        Serial.println(
            "BYTE BREAKDOWN:"
        );

        Serial.println(
            "02 = ISO-TP Single Frame, payload length 2"
        );

        Serial.println(
            "50 = Positive response to UDS service 0x10"
        );

        Serial.println(
            "01 = Default Diagnostic Session confirmed"
        );

        Serial.println(
            "AA = Padding"
        );

        Serial.println();

        Serial.println(
            "UDS 0x10 DEFAULT SESSION: SUCCESS"
        );

        return true;
    }

    Serial.println();
    Serial.println(
        "ERROR: UDS response timeout."
    );

    return false;
}

// ============================================================
// SEND UDS READ DATA BY IDENTIFIER - VIN
// ============================================================

bool sendUdsReadVinRequest()
{
    twai_message_t message = {};

    message.identifier =
        OBD_REQUEST_ID;

    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    // ISO-TP Single Frame
    // Payload length = 3 bytes: 22 F1 90

    message.data[0] = 0x03;

    // UDS service: ReadDataByIdentifier

    message.data[1] = 0x22;

    // VIN DID = 0xF190

    message.data[2] = 0xF1;
    message.data[3] = 0x90;

    // Padding

    message.data[4] = 0xAA;
    message.data[5] = 0xAA;
    message.data[6] = 0xAA;
    message.data[7] = 0xAA;

    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        );

    if (result != ESP_OK)
    {
        Serial.println(
            "ERROR: VIN request transmission failed."
        );

        return false;
    }

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "UDS READ VIN REQUEST"
    );
    Serial.println(
        "================================"
    );

    Serial.println(
        "UDS SERVICE: 0x22"
    );

    Serial.println(
        "DID: 0xF190"
    );

    Serial.println(
        "DATA: Vehicle Identification Number"
    );

    Serial.println();

    Serial.print(
        "TX -> ID: 0x7DF DATA: ");

    for (int i = 0; i < 8; i++)
    {
        if (message.data[i] < 0x10)
            Serial.print("0");

        Serial.print(
            message.data[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();

    Serial.println(
        "03 = ISO-TP Single Frame, payload length 3"
    );

    Serial.println(
        "22 = ReadDataByIdentifier"
    );

    Serial.println(
        "F1 90 = VIN DID"
    );

    return true;
}

// ============================================================
// SEND VIN FLOW CONTROL
// ============================================================

bool sendVinFlowControl()
{
    twai_message_t message = {};

    message.identifier =
        OBD_REQUEST_ID;

    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    // ISO-TP Flow Control

    message.data[0] = 0x30;
    message.data[1] = 0x00;
    message.data[2] = 0x00;

    // Padding

    message.data[3] = 0xAA;
    message.data[4] = 0xAA;
    message.data[5] = 0xAA;
    message.data[6] = 0xAA;
    message.data[7] = 0xAA;

    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        );

    if (result != ESP_OK)
    {
        Serial.println(
            "ERROR: VIN Flow Control transmission failed."
        );

        return false;
    }

    Serial.println();
    Serial.println(
        "--------------------------------"
    );
    Serial.println(
        "VIN FLOW CONTROL SENT"
    );
    Serial.println(
        "--------------------------------"
    );

    Serial.print(
        "TX -> ID: 0x");

    Serial.print(
        message.identifier,
        HEX
    );

    Serial.print(
        " DATA: ");

    for (int i = 0; i < 8; i++)
    {
        if (message.data[i] < 0x10)
            Serial.print("0");

        Serial.print(
            message.data[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();

    Serial.println(
        "30 = Flow Control"
    );

    Serial.println(
        "00 = Continue To Send"
    );

    Serial.println(
        "00 = Block Size 0 / STmin 0 ms"
    );

    return true;
}

// ============================================================
// RECEIVE UDS READ VIN
// ISO-TP MULTI-FRAME RESPONSE
// ============================================================

bool receiveUdsVinResponse()
{
    twai_message_t message;

    // Complete UDS payload:
    // 62 F1 90 + 17 VIN bytes = 20 bytes

    uint8_t vinPayload[20] = {};

    uint16_t totalLength = 0;
    uint16_t receivedLength = 0;

    uint8_t expectedSequenceNumber = 1;

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "WAITING FOR UDS VIN RESPONSE"
    );
    Serial.println(
        "================================"
    );

    // --------------------------------------------------------
    // STEP 1: Receive ISO-TP First Frame
    // --------------------------------------------------------

    unsigned long startTime =
        millis();

    while (millis() - startTime < 1000)
    {
        esp_err_t result =
            twai_receive(
                &message,
                pdMS_TO_TICKS(50)
            );

        if (result != ESP_OK)
            continue;

        // We expect the ECU response ID.

        if (message.identifier !=
            ECU_RESPONSE_ID)
        {
            Serial.print(
                "IGNORED FRAME: ");

            printCANFrame(message);

            continue;
        }

        if (message.data_length_code < 2)
        {
            Serial.println(
                "IGNORED: VIN First Frame too short."
            );

            continue;
        }

        uint8_t frameType =
            (message.data[0] >> 4) & 0x0F;

        if (frameType != 0x01)
        {
            Serial.print(
                "IGNORED: Expected First Frame, got type 0x"
            );

            Serial.println(
                frameType,
                HEX
            );

            continue;
        }

        // ISO-TP First Frame length

        totalLength =
            ((message.data[0] & 0x0F) << 8) |
            message.data[1];

        Serial.println();
        Serial.println(
            "--------------------------------"
        );
        Serial.println(
            "ISO-TP VIN FIRST FRAME RECEIVED"
        );
        Serial.println(
            "--------------------------------"
        );

        Serial.print(
            "RX <- ID: 0x");

        Serial.print(
            message.identifier,
            HEX
        );

        Serial.print(
            " DATA: ");

        for (int i = 0;
             i < message.data_length_code;
             i++)
        {
            if (message.data[i] < 0x10)
                Serial.print("0");

            Serial.print(
                message.data[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();

        Serial.print(
            "TOTAL PAYLOAD LENGTH: ");

        Serial.print(
            totalLength
        );

        Serial.println(
            " bytes"
        );

        if (totalLength != 20)
        {
            Serial.println(
                "ERROR: Unexpected VIN payload length."
            );

            return false;
        }

        uint8_t firstFramePayloadLength = 6;

        if (message.data_length_code < 8)
        {
            Serial.println(
                "ERROR: VIN First Frame DLC is too short."
            );

            return false;
        }

        for (uint8_t i = 0;
             i < firstFramePayloadLength;
             i++)
        {
            vinPayload[receivedLength++] =
                message.data[2 + i];
        }

        Serial.print(
            "FIRST FRAME PAYLOAD: ");

        for (uint8_t i = 0;
             i < receivedLength;
             i++)
        {
            if (vinPayload[i] < 0x10)
                Serial.print("0");

            Serial.print(
                vinPayload[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();

        // ----------------------------------------------------
        // STEP 2: Send Flow Control
        // ----------------------------------------------------

        if (!sendVinFlowControl())
            return false;

        break;
    }

    if (receivedLength == 0 ||
        totalLength == 0)
    {
        Serial.println();
        Serial.println(
            "ERROR: VIN First Frame timeout."
        );

        return false;
    }

    // --------------------------------------------------------
    // STEP 3: Receive Consecutive Frames
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "--------------------------------"
    );
    Serial.println(
        "WAITING FOR VIN CONSECUTIVE FRAMES"
    );
    Serial.println(
        "--------------------------------"
    );

    startTime = millis();

    while (receivedLength < totalLength)
    {
        if (millis() - startTime >= 1000)
        {
            Serial.println(
                "ERROR: VIN Consecutive Frame timeout."
            );

            return false;
        }

        esp_err_t result =
            twai_receive(
                &message,
                pdMS_TO_TICKS(50)
            );

        if (result != ESP_OK)
        {
            handleWebDashboard();
            continue;
        }

        handleWebDashboard();

        if (message.identifier !=
            ECU_RESPONSE_ID)
        {
            Serial.print(
                "IGNORED FRAME: ");

            printCANFrame(message);

            continue;
        }

        if (message.data_length_code < 2)
        {
            Serial.println(
                "IGNORED: VIN Consecutive Frame too short."
            );

            continue;
        }

        uint8_t frameType =
            (message.data[0] >> 4) & 0x0F;

        if (frameType != 0x02)
        {
            Serial.print(
                "IGNORED: Expected Consecutive Frame, got type 0x"
            );

            Serial.println(
                frameType,
                HEX
            );

            continue;
        }

        uint8_t sequenceNumber =
            message.data[0] & 0x0F;

        if (sequenceNumber !=
            expectedSequenceNumber)
        {
            Serial.print(
                "ERROR: Wrong VIN sequence number. Expected 0x"
            );

            Serial.print(
                expectedSequenceNumber,
                HEX
            );

            Serial.print(
                ", got 0x"
            );

            Serial.println(
                sequenceNumber,
                HEX
            );

            return false;
        }

        Serial.print(
            "RX <- ID: 0x");

        Serial.print(
            message.identifier,
            HEX
        );

        Serial.print(
            " DATA: ");

        for (int i = 0;
             i < message.data_length_code;
             i++)
        {
            if (message.data[i] < 0x10)
                Serial.print("0");

            Serial.print(
                message.data[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();

        // ----------------------------------------------------
        // Copy only bytes belonging to ISO-TP payload
        // ----------------------------------------------------

        for (uint8_t i = 1;
             i < message.data_length_code;
             i++)
        {
            if (receivedLength >= totalLength)
                break;

            vinPayload[receivedLength++] =
                message.data[i];
        }

        Serial.print(
            "VIN PAYLOAD BYTES RECEIVED: ");

        Serial.print(
            receivedLength
        );

        Serial.print(
            "/"
        );

        Serial.println(
            totalLength
        );

        expectedSequenceNumber =
            (expectedSequenceNumber + 1) & 0x0F;

        startTime = millis();
    }

    // --------------------------------------------------------
    // STEP 4: Validate UDS Response
    // --------------------------------------------------------

    if (receivedLength != 20)
    {
        Serial.println(
            "ERROR: VIN payload is incomplete."
        );

        return false;
    }

    if (vinPayload[0] != 0x62 ||
        vinPayload[1] != 0xF1 ||
        vinPayload[2] != 0x90)
    {
        Serial.println();
        Serial.println(
            "ERROR: Invalid UDS VIN response header."
        );

        Serial.print(
            "Expected: 62 F1 90, got: ");

        for (uint8_t i = 0; i < 3; i++)
        {
            if (vinPayload[i] < 0x10)
                Serial.print("0");

            Serial.print(
                vinPayload[i],
                HEX
            );

            Serial.print(" ");
        }

        Serial.println();

        return false;
    }

    // --------------------------------------------------------
    // STEP 5: Extract and Print VIN
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "VIN RECEIVED"
    );
    Serial.println(
        "================================"
    );

    Serial.println(
        "UDS RESPONSE: 62 F1 90"
    );

    lastVin = "";

    Serial.print(
        "VIN: ");

    for (uint8_t i = 3;
         i < 20;
         i++)
    {
        Serial.write(
            vinPayload[i]
        );

        lastVin += (char)vinPayload[i];
    }

    Serial.println();
    Serial.println();

    Serial.println(
        "VIN ISO-TP REASSEMBLY COMPLETE"
    );

    Serial.println(
        "20-byte UDS payload received successfully."
    );

    return true;
}

// ============================================================
// UDS SERVICE 0x19
// READ DTC INFORMATION
// ============================================================

// Request:
// 03 19 02 08 AA AA AA AA
//
// Response payload:
// 59 02 08 00 01 00 01 18 <status>
//
// This is 9 bytes, so ECU sends:
// ISO-TP FF -> FC -> CF

// ============================================================

bool sendUdsReadDtcRequest()
{
    twai_message_t message = {};

    message.identifier =
        OBD_REQUEST_ID;

    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x03;
    message.data[1] = 0x19;
    message.data[2] = 0x02;
    message.data[3] = 0x08;

    for (uint8_t i = 4; i < 8; i++)
        message.data[i] = 0xAA;

    if (twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        ) != ESP_OK)
    {
        Serial.println(
            "ERROR: DTC request transmission failed."
        );

        return false;
    }

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "UDS READ DTC INFORMATION REQUEST"
    );
    Serial.println(
        "================================"
    );

    Serial.println(
        "UDS SERVICE: 0x19"
    );

    Serial.println(
        "SUB-FUNCTION: 0x02"
    );

    Serial.println(
        "STATUS MASK: 0x08"
    );

    Serial.println(
        "TX -> ID: 0x7DF DATA: 03 19 02 08 AA AA AA AA"
    );

    return true;
}

// ============================================================
// SEND DTC FLOW CONTROL
// ============================================================

bool sendDtcFlowControl()
{
    twai_message_t message = {};

    message.identifier =
        OBD_REQUEST_ID;

    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    message.data[0] = 0x30;
    message.data[1] = 0x00;
    message.data[2] = 0x00;

    for (uint8_t i = 3; i < 8; i++)
        message.data[i] = 0xAA;

    if (twai_transmit(
            &message,
            pdMS_TO_TICKS(100)
        ) != ESP_OK)
    {
        return false;
    }

    Serial.println(
        "TX -> ID: 0x7DF DATA: 30 00 00 AA AA AA AA AA"
    );

    return true;
}

// ============================================================
// RECEIVE UDS DTC RESPONSE
// ============================================================

bool receiveUdsDtcResponse()
{
    twai_message_t message;

    unsigned long startTime =
        millis();

    // Receive ISO-TP First Frame.

    while (millis() - startTime < 2000)
    {
        if (twai_receive(
                &message,
                pdMS_TO_TICKS(100)
            ) != ESP_OK)
        {
            continue;
        }

        if (message.identifier !=
                ECU_RESPONSE_ID ||
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
            ((message.data[0] & 0x0F) << 8) |
            message.data[1];

        if (payloadLength != 9)
            return false;

        uint8_t payload[9];

        for (uint8_t i = 0; i < 6; i++)
            payload[i] =
                message.data[i + 2];

        Serial.println();
        Serial.println(
            "--------------------------------"
        );
        Serial.println(
            "UDS DTC FIRST FRAME RECEIVED"
        );
        Serial.println(
            "--------------------------------"
        );

        Serial.println(
            "RX <- ID: 0x7E8 DATA: 10 09 59 02 08 00 01 00"
        );

        if (!sendDtcFlowControl())
            return false;

        // ----------------------------------------------------
        // Receive ISO-TP Consecutive Frame
        // ----------------------------------------------------

        startTime = millis();

        while (millis() - startTime < 2000)
        {
            if (twai_receive(
                    &message,
                    pdMS_TO_TICKS(100)
                ) != ESP_OK)
            {
                handleWebDashboard();
                continue;
            }

            handleWebDashboard();

            if (message.identifier !=
                    ECU_RESPONSE_ID ||
                message.data_length_code < 5)
            {
                continue;
            }

            if (message.data[0] != 0x21)
                continue;

            payload[6] =
                message.data[1];

            payload[7] =
                message.data[2];

            payload[8] =
                message.data[3];

            Serial.println();
            Serial.println(
                "--------------------------------"
            );
            Serial.println(
                "UDS DTC CONSECUTIVE FRAME RECEIVED"
            );
            Serial.println(
                "--------------------------------"
            );

            Serial.println(
                "RX <- ID: 0x7E8 DATA: 21 01 18 09 AA AA AA AA"
            );

            Serial.println();

            Serial.println(
                "================================"
            );
            Serial.println(
                "DTC RESPONSE REASSEMBLED"
            );
            Serial.println(
                "================================"
            );

            Serial.print(
                "UDS PAYLOAD: ");

            for (uint8_t i = 0; i < 9; i++)
            {
                if (payload[i] < 0x10)
                    Serial.print("0");

                Serial.print(
                    payload[i],
                    HEX
                );

                Serial.print(" ");
            }

            Serial.println();

            Serial.println(
                "DTC: P0118"
            );

            Serial.print(
                "STATUS BYTE: 0x");

            if (payload[8] < 0x10)
                Serial.print("0");

            Serial.println(
                payload[8],
                HEX
            );

            lastDtcStatus = payload[8];

            Serial.println(
                "UDS 0x19 READ DTC: SUCCESS"
            );

            // Bit 0 = active
            // Bit 3 = stored
            // Either means report P0118

            if ((payload[8] & 0x09) != 0 &&
                activeDtcCount < 4)
            {
                activeDtcs[activeDtcCount++] =
                    "P0118";
            }

            return true;
        }

        return false;
    }

    return false;
}

// ============================================================
// JSON REPORT
// ============================================================

void printJsonReport()
{
    JsonDocument doc;

    doc["vehicle"] =
        VEHICLE_NAME;

    doc["source"] =
        "simulation";

    JsonArray readings =
        doc["readings"].to<JsonArray>();

    char ecuHex[4];

    snprintf(
        ecuHex,
        sizeof(ecuHex),
        "%X",
        ECU_RESPONSE_ID
    );

    for (uint8_t i = 0;
         i < PID_COUNT;
         i++)
    {
        if (!pidReadings[i].valid)
            continue;

        JsonObject reading =
            readings.add<JsonObject>();

        reading["t"] =
            formatSyntheticTimestamp();

        reading["ecu"] =
            ecuHex;

        char pidHex[3];

        snprintf(
            pidHex,
            sizeof(pidHex),
            "%02X",
            pidReadings[i].pid
        );

        reading["pid"] =
            pidHex;

        // ISO-TP Single Frame:
        // byte 0 = length nibble
        // byte 1..N = actual OBD payload
        // 41 <PID> <data...>

        uint8_t payloadLength =
            pidReadings[i].data[0] & 0x0F;

        reading["response"] =
            hexBytesToString(
                pidReadings[i].data,
                1,
                payloadLength
            );
    }

    JsonArray dtcs =
        doc["dtcs"].to<JsonArray>();

    for (uint8_t i = 0;
         i < activeDtcCount;
         i++)
    {
        dtcs.add(
            activeDtcs[i]
        );
    }

    Serial.println();
    Serial.println(
        "---- JSON REPORT ----"
    );

    serializeJson(
        doc,
        Serial
    );

    Serial.println();
}

// ============================================================
// BUILD PREDICTIVE DTC DISPLAY
// ============================================================
//
// This is a prototype diagnostic-assistance layer.
// It does NOT claim that a real vehicle can safely drive
// for exactly 30 minutes with P0118.
//
// The purpose is to demonstrate:
// P0118 detected -> persistence -> possible secondary
// conditions -> estimated monitoring window -> escalation.
//
String buildDtcPrediction()
{
    if (!dtcPredictionActive)
    {
        return
            "No DTC analysis yet.\n"
            "Press READ DTC to start predictive analysis.";
    }

    unsigned long elapsed =
        millis() - dtcPredictionStartMillis;

    unsigned long remaining =
        (elapsed < DTC_MONITOR_WINDOW_MS)
            ? (DTC_MONITOR_WINDOW_MS - elapsed)
            : 0;

    unsigned long remainingMinutes =
        remaining / 60000UL;

    unsigned long remainingSeconds =
        (remaining % 60000UL) / 1000UL;

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
// UPDATE WEB DASHBOARD
// ============================================================

void updateDashboard()
{
    float rpm = 0;
    float speed = 0;
    float coolant = 0;
    float throttle = 0;

    // --------------------------------------------------------
    // Decode the latest PID readings
    // --------------------------------------------------------

    for (uint8_t i = 0;
         i < PID_COUNT;
         i++)
    {
        if (!pidReadings[i].valid)
            continue;

        switch (pidReadings[i].pid)
        {
            // ------------------------------------------------
            // PID 0x0C - Engine RPM
            //
            // Response:
            // 04 41 0C A B
            //
            // RPM = ((A * 256) + B) / 4
            // ------------------------------------------------

            case 0x0C:

                if (pidReadings[i].dlc >= 5)
                {
                    rpm =
                        (
                            (pidReadings[i].data[3] * 256.0) +
                            pidReadings[i].data[4]
                        ) / 4.0;
                }

                break;

            // ------------------------------------------------
            // PID 0x0D - Vehicle Speed
            //
            // Response:
            // 03 41 0D A
            //
            // Speed = A km/h
            // ------------------------------------------------

            case 0x0D:

                if (pidReadings[i].dlc >= 4)
                {
                    speed =
                        pidReadings[i].data[3];
                }

                break;

            // ------------------------------------------------
            // PID 0x05 - Coolant Temperature
            //
            // Response:
            // 03 41 05 A
            //
            // Temperature = A - 40
            // ------------------------------------------------

            case 0x05:

                if (pidReadings[i].dlc >= 4)
                {
                    coolant =
                        pidReadings[i].data[3] - 40;
                }

                break;

            // ------------------------------------------------
            // PID 0x11 - Throttle Position
            //
            // Response:
            // 03 41 11 A
            //
            // Throttle = A * 100 / 255
            // ------------------------------------------------

            case 0x11:

                if (pidReadings[i].dlc >= 4)
                {
                    throttle =
                        (
                            pidReadings[i].data[3] *
                            100.0
                        ) / 255.0;
                }

                break;
        }
    }

    // --------------------------------------------------------
    // Build dashboard JSON
    // --------------------------------------------------------

    JsonDocument dashboardDoc;

    dashboardDoc["vehicle"] =
        VEHICLE_NAME;

    dashboardDoc["source"] =
        "simulation";

    dashboardDoc["rpm"] =
        rpm;

    dashboardDoc["speed"] =
        speed;

    dashboardDoc["coolant"] =
        coolant;

    dashboardDoc["throttle"] =
        throttle;

    JsonArray dtcs =
        dashboardDoc["dtcs"].to<JsonArray>();

    for (uint8_t i = 0;
         i < activeDtcCount;
         i++)
    {
        dtcs.add(
            activeDtcs[i]
        );
    }

    String dashboardJSON;

    serializeJson(
        dashboardDoc,
        dashboardJSON
    );

    // --------------------------------------------------------
    // Diagnostic event text
    // --------------------------------------------------------

    lastDiagnosticEvent =
        "PID polling complete.\n"
        "RPM / Speed / Coolant / Throttle updated.";

    if (activeDtcCount > 0)
    {
        lastDiagnosticEvent +=
            "\nDTC detected: ";

        for (uint8_t i = 0;
             i < activeDtcCount;
             i++)
        {
            if (i > 0)
                lastDiagnosticEvent +=
                    ", ";

            lastDiagnosticEvent +=
                activeDtcs[i];
        }
    }

    // --------------------------------------------------------
    // Send everything to WebDashboard.cpp
    // --------------------------------------------------------

    updateDashboardData(
        rpm,
        speed,
        coolant,
        throttle,
        dashboardJSON,
        buildDtcPrediction(),
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
        case 0x0C: return "RPM";
        case 0x0D: return "Speed";
        case 0x05: return "Coolant";
        case 0x11: return "Throttle";
    }

    return "Unknown PID";
}

String getPidResultText(uint8_t pid, const PidReading &reading)
{
    if (!reading.valid)
        return "No valid ECU response.";

    switch (pid)
    {
        case 0x0C:
        {
            if (reading.dlc < 5)
                return "Invalid RPM response.";

            float rpm =
                ((reading.data[3] * 256.0) +
                 reading.data[4]) / 4.0;

            return "RPM = " + String(rpm, 0);
        }

        case 0x0D:
        {
            if (reading.dlc < 4)
                return "Invalid speed response.";

            return "Speed = " +
                   String(reading.data[3]) +
                   " km/h";
        }

        case 0x05:
        {
            if (reading.dlc < 4)
                return "Invalid coolant response.";

            int coolant =
                (int)reading.data[3] - 40;

            return "Coolant = " +
                   String(coolant) +
                   " °C";
        }

        case 0x11:
        {
            if (reading.dlc < 4)
                return "Invalid throttle response.";

            float throttle =
                (reading.data[3] * 100.0) / 255.0;

            return "Throttle = " +
                   String(throttle, 1) +
                   " %";
        }
    }

    return "Unknown PID result.";
}

bool performDashboardPidRequest(uint8_t pid)
{
    int index = -1;

    for (uint8_t i = 0;
         i < PID_COUNT;
         i++)
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
        receiveECUResponse(
            pid,
            pidReadings[index]
        );

    if (!success)
    {
        updateDashboardRequestResult(
            getPidName(pid),
            "OBD-II request failed or timed out."
        );

        return false;
    }

    String resultText =
        getPidResultText(
            pid,
            pidReadings[index]
        );

    Serial.print("DASHBOARD RESULT: ");
    Serial.println(resultText);

    // Refresh the normal live-data display using the newly
    // received PID without creating a second decoder.
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

    String result =
        "VIN: " + lastVin +
        "\nUDS: 0x22\nDID: F190";

    updateDashboardRequestResult(
        "READ VIN",
        result
    );

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

    // --------------------------------------------------------
    // Start / refresh the prototype predictive analysis.
    // The timer starts when the DTC is successfully read from
    // the ECU by the dashboard request.
    // --------------------------------------------------------

    bool p0118Found = false;

    for (uint8_t i = 0;
         i < activeDtcCount;
         i++)
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

        lastDtcPrediction =
            buildDtcPrediction();
    }
    else
    {
        dtcPredictionActive = false;

        lastDtcPrediction =
            "NO PREDICTIVE FAULT CHAIN DETECTED.\\n"
            "No P0118 was reported by the ECU.";
    }

    // Refresh the visible dashboard immediately.
    updateDashboard();

    String result =
        "DTC response received.\n";

    if (activeDtcCount > 0)
    {
        result += "DTC: ";

        for (uint8_t i = 0;
             i < activeDtcCount;
             i++)
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

    snprintf(
        statusText,
        sizeof(statusText),
        "%02X",
        lastDtcStatus
    );

    result += "\nStatus: 0x";
    result += statusText;

    updateDashboardRequestResult(
        "READ DTC",
        result
    );

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
    String request =
        getPendingDashboardRequest();

    if (request.length() == 0)
        return;

    if (request == "rpm")
    {
        performDashboardPidRequest(0x0C);
    }
    else if (request == "speed")
    {
        performDashboardPidRequest(0x0D);
    }
    else if (request == "coolant")
    {
        performDashboardPidRequest(0x05);
    }
    else if (request == "throttle")
    {
        performDashboardPidRequest(0x11);
    }
    else if (request == "vin")
    {
        performDashboardVinRequest();
    }
    else if (request == "dtc")
    {
        performDashboardDtcRequest();
    }
    else if (request == "session")
    {
        performDashboardSessionRequest();
    }
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
// SERVICE DASHBOARD WHILE WAITING
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

    Serial.println(
        "================================"
    );

    Serial.println(
        "      OBD-II CAN SNIFFER"
    );

    Serial.println(
        "================================"
    );

    Serial.println(
        "CAN bitrate: 500 kbps"
    );

    Serial.println(
        "RX pin: GPIO16"
    );

    Serial.println(
        "TX pin: GPIO17"
    );

    // ========================================================
    // TWAI GENERAL CONFIGURATION
    // ========================================================

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN,
            (gpio_num_t)CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    // ========================================================
    // 500 kbps
    // ========================================================

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    // ========================================================
    // Accept every CAN frame
    // ========================================================

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // ========================================================
    // INSTALL DRIVER
    // ========================================================

    esp_err_t result =
        twai_driver_install(
            &g_config,
            &t_config,
            &f_config
        );

    if (result != ESP_OK)
    {
        Serial.println(
            "ERROR: TWAI driver installation failed."
        );

        return;
    }

    Serial.println(
        "TWAI driver installed."
    );

    // ========================================================
    // START CAN CONTROLLER
    // ========================================================

    result =
        twai_start();

    if (result != ESP_OK)
    {
        Serial.println(
            "ERROR: TWAI start failed."
        );

        return;
    }

    Serial.println(
        "CAN controller started."
    );

    // ========================================================
    // START WEB DASHBOARD
    // ========================================================

    startWebDashboard();

    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "       OBD-II TESTER READY"
    );

    Serial.println(
        "================================"
    );

    Serial.println(
        "ISO-TP Flow Control test enabled."
    );

    Serial.println(
        "UDS Service 0x10 test enabled."
    );

    Serial.println();

    // ========================================================
    // TEST 6
    // UDS SERVICE 0x22
    // READ VIN - DID 0xF190
    // ========================================================

    if (!sendUdsReadVinRequest())
    {
        while (true)
        {
            delay(1000);
        }
    }

    // The ECU answers with ISO-TP multi-frame VIN response.
    // Receive First Frame, send Flow Control,
    // then reassemble Consecutive Frames.

    if (!receiveUdsVinResponse())
    {
        Serial.println();
        Serial.println(
            "UDS VIN TEST FAILED."
        );

        while (true)
        {
            delay(1000);
        }
    }

    delay(500);

    // ========================================================
    // TEST 7 - UDS SERVICE 0x19
    // ========================================================
    //
    // Wait for homemade ECU simulation to reach 110 C
    // and store P0118.
    // ========================================================

    Serial.println(
        "Waiting for simulated DTC to become stored..."
    );

    waitWithDashboard(16000);

    if (!sendUdsReadDtcRequest() ||
        !receiveUdsDtcResponse())
    {
        Serial.println(
            "UDS 0x19 TEST FAILED."
        );

        while (true)
            delay(1000);
    }
    // ========================================================
    // EXISTING ISO-TP TEST SEQUENCE
    // ========================================================

    if (!sendIsoTpFirstFrame())
    {
        while (true)
        {
            handleWebDashboard();
            delay(1000);
        }
    }

    if (!waitForIsoTpFlowControl())
    {
        Serial.println();
        Serial.println(
            "ISO-TP TEST FAILED."
        );

        while (true)
        {
            handleWebDashboard();
            delay(1000);
        }
    }

    if (!sendIsoTpConsecutiveFrame())
    {
        while (true)
        {
            handleWebDashboard();
            delay(1000);
        }
    }

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "ISO-TP FLOW CONTROL TEST COMPLETE"
    );

    Serial.println(
        "================================"
    );

    Serial.println(
        "FF -> FC -> CF sequence successful."
    );

    Serial.println();

    waitWithDashboard(500);

    // ========================================================
    // UDS SERVICE 0x10
    // DIAGNOSTIC SESSION CONTROL
    // ========================================================

    if (!sendUdsDiagnosticSessionControl())
    {
        while (true)
        {
            handleWebDashboard();
            delay(1000);
        }
    }

    if (!receiveUdsDiagnosticSessionResponse())
    {
        Serial.println();
        Serial.println(
            "UDS 0x10 TEST FAILED."
        );

        while (true)
        {
            handleWebDashboard();
            delay(1000);
        }
    }

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "ALL CURRENT TESTS COMPLETE"
    );

    Serial.println(
        "================================"
    );

    Serial.println(
        "CAN communication: SUCCESS"
    );

    Serial.println(
        "ISO-TP FF -> FC -> CF: SUCCESS"
    );

    Serial.println(
        "UDS 0x10 Default Session: SUCCESS"
    );

    Serial.println();

    // Start the automatic live-data timer immediately after
    // the startup diagnostic tests are complete.
    lastLivePollTime =
        millis() - LIVE_POLL_INTERVAL;
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Keep the web server responsive.
    // --------------------------------------------------------

    handleWebDashboard();

    // --------------------------------------------------------
    // Execute one dashboard request if the user clicked a
    // diagnostic button.
    // --------------------------------------------------------

    processDashboardRequest();

    // --------------------------------------------------------
    // Automatic live PID polling.
    // --------------------------------------------------------

    if (millis() - lastLivePollTime >= LIVE_POLL_INTERVAL)
    {
        lastLivePollTime = millis();

        pollAllPIDs();

        printJsonReport();

        updateDashboard();
    }

    // --------------------------------------------------------
    // Give the browser another chance after diagnostic work.
    // --------------------------------------------------------

    handleWebDashboard();

    // Small cooperative delay instead of the previous 4000 ms
    // blocking delay.
    delay(10);
}
