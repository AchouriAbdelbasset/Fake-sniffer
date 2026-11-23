
#include "gsm_call.h"

// ESP32 UART2
HardwareSerial simSerial(2);

// UART pins
#define SIM_RX 26
#define SIM_TX 27

void SIM800L::begin() {

    Serial.println("\n--- SIM800L INITIALIZATION ---");

    // Start communication with SIM800L
    simSerial.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);

    delay(1500);

    // Check if SIM800L responds
    bool connected = false;

    for (int i = 0; i < 5; i++) {

        String response = sendAT("AT", 1500);

        if (response.indexOf("OK") >= 0) {
            connected = true;
            break;
        }

        delay(1000);
    }

    if (!connected) {
        Serial.println("ERROR: SIM800L not responding!");
        return;
    }

    Serial.println("SIM800L communication OK");

    // Disable command echo
    sendAT("ATE0", 2000);

    Serial.println("SIM800L initialized");
}


String SIM800L::sendAT(const String& command,
                       unsigned long timeout) {

    // Clear previous responses
    while (simSerial.available()) {
        simSerial.read();
    }

    Serial.print("AT COMMAND: ");
    Serial.println(command);

    simSerial.println(command);

    String response = "";
    unsigned long start = millis();

    while (millis() - start < timeout) {

        while (simSerial.available()) {

            char c = simSerial.read();
            response += c;
        }

        delay(10);
    }

    Serial.print("RESPONSE: ");
    Serial.println(response);

    return response;
}


bool SIM800L::waitForNetwork(unsigned long timeout) {

    Serial.println("\nChecking GSM network...");

    unsigned long start = millis();

    while (millis() - start < timeout) {

        // Check whether the SIM card is ready
        String pinResponse = sendAT("AT+CPIN?", 2000);

        if (pinResponse.indexOf("READY") < 0) {
            Serial.println("SIM card not ready...");
            delay(2000);
            continue;
        }

        // Check network registration
        String response = sendAT("AT+CREG?", 2000);

        if (response.indexOf("+CREG: 0,1") >= 0 ||
            response.indexOf("+CREG: 0,5") >= 0) {

            Serial.println("GSM NETWORK REGISTERED!");
            return true;
        }

        Serial.println("Waiting for network...");
        delay(3000);
    }

    Serial.println("ERROR: Network registration timeout!");

    return false;
}


bool SIM800L::makeCall(const char* phoneNumber,
                       unsigned long duration) {

    Serial.println("\n--- MAKING PHONE CALL ---");

    String command = "ATD";
    command += phoneNumber;
    command += ";";

    String response = sendAT(command, 10000);

    if (response.indexOf("OK") < 0) {

        Serial.println("ERROR: Call command failed!");

        return false;
    }

    Serial.println("CALL STARTED!");

    // Keep call active for requested duration
    delay(duration);

    Serial.println("Hanging up...");

    sendAT("ATH", 5000);

    Serial.println("CALL FINISHED");

    return true;
}
