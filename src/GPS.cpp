
#include "GPS.h"

GPS::GPS() : gpsSerial(1) {
    lastStatusPrint = 0;
}

void GPS::begin() {
    gpsSerial.begin(9600, SERIAL_8N1, 32, 33);

    Serial.println("\n================================");
    Serial.println("   CAR-MUNICATION GPS STARTED");
    Serial.println("================================");
}

void GPS::update() {

    // Read and decode GPS messages
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // Print location when a new position is received
    if (gps.location.isUpdated()) {

        Serial.println("\n========== GPS LOCATION ==========");

        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);

        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);

        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());

        Serial.print("HDOP: ");
        Serial.println(gps.hdop.hdop());

        if (gps.altitude.isValid()) {
            Serial.print("Altitude: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" m");
        }

        if (gps.speed.isValid()) {
            Serial.print("Speed: ");
            Serial.print(gps.speed.kmph());
            Serial.println(" km/h");
        }

        Serial.println("==================================");
    }

    // Report when no position is available
    if (millis() - lastStatusPrint >= 3000) {
        lastStatusPrint = millis();

        if (!gps.location.isValid()) {
            Serial.println("GPS: Waiting for position...");
        }
    }
}
