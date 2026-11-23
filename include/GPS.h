
#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPS {
public:
    GPS();

    void begin();
    void update();

private:
    HardwareSerial gpsSerial;
    TinyGPSPlus gps;

    unsigned long lastStatusPrint;
};

#endif
