
#ifndef gsm_call_H
#define gsm_call_H

#include <Arduino.h>

class SIM800L {
public:
    void begin();

    bool waitForNetwork(unsigned long timeout);

    bool makeCall(const char* phoneNumber, unsigned long duration);

private:
    String sendAT(const String& command, unsigned long timeout);
};

#endif
