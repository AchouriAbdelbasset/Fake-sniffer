#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include <Arduino.h>

void startWebDashboard();
void handleWebDashboard();

void updateDashboardData(
    const String &liveJSON,
    const String &readingsJSON,
    const String &canFeed,
    const String &actionFeed,
    const String &eventData,
    float obdDistMeters,
    float udsDistMeters
);

void updateDashboardJSON(const String &readingsJSON);

String getPendingDashboardRequest();

void updateDashboardRequestResult(
    const String &requestName,
    const String &result
);

#endif