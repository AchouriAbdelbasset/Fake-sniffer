#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include <Arduino.h>

// ============================================================
// START / HANDLE WEB DASHBOARD
// ============================================================

void startWebDashboard();
void handleWebDashboard();

// ============================================================
// DASHBOARD DATA
// ============================================================
//
// liveJSON       - decoded PID values as JSON object,
//                  e.g. {"04":15.2,"05":110,"0C":3000}
//
// readingsJSON   - the app-facing readings JSON (the deliverable)
//
// prediction     - predictive DTC text (unchanged content)
//
// canFeed        - raw CAN frame feed (multi-line string)
//
// eventData      - short diagnostic event line

void updateDashboardData(
    const String &liveJSON,
    const String &readingsJSON,
    const String &prediction,
    const String &canFeed,
    const String &eventData
);

// Update just the readings JSON (used during streaming)
void updateDashboardJSON(const String &readingsJSON);

// ============================================================
// DASHBOARD REQUESTS
// ============================================================

String getPendingDashboardRequest();

void updateDashboardRequestResult(
    const String &requestName,
    const String &result
);

#endif