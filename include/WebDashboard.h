#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include <Arduino.h>

// ============================================================
// START / HANDLE WEB DASHBOARD
// ============================================================

// Start the ESP32 Wi-Fi dashboard
void startWebDashboard();

// Process incoming browser requests
void handleWebDashboard();

// ============================================================
// DASHBOARD DATA
// ============================================================

// Update the dashboard with current diagnostic data
void updateDashboardData(
    float rpm,
    float speed,
    float coolant,
    float throttle,
    const String &jsonData,
    const String &rawCanData,
    const String &eventData
);

// ============================================================
// DASHBOARD REQUESTS
// ============================================================

// Check whether the browser has requested a diagnostic action.
//
// Returns the command name, or an empty String if no request:
// "rpm", "speed", "coolant", "throttle", "vin", "dtc", "session"
String getPendingDashboardRequest();

// Send the result of the last dashboard-triggered request
// back to the web dashboard.
void updateDashboardRequestResult(
    const String &requestName,
    const String &result
);

#endif