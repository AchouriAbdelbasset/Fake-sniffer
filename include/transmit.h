#ifndef TRANSMIT_H
#define TRANSMIT_H

#include <Arduino.h>

// ============================================================
// TRANSMIT - APP-FACING DATA CHANNEL
// ============================================================
//
// This module is separate from the dashboard. It exists only
// to deliver the readings JSON to the application.
//
// It registers two HTTP routes on the shared WebServer:
//
//   GET /stream    - Server-Sent Events (SSE). Long-lived.
//                    Receives a new readings JSON every time
//                    the data changes.
//
//   GET /readings  - One-shot. Returns the latest readings
//                    JSON immediately. Used for initial sync
//                    or as a fallback.
//
// Both routes serve ONLY the readings JSON:
//
//   {
//     "vehicle": "...",
//     "readings": [
//       { "ecu": "7E8", "pid": "0C", "response": "410C2EE0" },
//       ...
//     ]
//   }
//
// Dashboard endpoints (/, /data, /request) are unaffected.
// ============================================================

// Register the transmit routes with the shared WebServer.
// Call once in setup(), after startWebDashboard().
void startTransmitServer();

// Flush any pending readings JSON to subscribed SSE clients.
// Call frequently from loop().
//
// Coalesces changes: if many changes happen within 10 ms,
// only the final one is sent. This keeps the socket calm
// while still feeling instant on the receiving side.
void handleTransmitClient();

// Notify the transmitter that the readings JSON has changed.
// Compares against the last-sent JSON; if identical, does
// nothing. Otherwise marks it as pending for the next flush.
void pushReadingsJSON(const String &json);

#endif