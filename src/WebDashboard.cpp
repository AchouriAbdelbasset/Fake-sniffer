#include "WebDashboard.h"

#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

// ============================================================
// DASHBOARD DATA
// ============================================================

float dashboardRPM = 0;
float dashboardSpeed = 0;
float dashboardCoolant = 0;
float dashboardThrottle = 0;

String dashboardJSON = "Waiting for data...";
String dashboardRawCAN =
    "No DTC analysis yet.\n"
    "Press READ DTC to start predictive analysis.";
String dashboardEvents = "Dashboard started.";

String dashboardRequestResult = "No dashboard request yet.";
String dashboardLastRequest = "None";

volatile int pendingDashboardRequest = 0;

// ============================================================
// DASHBOARD WEBPAGE
// ============================================================

const char dashboardHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>

<head>

    <meta charset="UTF-8">

    <meta name="viewport"
          content="width=device-width, initial-scale=1.0">

    <title>CAR-MUNICATION</title>

    <style>

        body {
            font-family: Arial, sans-serif;
            background: #111;
            color: white;
            margin: 0;
            padding: 20px;
        }

        h1 {
            text-align: center;
        }

        .status {
            text-align: center;
            margin-bottom: 20px;
            color: #aaa;
        }

        .dashboard {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }

        .zone {
            background: #1c1c1c;
            border: 1px solid #333;
            border-radius: 10px;
            padding: 15px;
            min-height: 200px;
        }

        .zone h2 {
            margin-top: 0;
            border-bottom: 1px solid #333;
            padding-bottom: 10px;
        }

        .data {
            font-size: 20px;
            line-height: 1.8;
        }

        pre {
            white-space: pre-wrap;
            word-wrap: break-word;
            font-size: 14px;
            max-height: 350px;
            overflow-y: auto;
        }

        /* ====================================================
           PREDICTIVE DIAGNOSTICS
           ==================================================== */

        #rawcan {
            min-height: 170px;
            overflow: visible;
        }

        .prediction-card {
            border: 1px solid #3a3a3a;
            border-left: 5px solid #777;
            border-radius: 9px;
            background: #151515;
            padding: 12px 14px;
            margin-bottom: 10px;
        }

        .prediction-card:last-child {
            margin-bottom: 0;
        }

        .prediction-title {
            font-size: 14px;
            font-weight: bold;
            letter-spacing: 0.5px;
            margin-bottom: 7px;
        }

        .prediction-main {
            font-size: 18px;
            font-weight: bold;
            margin-bottom: 4px;
        }

        .prediction-detail {
            color: #bbb;
            font-size: 13px;
            line-height: 1.5;
        }

        .prediction-status {
            display: inline-block;
            margin-top: 8px;
            padding: 4px 9px;
            border-radius: 999px;
            background: #242424;
            font-size: 12px;
            font-weight: bold;
        }

        .prediction-timer {
            font-size: 30px;
            font-weight: bold;
            letter-spacing: 2px;
            margin-top: 5px;
        }

        .prediction-list {
            margin: 6px 0 0 18px;
            padding: 0;
            color: #ccc;
            line-height: 1.7;
            font-size: 13px;
        }

        .prediction-footnote {
            color: #777;
            font-size: 11px;
            margin-top: 10px;
            text-align: center;
        }

        .status-red {
            border-left-color: #e05252;
        }

        .status-red .prediction-status {
            color: #ff7777;
        }

        .status-yellow {
            border-left-color: #d9b44a;
        }

        .status-yellow .prediction-status {
            color: #e6c65c;
        }

        .status-green {
            border-left-color: #52b788;
        }

        .status-green .prediction-status {
            color: #72d3a1;
        }

        .status-neutral {
            border-left-color: #777;
        }

        .status-critical {
            border-left-color: #e05252;
            background: #1b1313;
        }

        .critical-text {
            color: #ff7777;
            font-size: 19px;
            font-weight: bold;
            line-height: 1.5;
        }

        .phone {
            font-size: 16px;
            margin-top: 7px;
            font-weight: bold;
        }

        /* ====================================================
           REQUEST BUTTONS
           ==================================================== */

        .request-zone {
            grid-column: 1 / -1;
        }

        .request-group {
            margin-bottom: 18px;
        }

        .request-group h3 {
            margin-bottom: 10px;
        }

        .button-row {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
        }

        button {
            background: #2b2b2b;
            color: white;
            border: 1px solid #555;
            border-radius: 7px;
            padding: 12px 16px;
            font-size: 15px;
            cursor: pointer;
        }

        button:hover {
            background: #3a3a3a;
        }

        button:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }

        .request-result {
            background: #111;
            border: 1px solid #333;
            border-radius: 7px;
            padding: 12px;
            margin-top: 15px;
            white-space: pre-wrap;
            word-wrap: break-word;
            min-height: 60px;
        }

        .request-status {
            margin-top: 8px;
            color: #aaa;
            font-size: 14px;
        }

        @media (max-width: 700px) {

            .dashboard {
                grid-template-columns: 1fr;
            }

            .request-zone {
                grid-column: auto;
            }

        }

    </style>

</head>

<body>

    <h1>CAR-MUNICATION</h1>

    <div class="status">
        ESP32 Diagnostic Dashboard
        <br>
        <span id="connection">Connecting...</span>
    </div>


    <div class="dashboard">

        <!-- ========================================= -->
        <!-- ZONE 1 : LIVE VEHICLE DATA -->
        <!-- ========================================= -->

        <div class="zone">

            <h2>LIVE VEHICLE DATA</h2>

            <div class="data">

                RPM:
                <span id="rpm">--</span>

                <br>

                Speed:
                <span id="speed">--</span>
                km/h

                <br>

                Coolant:
                <span id="coolant">--</span>
                °C

                <br>

                Throttle:
                <span id="throttle">--</span>
                %

            </div>

        </div>


        <!-- ========================================= -->
        <!-- ZONE 2 : JSON -->
        <!-- ========================================= -->

        <div class="zone">

            <h2>JSON OUTPUT</h2>

            <pre id="json">Waiting for data...</pre>

        </div>


        <!-- ========================================= -->
        <!-- ZONE 3 : PREDICTIVE DIAGNOSTICS -->
        <!-- ========================================= -->

        <div class="zone">

            <h2>PREDICTIVE DIAGNOSTICS</h2>

            <div id="rawcan">

                <div class="prediction-card status-neutral">
                    <div class="prediction-title">DIAGNOSTIC STATUS</div>
                    <div class="prediction-main">No DTC analysis yet</div>
                    <div class="prediction-detail">
                        Press READ DTC to start predictive analysis.
                    </div>
                </div>

            </div>

        </div>


        <!-- ========================================= -->
        <!-- ZONE 4 : EVENTS -->
        <!-- ========================================= -->

        <div class="zone">

            <h2>DIAGNOSTIC EVENTS</h2>

            <pre id="events">Dashboard started.</pre>

        </div>


        <!-- ========================================= -->
        <!-- ZONE 5 : SEND REQUEST -->
        <!-- ========================================= -->

        <div class="zone request-zone">

            <h2>SEND DIAGNOSTIC REQUEST</h2>

            <div class="request-group">

                <h3>OBD-II</h3>

                <div class="button-row">

                    <button onclick="sendRequest('rpm')">
                        READ RPM
                    </button>

                    <button onclick="sendRequest('speed')">
                        READ SPEED
                    </button>

                    <button onclick="sendRequest('coolant')">
                        READ COOLANT
                    </button>

                    <button onclick="sendRequest('throttle')">
                        READ THROTTLE
                    </button>

                </div>

            </div>


            <div class="request-group">

                <h3>UDS</h3>

                <div class="button-row">

                    <button onclick="sendRequest('vin')">
                        READ VIN
                    </button>

                    <button onclick="sendRequest('dtc')">
                        READ DTC
                    </button>

                    <button onclick="sendRequest('session')">
                        DEFAULT SESSION
                    </button>

                </div>

            </div>


            <div class="request-status"
                 id="requestStatus">
                Ready.
            </div>

            <div class="request-result"
                 id="requestResult">
                No dashboard request yet.
            </div>

        </div>

    </div>


    <!-- ============================================= -->
    <!-- DASHBOARD SCRIPT -->
    <!-- ============================================= -->

    <script>

        let requestBusy = false;


        // ====================================================
        // RENDER PREDICTIVE DIAGNOSTICS
        // Converts the diagnostic text from the ESP32 into
        // human-readable status cards.
        // ====================================================

        function renderPredictiveDiagnostics(text)
        {
            const container =
                document.getElementById("rawcan");

            if (!text || text.trim() === "")
                return;

            // ------------------------------------------------
            // No analysis yet
            // ------------------------------------------------

            if (text.includes("No DTC analysis yet"))
            {
                container.innerHTML = `
                    <div class="prediction-card status-neutral">
                        <div class="prediction-title">
                            DIAGNOSTIC STATUS
                        </div>
                        <div class="prediction-main">
                            No DTC analysis yet
                        </div>
                        <div class="prediction-detail">
                            Press READ DTC to start predictive analysis.
                        </div>
                    </div>`;

                return;
            }

            // ------------------------------------------------
            // Determine current DTC state
            // ------------------------------------------------

            const active =
                text.includes("Status: ACTIVE");

            const stored =
                text.includes("Status: STORED");

            const expired =
                text.includes("Monitoring window exceeded");

            // ------------------------------------------------
            // Extract remaining time
            // Example: Approx. 29 min 27 remaining
            // ------------------------------------------------

            let timer = "--:--";

            const timerMatch = text.match(
                /Approx\.\s*(\d+)\s*min\s*(\d+)\s*remaining/i
            );

            if (timerMatch)
            {
                const minutes =
                    timerMatch[1].padStart(2, "0");

                const seconds =
                    timerMatch[2].padStart(2, "0");

                timer = minutes + ":" + seconds;
            }

            // ------------------------------------------------
            // Current fault card
            // ------------------------------------------------

            let statusLabel = "REPORTED";

            if (active)
                statusLabel = "ACTIVE";
            else if (stored)
                statusLabel = "STORED";

            const faultCard = `
                <div class="prediction-card status-red">
                    <div class="prediction-title">
                        🔴 CURRENT FAULT
                    </div>
                    <div class="prediction-main">
                        P0118
                    </div>
                    <div class="prediction-detail">
                        Engine Coolant Temperature Sensor<br>
                        Signal High
                    </div>
                    <div class="prediction-status">
                        ${statusLabel}
                    </div>
                </div>`;

            // ------------------------------------------------
            // Drive status
            // ------------------------------------------------

            let driveCard = "";

            if (expired)
            {
                driveCard = `
                    <div class="prediction-card status-red">
                        <div class="prediction-title">
                            🔴 DRIVE STATUS
                        </div>
                        <div class="prediction-main">
                            Monitoring window exceeded
                        </div>
                        <div class="prediction-detail">
                            The prototype escalation condition has been reached.
                        </div>
                    </div>`;
            }
            else
            {
                driveCard = `
                    <div class="prediction-card status-green">
                        <div class="prediction-title">
                            🟢 DRIVE STATUS
                        </div>
                        <div class="prediction-main">
                            Temporary operation
                        </div>
                        <div class="prediction-detail">
                            Vehicle may remain operational temporarily.<br>
                            Coolant-temperature signal is unreliable.
                        </div>
                        <div class="prediction-status">
                            MONITOR CONDITION
                        </div>
                    </div>`;
            }

            // ------------------------------------------------
            // Monitoring / prediction card
            // ------------------------------------------------

            let monitoringCard = "";

            if (expired)
            {
                monitoringCard = `
                    <div class="prediction-card status-red">
                        <div class="prediction-title">
                            🔴 CRITICAL ESCALATION
                        </div>
                        <div class="critical-text">
                            STOP VEHICLE SAFELY<br>
                            TURN OFF ENGINE
                        </div>
                        <div class="phone">
                            📞 +213 782 773 051
                        </div>
                    </div>`;
            }
            else
            {
                monitoringCard = `
                    <div class="prediction-card status-yellow">
                        <div class="prediction-title">
                            🟡 MONITORING WINDOW
                        </div>
                        <div class="prediction-timer">
                            ${timer}
                        </div>
                        <div class="prediction-detail">
                            Prototype monitoring estimate
                        </div>
                    </div>`;
            }

            // ------------------------------------------------
            // Predictive secondary conditions
            // ------------------------------------------------

            const predictionCard = `
                <div class="prediction-card status-yellow">
                    <div class="prediction-title">
                        🟡 PREDICTIVE WARNING
                    </div>
                    <div class="prediction-detail">
                        Persistent P0118 may lead to additional
                        temperature-management or correlation faults.
                    </div>
                    <ul class="prediction-list">
                        <li>P011B — ECT / IAT correlation</li>
                        <li>P0128 — Coolant temperature regulation</li>
                    </ul>
                </div>`;

            container.innerHTML =
                faultCard +
                driveCard +
                predictionCard +
                monitoringCard +
                `<div class="prediction-footnote">
                    Prototype estimation — not a certified safe-driving limit.
                </div>`;
        }


        // ====================================================
        // UPDATE DASHBOARD DATA
        // ====================================================

        async function updateDashboard()
        {
            try
            {
                const response = await fetch(
                    "/data",
                    {
                        cache: "no-store"
                    }
                );

                if (!response.ok)
                {
                    throw new Error("HTTP error");
                }

                const data =
                    await response.json();


                // -----------------------------
                // LIVE VEHICLE DATA
                // -----------------------------

                document.getElementById("rpm").textContent =
                    data.rpm;

                document.getElementById("speed").textContent =
                    data.speed;

                document.getElementById("coolant").textContent =
                    data.coolant;

                document.getElementById("throttle").textContent =
                    data.throttle;


                // -----------------------------
                // JSON OUTPUT
                // -----------------------------

                document.getElementById("json").textContent =
                    data.json;


                // -----------------------------
                // PREDICTIVE DIAGNOSTICS
                // -----------------------------

                renderPredictiveDiagnostics(
                    data.rawcan
                );


                // -----------------------------
                // EVENTS
                // -----------------------------

                document.getElementById("events").textContent =
                    data.events;


                // -----------------------------
                // REQUEST RESULT
                // -----------------------------

                if (data.request !== undefined)
                {
                    document.getElementById(
                        "requestResult"
                    ).textContent =
                        data.request;
                }

                if (data.lastRequest !== undefined)
                {
                    document.getElementById(
                        "requestStatus"
                    ).textContent =
                        "Last request: " +
                        data.lastRequest;
                }


                document.getElementById(
                    "connection"
                ).textContent =
                    "● ESP32 CONNECTED";

            }

            catch(error)
            {
                document.getElementById(
                    "connection"
                ).textContent =
                    "● CONNECTION LOST";
            }
        }


        // ====================================================
        // SEND DASHBOARD REQUEST
        // ====================================================

        async function sendRequest(command)
        {
            if (requestBusy)
            {
                return;
            }

            requestBusy = true;

            document.getElementById(
                "requestStatus"
            ).textContent =
                "Sending request: " + command + "...";


            const buttons =
                document.querySelectorAll("button");

            buttons.forEach(
                function(button)
                {
                    button.disabled = true;
                }
            );


            try
            {
                const response =
                    await fetch(
                        "/request?cmd=" +
                        encodeURIComponent(command),
                        {
                            cache: "no-store"
                        }
                    );


                if (!response.ok)
                {
                    throw new Error("HTTP error");
                }


                const data =
                    await response.json();


                if (data.status !== undefined)
                {
                    document.getElementById(
                        "requestStatus"
                    ).textContent =
                        data.status;
                }


                if (data.result !== undefined)
                {
                    document.getElementById(
                        "requestResult"
                    ).textContent =
                        data.result;
                }

            }

            catch(error)
            {
                document.getElementById(
                    "requestStatus"
                ).textContent =
                    "Request communication failed.";

                document.getElementById(
                    "requestResult"
                ).textContent =
                    "No result received.";

            }

            finally
            {
                requestBusy = false;

                buttons.forEach(
                    function(button)
                    {
                        button.disabled = false;
                    }
                );

                updateDashboard();
            }
        }


        // ====================================================
        // DASHBOARD REFRESH
        // ====================================================

        // Faster than the previous 1-second refresh.
        // The ESP32 still controls the real diagnostic timing.
        setInterval(
            updateDashboard,
            500
        );


        // First update immediately
        updateDashboard();

    </script>

</body>

</html>
)rawliteral";


// ============================================================
// ESCAPE JSON STRING
// ============================================================

String escapeJsonString(
    const String &input
)
{
    String output;

    output.reserve(
        input.length() + 20
    );

    for (size_t i = 0;
         i < input.length();
         i++)
    {
        char c = input[i];

        switch (c)
        {
            case '\\':
                output += "\\\\";
                break;

            case '"':
                output += "\\\"";
                break;

            case '\n':
                output += "\\n";
                break;

            case '\r':
                output += "\\r";
                break;

            case '\t':
                output += "\\t";
                break;

            default:
                output += c;
                break;
        }
    }

    return output;
}


// ============================================================
// JSON DATA ENDPOINT
// ============================================================

void handleDashboardData()
{
    String response;

    response.reserve(
        dashboardJSON.length() +
        dashboardRawCAN.length() +
        dashboardEvents.length() +
        dashboardRequestResult.length() +
        300
    );


    response += "{";


    response += "\"rpm\":";
    response += String(
        dashboardRPM,
        0
    );


    response += ",\"speed\":";
    response += String(
        dashboardSpeed,
        1
    );


    response += ",\"coolant\":";
    response += String(
        dashboardCoolant,
        1
    );


    response += ",\"throttle\":";
    response += String(
        dashboardThrottle,
        1
    );


    response += ",\"json\":\"";
    response += escapeJsonString(
        dashboardJSON
    );
    response += "\"";


    response += ",\"rawcan\":\"";
    response += escapeJsonString(
        dashboardRawCAN
    );
    response += "\"";


    response += ",\"events\":\"";
    response += escapeJsonString(
        dashboardEvents
    );
    response += "\"";


    response += ",\"request\":\"";
    response += escapeJsonString(
        dashboardRequestResult
    );
    response += "\"";


    response += ",\"lastRequest\":\"";
    response += escapeJsonString(
        dashboardLastRequest
    );
    response += "\"";


    response += "}";


    server.send(
        200,
        "application/json",
        response
    );
}


// ============================================================
// UPDATE DASHBOARD DATA
// ============================================================

void updateDashboardData(
    float rpm,
    float speed,
    float coolant,
    float throttle,
    const String &jsonData,
    const String &rawCanData,
    const String &eventData
)
{
    dashboardRPM =
        rpm;

    dashboardSpeed =
        speed;

    dashboardCoolant =
        coolant;

    dashboardThrottle =
        throttle;


    dashboardJSON =
        jsonData;

    dashboardRawCAN =
        rawCanData;

    dashboardEvents =
        eventData;
}


// ============================================================
// GET PENDING DASHBOARD REQUEST
// Returns the command name, or "" if nothing is pending.
// ============================================================

String getPendingDashboardRequest()
{
    int request =
        pendingDashboardRequest;

    pendingDashboardRequest =
        0;

    switch (request)
    {
        case 1: return "rpm";
        case 2: return "speed";
        case 3: return "coolant";
        case 4: return "throttle";
        case 5: return "vin";
        case 6: return "dtc";
        case 7: return "session";
    }

    return "";
}


// ============================================================
// UPDATE DASHBOARD REQUEST RESULT
// ============================================================

void updateDashboardRequestResult(
    const String &requestName,
    const String &result
)
{
    dashboardLastRequest =
        requestName;

    dashboardRequestResult =
        result;

    dashboardEvents =
        "Dashboard request completed:\n" +
        requestName;
}


// ============================================================
// HANDLE REQUEST ENDPOINT
// ============================================================

void handleDashboardRequest()
{
    if (!server.hasArg("cmd"))
    {
        server.send(
            400,
            "application/json",
            "{\"status\":\"Missing command.\",\"result\":\"No command received.\"}"
        );

        return;
    }


    String command =
        server.arg("cmd");


    command.toLowerCase();


    int requestCode =
        0;


    // --------------------------------------------------------
    // OBD-II
    // --------------------------------------------------------

    if (command == "rpm")
    {
        requestCode = 1;
    }

    else if (command == "speed")
    {
        requestCode = 2;
    }

    else if (command == "coolant")
    {
        requestCode = 3;
    }

    else if (command == "throttle")
    {
        requestCode = 4;
    }


    // --------------------------------------------------------
    // UDS
    // --------------------------------------------------------

    else if (command == "vin")
    {
        requestCode = 5;
    }

    else if (command == "dtc")
    {
        requestCode = 6;
    }

    else if (command == "session")
    {
        requestCode = 7;
    }


    // --------------------------------------------------------
    // Unknown command
    // --------------------------------------------------------

    else
    {
        server.send(
            400,
            "application/json",
            "{\"status\":\"Unknown request.\",\"result\":\"Command not implemented.\"}"
        );

        return;
    }


    // --------------------------------------------------------
    // Only one request is queued at a time.
    // --------------------------------------------------------

    if (pendingDashboardRequest != 0)
    {
        server.send(
            409,
            "application/json",
            "{\"status\":\"Request already pending.\",\"result\":\"Please wait for the current diagnostic operation.\"}"
        );

        return;
    }


    pendingDashboardRequest =
        requestCode;


    dashboardLastRequest =
        command;

    dashboardRequestResult =
        "Request sent to diagnostic core.\nWaiting for ECU response...";


    String response =
        "{\"status\":\"Request queued.\",\"result\":\"";

    response += escapeJsonString(
        dashboardRequestResult
    );

    response += "\"}";


    server.send(
        200,
        "application/json",
        response
    );
}


// ============================================================
// START WEB DASHBOARD
// ============================================================

void startWebDashboard()
{
    // ESP32 creates its own Wi-Fi network
    WiFi.mode(
        WIFI_AP
    );

    WiFi.softAP(
        "CAR-MUNICATION"
    );


    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "     CAR-MUNICATION DASHBOARD"
    );

    Serial.println(
        "================================"
    );


    Serial.print(
        "WiFi SSID: "
    );

    Serial.println(
        "CAR-MUNICATION"
    );


    Serial.print(
        "Dashboard IP: "
    );

    Serial.println(
        WiFi.softAPIP()
    );


    // --------------------------------------------------------
    // Main webpage
    // --------------------------------------------------------

    server.on(
        "/",
        HTTP_GET,
        []()
        {
            server.send_P(
                200,
                "text/html",
                dashboardHTML
            );
        }
    );


    // --------------------------------------------------------
    // Live diagnostic data
    // --------------------------------------------------------

    server.on(
        "/data",
        HTTP_GET,
        []()
        {
            handleDashboardData();
        }
    );


    // --------------------------------------------------------
    // Dashboard diagnostic request
    // --------------------------------------------------------

    server.on(
        "/request",
        HTTP_GET,
        []()
        {
            handleDashboardRequest();
        }
    );


    server.begin();


    Serial.println(
        "Web server started."
    );

    Serial.println(
        "Live data endpoint: /data"
    );

    Serial.println(
        "Diagnostic request endpoint: /request"
    );

    Serial.println(
        "Dashboard refresh: 500 ms"
    );

    Serial.println(
        "================================"
    );
}


// ============================================================
// HANDLE WEB REQUESTS
// ============================================================

void handleWebDashboard()
{
    server.handleClient();
}