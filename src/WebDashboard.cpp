#include "WebDashboard.h"

#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

String dashboardLiveJSON     = "{}";
String dashboardReadingsJSON = "{}";
String dashboardCanFeed      = "Waiting for CAN frames...\n";
String dashboardActionFeed   = "Waiting for CAN activity...\n";
String dashboardEvents       = "Dashboard started.";
String dashboardRequestResult = "No dashboard request yet.";
String dashboardLastRequest   = "None";

float dashboardObdDist = 0.0f;
float dashboardUdsDist = -1.0f;

volatile int pendingDashboardRequest = 0;

// ============================================================
// DASHBOARD WEBPAGE
// ============================================================
const char dashboardHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>GOLF-8 DIAG</title>
<style>
    body { font-family: Arial, sans-serif; background: #111; color: white;
           margin: 0; padding: 20px; }
    h1 { text-align: center; margin: 0 0 10px 0; }
    .status { text-align: center; margin-bottom: 20px; color: #aaa; }
    .dashboard { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
    .zone { background: #1c1c1c; border: 1px solid #333; border-radius: 10px;
            padding: 15px; min-height: 180px; }
    .zone h2 { margin-top: 0; border-bottom: 1px solid #333;
               padding-bottom: 10px; font-size: 16px; }
    .wide { grid-column: 1 / -1; }
    pre { white-space: pre-wrap; word-wrap: break-word; font-size: 12px;
          max-height: 320px; overflow-y: auto; margin: 0;
          font-family: Consolas, monospace; }

    /* LIVE GRID */
    .live-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; max-height: 400px; overflow-y: auto; padding-right: 5px; }
    .live-row { display: flex; justify-content: space-between;
                padding: 5px 8px; background: #151515; border-radius: 5px;
                font-size: 13px; }
    .live-label { color: #aaa; }
    .live-value { font-weight: bold; color: #7fd6a5; }
    .live-value.missing { color: #555; }

    /* DISTANCE BATTERIES */
    .dist-container { display: flex; justify-content: space-around; align-items: center; padding: 10px 0; }
    .battery-box { text-align: center; }
    .battery-box p { margin: 10px 0 0 0; font-weight: bold; font-size: 16px; }
    
    .battery { 
        width: 120px; height: 60px; 
        border: 4px solid #333; border-radius: 6px; 
        position: relative; background: #000;
        padding: 4px;
    }
    .battery::after {
        content: ''; position: absolute; right: -12px; top: 15px;
        width: 8px; height: 22px; background: #333; border-radius: 0 3px 3px 0;
    }
    .battery-fill {
        height: 100%; width: 0%; transition: width 0.2s, background 0.3s;
        border-radius: 2px;
    }
    .battery-fill.green { background: #26d04c; }
    .battery-fill.red { background: #e53935; }
    .battery-fill.full { width: 100%; box-shadow: 0 0 15px #26d04c; }
    
    .bolt {
        position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
        font-size: 32px; color: rgba(0,0,0,0.6); pointer-events: none;
        z-index: 2;
    }

    /* FEEDS */
    #canfeed { color: #9ecbff; background: #0c0c0c; border-radius: 6px; padding: 8px; }
    #actionfeed { color: #ffd28b; background: #0c0c0c; border-radius: 6px; padding: 8px; }
    #gpszone { color: #666; font-style: italic; text-align: center; padding: 30px 10px; }

    /* Requests */
    .request-group { margin-bottom: 16px; }
    .request-group h3 { margin: 6px 0 10px 0; font-size: 14px; color: #ccc; }
    .button-row { display: flex; flex-wrap: wrap; gap: 8px; }
    button { background: #2b2b2b; color: white; border: 1px solid #555;
             border-radius: 7px; padding: 10px 14px; font-size: 14px;
             cursor: pointer; }
    button:hover { background: #3a3a3a; }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    button.hot { background: #4a2b2b; border-color: #a05555; }
    button.hot:hover { background: #5c3535; }
    button.kill { background: #6a1f1f; border-color: #c04040;
                  font-weight: bold; letter-spacing: 1px; }
    button.kill:hover { background: #7d2626; }

    .request-result { background: #0a0a0a; border: 1px solid #333;
                      border-radius: 7px; padding: 12px; margin-top: 15px;
                      white-space: pre-wrap; word-wrap: break-word;
                      min-height: 80px; font-family: Consolas, monospace;
                      font-size: 12px; color: #d0d0d0; }
    .request-status { margin-top: 8px; color: #aaa; font-size: 13px; }

    @media (max-width: 700px) {
        .dashboard { grid-template-columns: 1fr; }
        .live-grid { grid-template-columns: 1fr; }
    }
</style>
</head>
<body>

<h1>GOLF-8 DIAG</h1>
<div class="status">
    ESP32 Diagnostic Dashboard<br>
    <span id="connection">Connecting...</span>
</div>

<div class="dashboard">

    <!-- ZONE 1: LIVELY DISTANCE TRACKER -->
    <div class="zone wide">
        <h2>LIVELY DISTANCE TRACKER</h2>
        <div class="dist-container">
            <div class="battery-box">
                <div class="battery">
                    <div class="battery-fill green" id="obdBattery"></div>
                    <div class="bolt">⚡</div>
                </div>
                <p style="color:#26d04c">OBD 100m Target</p>
                <p style="font-size:14px; color:#aaa" id="obdText">0.0 m / 100 m</p>
            </div>
            
            <div class="battery-box">
                <div class="battery">
                    <div class="battery-fill red" id="udsBattery" style="width: 10%"></div>
                    <div class="bolt">⚡</div>
                </div>
                <p style="color:#e53935">UDS Odometer</p>
                <p style="font-size:14px; color:#aaa" id="udsText">-- km</p>
            </div>
        </div>
    </div>

    <!-- ZONE 2: LIVE VEHICLE DATA (Full 44 PIDs) -->
    <div class="zone">
        <h2>LIVE VEHICLE DATA</h2>
        <div class="live-grid" id="liveGrid">
            <div class="live-row">
                <span class="live-label">Waiting...</span>
                <span class="live-value missing">--</span>
            </div>
        </div>
    </div>

    <!-- ZONE 3: JSON -->
    <div class="zone">
        <h2>JSON OUTPUT (to app)</h2>
        <pre id="json">Waiting for data...</pre>
    </div>

    <!-- ZONE 4: DIAGNOSTIC CAN FEED -->
    <div class="zone wide">
        <h2>DIAGNOSTIC EVENTS / RAW CAN FRAMES</h2>
        <pre id="canfeed">Waiting for CAN frames...</pre>
    </div>

    <!-- ZONE 5: GPS -->
    <div class="zone wide">
        <h2>GPS</h2>
        <div id="gpszone">
            No GPS module connected.<br>
            This zone will populate once a GNSS module is attached.
        </div>
    </div>

    <!-- ZONE 6: REQUESTS -->
    <div class="zone wide">
        <h2>SEND DIAGNOSTIC REQUEST</h2>

        <div class="request-group">
            <h3>Stage II — Odometer & Actuator</h3>
            <div class="button-row">
                <button class="hot" onclick="sendRequest('odometer')">READ ODOMETER (UDS)</button>
                <button class="kill" onclick="killEngine()">KILL ENGINE</button>
            </div>
        </div>

        <div class="request-group">
            <h3>Stage I / II — Standard OBD & UDS</h3>
            <div class="button-row">
                <button onclick="sendRequest('rpm')">RPM</button>
                <button onclick="sendRequest('speed')">SPEED</button>
                <button onclick="sendRequest('pid0100')">PID 01 00</button>
                <button onclick="sendRequest('sgw')">SGW TEST</button>
            </div>
        </div>

        <div class="request-status" id="requestStatus">Ready.</div>
        <div class="request-result" id="requestResult">No dashboard request yet.</div>
    </div>

</div>

<script>

let requestBusy = false;

// FULL 44 PID DICTIONARY
const PID_META = {
    "04": { name: "Engine load",       unit: "%"    },
    "05": { name: "Coolant",           unit: "°C"   },
    "06": { name: "STFT B1",           unit: "%"    },
    "07": { name: "LTFT B1",           unit: "%"    },
    "08": { name: "STFT B2",           unit: "%"    },
    "09": { name: "LTFT B2",           unit: "%"    },
    "0A": { name: "Fuel pressure",     unit: "kPa"  },
    "0B": { name: "MAP",               unit: "kPa"  },
    "0C": { name: "RPM",               unit: "rpm"  },
    "0D": { name: "Speed",             unit: "km/h" },
    "0E": { name: "Timing advance",    unit: "°"    },
    "0F": { name: "Intake temp",       unit: "°C"   },
    "10": { name: "MAF",               unit: "g/s"  },
    "11": { name: "Throttle",          unit: "%"    },
    "1C": { name: "OBD standard",      unit: ""     },
    "1F": { name: "Engine runtime",    unit: "s"    },
    "21": { name: "Dist w/ MIL",       unit: "km"   },
    "22": { name: "Fuel rail",         unit: "kPa"  },
    "23": { name: "Fuel rail DI",      unit: "kPa"  },
    "2C": { name: "Commanded EGR",     unit: "%"    },
    "2D": { name: "EGR error",         unit: "%"    },
    "2E": { name: "Evap purge",        unit: "%"    },
    "2F": { name: "Fuel level",        unit: "%"    },
    "30": { name: "Warmups",           unit: ""     },
    "31": { name: "Dist since clr",    unit: "km"   },
    "32": { name: "Evap vapor",        unit: "Pa"   },
    "33": { name: "Barometric",        unit: "kPa"  },
    "3C": { name: "Cat temp B1S1",     unit: "°C"   },
    "42": { name: "Module voltage",    unit: "V"    },
    "43": { name: "Abs load",          unit: "%"    },
    "45": { name: "Rel throttle",      unit: "%"    },
    "46": { name: "Ambient temp",      unit: "°C"   },
    "47": { name: "Throttle B",        unit: "%"    },
    "48": { name: "Throttle C",        unit: "%"    },
    "49": { name: "Pedal D",           unit: "%"    },
    "4A": { name: "Pedal E",           unit: "%"    },
    "4B": { name: "Pedal F",           unit: "%"    },
    "4C": { name: "Cmd throttle",      unit: "%"    },
    "4D": { name: "MIL time",          unit: "min"  },
    "4E": { name: "Time since clr",    unit: "min"  },
    "51": { name: "Fuel type",         unit: ""     },
    "5C": { name: "Oil temp",          unit: "°C"   },
    "5D": { name: "Inj timing",        unit: "°"    },
    "5E": { name: "Fuel rate",         unit: "L/h"  }
};

function renderLiveZone(liveData)
{
    const grid = document.getElementById("liveGrid");
    let html = "";
    for (const pid in PID_META)
    {
        const meta = PID_META[pid];
        let displayValue = "--";
        let missing = true;

        if (liveData && liveData[pid] !== undefined && liveData[pid] !== null)
        {
            displayValue = liveData[pid];
            if (typeof displayValue === "number")
            {
                displayValue = displayValue.toFixed(1);
                if (meta.unit === "rpm" || meta.unit === "km/h" ||
                    meta.unit === "s"   || meta.unit === "km" ||
                    meta.unit === "min" || meta.unit === "")
                    displayValue = Math.round(liveData[pid]).toString();
            }
            if (meta.unit) displayValue += " " + meta.unit;
            missing = false;
        }

        html += '<div class="live-row">' +
                    '<span class="live-label">' + meta.name + '</span>' +
                    '<span class="live-value' + (missing ? ' missing' : '') + '">' +
                        displayValue + '</span>' +
                '</div>';
    }
    grid.innerHTML = html;
}

async function updateDashboard()
{
    try
    {
        const response = await fetch("/data", { cache: "no-store" });
        if (!response.ok) throw new Error("HTTP error");
        const data = await response.json();

        try { renderLiveZone(JSON.parse(data.live)); } catch (e) {}

        document.getElementById("json").textContent        = data.json;
        document.getElementById("canfeed").textContent     = data.can;
        document.getElementById("actionfeed").textContent  = data.action;

        // DISTANCE LOGIC
        let obd = data.obdDist || 0;
        let uds = data.udsDist || -1;

        // OBD Green Battery
        let pct = Math.min((obd / 100.0) * 100, 100);
        let obdBat = document.getElementById("obdBattery");
        obdBat.style.width = Math.max(pct, 4) + "%";
        
        if (obd >= 100) {
            obdBat.className = "battery-fill green full";
            document.getElementById("obdText").textContent = "100m REACHED! (" + Math.round(obd) + "m)";
            document.getElementById("obdText").style.color = "#fff";
        } else {
            obdBat.className = "battery-fill green";
            document.getElementById("obdText").textContent = Math.round(obd) + " m / 100 m";
        }

        // UDS Red Battery
        if (uds > 0) {
            document.getElementById("udsBattery").style.width = "100%";
            document.getElementById("udsText").textContent = (uds / 1000.0).toFixed(1) + " km";
            document.getElementById("udsText").style.color = "#fff";
        }

        if (data.request !== undefined)
            document.getElementById("requestResult").textContent = data.request;

        if (data.lastRequest !== undefined)
            document.getElementById("requestStatus").textContent = "Last request: " + data.lastRequest;

        document.getElementById("connection").textContent = "● ESP32 CONNECTED";
    }
    catch (error)
    {
        document.getElementById("connection").textContent = "● CONNECTION LOST";
    }
}

async function sendRequest(command)
{
    if (requestBusy) return;
    requestBusy = true;
    document.getElementById("requestStatus").textContent = "Sending request: " + command + "...";

    const buttons = document.querySelectorAll("button");
    buttons.forEach(b => b.disabled = true);

    try
    {
        const response = await fetch("/request?cmd=" + encodeURIComponent(command), { cache: "no-store" });
        const data = await response.json();
        if (data.status !== undefined) document.getElementById("requestStatus").textContent = data.status;
        if (data.result !== undefined) document.getElementById("requestResult").textContent = data.result;
    }
    catch (error) {
        document.getElementById("requestStatus").textContent = "Request communication failed.";
    }
    finally {
        requestBusy = false;
        buttons.forEach(b => b.disabled = false);
        updateDashboard();
    }
}

// ============================================================
// KILL ENGINE (Fake with safety interlock)
// ============================================================
async function killEngine()
{
    if (requestBusy) return;
    requestBusy = true;

    const status = document.getElementById("requestStatus");
    const result = document.getElementById("requestResult");

    status.textContent = "Sending KILL ENGINE command...";
    result.textContent = "Waiting for ECU acknowledgement...";

    const buttons = document.querySelectorAll("button");
    buttons.forEach(b => b.disabled = true);

    await new Promise(r => setTimeout(r, 1800));

    let currentObd = 0;
    try {
        const resp = await fetch("/data");
        const d = await resp.json();
        currentObd = d.obdDist || 0;
    } catch(e) {}

    status.textContent = "Command failed.";
    
    if (currentObd >= 100) {
        result.textContent =
            "KILL ENGINE — request rejected\n" +
            "------------------------------\n" +
            "TX -> 7E0: 03 31 01 01 00 00 00 00\n" +
            "RX <- 7E8: 03 7F 31 22\n\n" +
            "NRC 0x22 — Conditions Not Correct\n" +
            "SAFETY INTERLOCK ACTIVE:\n" +
            "Vehicle is in motion (" + Math.round(currentObd) + "m traveled).\n" +
            "Engine kill is blocked by OEM safety protocols.\n" +
            "Stop the vehicle to enable actuator routines.";
    } else {
        result.textContent =
            "KILL ENGINE — request rejected\n" +
            "------------------------------\n" +
            "TX -> 7E0: 03 31 01 01 00 00 00 00\n" +
            "RX <- 7E8: 03 7F 31 22\n\n" +
            "NRC 0x22 — Conditions Not Correct\n" +
            "Actuator routine is not available in\n" +
            "the current diagnostic session.\n\n" +
            "Try again after entering extended session\n" +
            "and unlocking SFD (0x27).";
    }

    requestBusy = false;
    buttons.forEach(b => b.disabled = false);
    updateDashboard();
}

setInterval(updateDashboard, 250);
updateDashboard();

</script>

</body>
</html>
)rawliteral";

// ============================================================
String escapeJsonString(const String &input)
{
    String output;
    output.reserve(input.length() + 20);
    for (size_t i = 0; i < input.length(); i++)
    {
        char c = input[i];
        switch (c)
        {
            case '\\': output += "\\\\"; break;
            case '"':  output += "\\\""; break;
            case '\n': output += "\\n";  break;
            case '\r': output += "\\r";  break;
            case '\t': output += "\\t";  break;
            default:   output += c;      break;
        }
    }
    return output;
}

void handleDashboardData()
{
    String response;
    response.reserve(1500);

    response += "{";
    response += "\"live\":\"";    response += escapeJsonString(dashboardLiveJSON);    response += "\"";
    response += ",\"json\":\"";   response += escapeJsonString(dashboardReadingsJSON);response += "\"";
    response += ",\"can\":\"";    response += escapeJsonString(dashboardCanFeed);     response += "\"";
    response += ",\"action\":\""; response += escapeJsonString(dashboardActionFeed);  response += "\"";
    response += ",\"request\":\"";response += escapeJsonString(dashboardRequestResult);response += "\"";
    response += ",\"lastRequest\":\"";response += escapeJsonString(dashboardLastRequest);response += "\"";
    
    response += ",\"obdDist\":"; response += String(dashboardObdDist, 1);
    response += ",\"udsDist\":"; response += String(dashboardUdsDist, 1);
    
    response += "}";

    server.send(200, "application/json", response);
}

void updateDashboardData(
    const String &liveJSON,
    const String &readingsJSON,
    const String &canFeed,
    const String &actionFeed,
    const String &eventData,
    float obdDistMeters,
    float udsDistMeters)
{
    dashboardLiveJSON     = liveJSON;
    dashboardReadingsJSON = readingsJSON;
    dashboardCanFeed      = canFeed;
    dashboardActionFeed   = actionFeed;
    dashboardEvents       = eventData;
    dashboardObdDist      = obdDistMeters;
    dashboardUdsDist      = udsDistMeters;
}

void updateDashboardJSON(const String &readingsJSON)
{
    dashboardReadingsJSON = readingsJSON;
}

String getPendingDashboardRequest()
{
    int request = pendingDashboardRequest;
    pendingDashboardRequest = 0;
    switch (request)
    {
        case 1:  return "rpm";
        case 2:  return "speed";
        case 3:  return "coolant";
        case 4:  return "throttle";
        case 5:  return "vin";
        case 6:  return "dtc";
        case 7:  return "session";
        case 8:  return "sgw";
        case 9:  return "pid0100";
        case 10: return "pid0121";
        case 11: return "odometer";
    }
    return "";
}

void updateDashboardRequestResult(
    const String &requestName,
    const String &result)
{
    dashboardLastRequest = requestName;
    dashboardRequestResult = result;
    dashboardEvents = "Dashboard request completed: " + requestName;
}

void handleDashboardRequest()
{
    if (!server.hasArg("cmd"))
    {
        server.send(400, "application/json", "{\"status\":\"Missing command.\",\"result\":\"No command received.\"}");
        return;
    }

    String command = server.arg("cmd");
    command.toLowerCase();

    int requestCode = 0;
    if      (command == "rpm")      requestCode = 1;
    else if (command == "speed")    requestCode = 2;
    else if (command == "coolant")  requestCode = 3;
    else if (command == "throttle") requestCode = 4;
    else if (command == "vin")      requestCode = 5;
    else if (command == "dtc")      requestCode = 6;
    else if (command == "session")  requestCode = 7;
    else if (command == "sgw")      requestCode = 8;
    else if (command == "pid0100")  requestCode = 9;
    else if (command == "pid0121")  requestCode = 10;
    else if (command == "odometer") requestCode = 11;
    else
    {
        server.send(400, "application/json", "{\"status\":\"Unknown request.\",\"result\":\"Command not implemented.\"}");
        return;
    }

    if (pendingDashboardRequest != 0)
    {
        server.send(409, "application/json", "{\"status\":\"Request already pending.\",\"result\":\"Please wait.\"}");
        return;
    }

    pendingDashboardRequest = requestCode;
    dashboardLastRequest = command;
    dashboardRequestResult = "Request queued.\nWaiting for ECU response...";

    String response = "{\"status\":\"Request queued.\",\"result\":\"" + escapeJsonString(dashboardRequestResult) + "\"}";
    server.send(200, "application/json", response);
}

void startWebDashboard()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP("CAR-MUNICATION", "golf8diag2026");

    Serial.println("================================");
    Serial.println("     GOLF-8 DASHBOARD");
    Serial.println("================================");
    Serial.println("WiFi SSID: CAR-MUNICATION");
    Serial.println("WiFi PASS: golf8diag2026");
    Serial.print("Dashboard IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", dashboardHTML); });
    server.on("/data", HTTP_GET, []() { handleDashboardData(); });
    server.on("/request", HTTP_GET, []() { handleDashboardRequest(); });

    server.begin();
    Serial.println("Web server started.");
}

void handleWebDashboard()
{
    server.handleClient();
}