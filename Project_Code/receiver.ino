/*
 * ============================================================
 *  SMART FIRE EXTINGUISHER - RECEIVER (ESP32 Type-C 38-pin)
 * ============================================================
 *  Purpose : Receives LoRa data, hosts WiFi dashboard,
 *            displays status on OLED, sends commands back
 *
 * PIN CONNECTIONS:
 * ─────────────────────────────────────────────────────────
 * LoRa Ra-02 (VSPI default):
 *   SCK  → GPIO 18   MISO → GPIO 19
 *   MOSI → GPIO 23   CS   → GPIO 5
 *   RST  → GPIO 14   DIO0 → GPIO 2
 *
 * OLED 0.96" I2C:
 *   SDA  → GPIO 21   SCL  → GPIO 22
 *
 * WiFi: SSID = Nani   PASS = 1234567890
 * Dashboard: http://<ESP32_IP>
 * ─────────────────────────────────────────────────────────
 * LIBRARIES:
 *   LoRa by Sandeep Mistry
 *   Adafruit SSD1306 + GFX
 *   ESPAsyncWebServer (me-no-dev)
 *   AsyncTCP
 * ─────────────────────────────────────────────────────────
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h>

// ══════════════════════════════════════════
//  PIN DEFINITIONS
// ══════════════════════════════════════════
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_CS     5
#define LORA_RST   14
#define LORA_DIO0   2
#define OLED_SDA   21
#define OLED_SCL   22
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C

// ══════════════════════════════════════════
//  WIFI / NTP
// ══════════════════════════════════════════
const char* WIFI_SSID  = "Nani";
const char* WIFI_PASS  = "1234567890";
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET = 19800;    // IST = UTC+5:30
const int   DST_OFFSET = 0;

// ══════════════════════════════════════════
//  OBJECTS
// ══════════════════════════════════════════
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
AsyncWebServer   server(80);
AsyncWebSocket   ws("/ws");

// ══════════════════════════════════════════
//  DATA FROM TRANSMITTER
// ══════════════════════════════════════════
struct SensorData {
  float temp      = 0.0f;
  float hum       = 0.0f;
  int   gas       = 0;
  bool  flame     = false;
  bool  locked    = false;
  bool  manual    = false;
  bool  mServo    = false;
  bool  tempAlert = false;
  bool  gasAlert  = false;
  int   rssi      = 0;
  float snr       = 0.0f;
  unsigned long lastRx   = 0;
  unsigned long uptimeRx = 0;
  bool  connected = false;
  int   pktCount  = 0;
};
SensorData sd;

// ══════════════════════════════════════════
//  EVENT LOG  (circular buffer)
// ══════════════════════════════════════════
#define MAX_LOG 60
struct LogEntry {
  char time[10];
  char type[8];
  String event;
};
LogEntry eventLog[MAX_LOG];
int logHead  = 0;   // next write position
int logCount = 0;   // total entries written

// OLED
int oledPage = 0;
unsigned long tPage = 0, tOled = 0;

// Timing
unsigned long tWsUpdate = 0;
#define T_PAGE       4000
#define T_OLED        800
#define T_WS_UPDATE   400
#define LORA_TIMEOUT 15000

// ══════════════════════════════════════════
//  FORWARD DECLARATIONS
// ══════════════════════════════════════════
String getDashboardHTML();
String buildJSON();
String buildLogJSON();
void   sendLoRaCmd(const String &cmd);
void   addLog(const char *type, const String &event);
void   parsePacket(const String &pkt);
void   logAlerts();
void   rxLoRa(unsigned long now);
void   updateOLED(unsigned long now);
void   oledSplash(const String &title);
void   oledError(const String &msg);
void   oledMessage(const String &l1, const String &l2);
void   onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c,
                 AwsEventType type, void *arg, uint8_t *data, size_t len);

// ══════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== Smart Fire Extinguisher - RECEIVER ==="));

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    Serial.println(F("[ERR] OLED not found"));
  oledSplash("Receiver Node");

  // LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println(F("[ERR] LoRa FAILED"));
    oledError("LoRa FAIL");
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  Serial.println(F("[OK] LoRa 433 MHz ready"));

  // WiFi
  oledMessage("Connecting WiFi...", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500); Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET, DST_OFFSET, NTP_SERVER);
    String ip = WiFi.localIP().toString();
    Serial.println("\n[OK] WiFi: " + ip);
    oledMessage("WiFi Connected!", ip);
    addLog("INFO", "System online: " + ip);
  } else {
    Serial.println(F("\n[WARN] WiFi failed — no dashboard"));
    oledMessage("WiFi FAILED", "No Dashboard");
  }
  delay(1500);

  // WebSocket + HTTP
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "text/html", getDashboardHTML());
  });
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "application/json", buildJSON());
  });
  server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "application/json", buildLogJSON());
  });
  server.on("/api/cmd", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (r->hasParam("cmd", true)) {
      String cmd = r->getParam("cmd", true)->value();
      cmd.trim();
      sendLoRaCmd(cmd);
      r->send(200, "application/json", "{\"ok\":true,\"cmd\":\"" + cmd + "\"}");
    } else {
      r->send(400, "application/json", "{\"ok\":false,\"error\":\"Missing cmd\"}");
    }
  });

  server.begin();
  Serial.println(F("[OK] Web server started"));
  Serial.println("Dashboard: http://" + WiFi.localIP().toString());
}

// ══════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  rxLoRa(now);

  // Timeout check
  if (sd.connected && (now - sd.lastRx) > LORA_TIMEOUT) {
    sd.connected = false;
    addLog("INFO", "Transmitter offline (timeout)");
    Serial.println(F("[WARN] Transmitter timeout"));
  }

  // WebSocket broadcast
  if (now - tWsUpdate >= T_WS_UPDATE) {
    tWsUpdate = now;
    ws.cleanupClients();
    if (ws.count() > 0) {
      ws.textAll(buildJSON());
    }
  }

  // OLED update
  if (now - tOled >= T_OLED) {
    tOled = now;
    updateOLED(now);
  }
}

// ══════════════════════════════════════════
//  LORA RX — receive sensor data from TX
// ══════════════════════════════════════════
void rxLoRa(unsigned long now) {
  int sz = LoRa.parsePacket();
  if (!sz) return;

  String pkt = "";
  pkt.reserve(sz + 4);
  while (LoRa.available()) pkt += (char)LoRa.read();
  pkt.trim();

  sd.rssi   = LoRa.packetRssi();
  sd.snr    = LoRa.packetSnr();
  sd.lastRx = now;
  sd.pktCount++;
  if (!sd.connected) sd.uptimeRx = now;
  sd.connected = true;

  parsePacket(pkt);
  Serial.printf("[RX] %s | RSSI=%d SNR=%.1f\n", pkt.c_str(), sd.rssi, sd.snr);
  logAlerts();
}

// ══════════════════════════════════════════
//  PACKET PARSER
//  Format: T:xx.x,H:xx.x,G:xxxx,F:0,L:0,...
// ══════════════════════════════════════════
static String pktExtract(const String &pkt, const String &key) {
  String search = key + ":";
  int i = pkt.indexOf(search);
  if (i < 0) return "0";
  int start = i + search.length();
  int j = pkt.indexOf(',', start);
  return (j < 0) ? pkt.substring(start) : pkt.substring(start, j);
}
static float pktFloat(const String &p, const String &k) { return pktExtract(p, k).toFloat(); }
static int   pktInt  (const String &p, const String &k) { return (int)pktExtract(p, k).toInt(); }
static bool  pktBool (const String &p, const String &k) { return pktExtract(p, k) == "1"; }

void parsePacket(const String &pkt) {
  sd.temp      = pktFloat(pkt, "T");
  sd.hum       = pktFloat(pkt, "H");
  sd.gas       = pktInt  (pkt, "G");
  sd.flame     = pktBool (pkt, "F");
  sd.locked    = pktBool (pkt, "L");
  sd.manual    = pktBool (pkt, "MA");
  sd.mServo    = pktBool (pkt, "MS");
  sd.tempAlert = pktBool (pkt, "TA");
  sd.gasAlert  = pktBool (pkt, "GA");
}

// ══════════════════════════════════════════
//  ALERT LOGGING
// ══════════════════════════════════════════
void logAlerts() {
  static bool pFlame = false, pGas = false, pTemp = false, pLock = false;

  if ( sd.flame    && !pFlame) addLog("FLAME", "FLAME DETECTED - Extinguisher activated");
  if (!sd.flame    &&  pFlame) addLog("FLAME", "Flame cleared - System normal");
  if ( sd.gasAlert && !pGas)   addLog("GAS",   "Gas alert: ADC=" + String(sd.gas));
  if (!sd.gasAlert &&  pGas)   addLog("GAS",   "Gas level normal");
  if ( sd.tempAlert&& !pTemp)  addLog("TEMP",  "High temp: " + String(sd.temp, 1) + " C");
  if (!sd.tempAlert&&  pTemp)  addLog("TEMP",  "Temperature normal");
  if ( sd.locked   && !pLock)  addLog("RFID",  "System LOCKED via RFID");
  if (!sd.locked   &&  pLock)  addLog("RFID",  "System UNLOCKED via RFID");

  pFlame = sd.flame;
  pGas   = sd.gasAlert;
  pTemp  = sd.tempAlert;
  pLock  = sd.locked;
}

void addLog(const char *type, const String &event) {
  struct tm ti;
  char buf[10] = "--:--:--";
  if (getLocalTime(&ti)) strftime(buf, sizeof(buf), "%H:%M:%S", &ti);

  LogEntry &e = eventLog[logHead];
  strncpy(e.time, buf,  sizeof(e.time)  - 1);
  strncpy(e.type, type, sizeof(e.type)  - 1);
  e.time[sizeof(e.time) - 1] = '\0';
  e.type[sizeof(e.type) - 1] = '\0';
  e.event = event;

  logHead = (logHead + 1) % MAX_LOG;
  if (logCount < MAX_LOG) logCount++;

  Serial.println("[LOG][" + String(type) + "] " + event);
}

// ══════════════════════════════════════════
//  LORA TX — send command to transmitter
// ══════════════════════════════════════════
void sendLoRaCmd(const String &cmd) {
  LoRa.beginPacket();
  LoRa.print(cmd);
  LoRa.endPacket();
  Serial.println("[TX CMD] " + cmd);
  addLog("CMD", "Sent: " + cmd);
}

// ══════════════════════════════════════════
//  WEBSOCKET
// ══════════════════════════════════════════
void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u connected\n", c->id());
    c->text(buildJSON());   // send current data immediately
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String msg((char*)data, len);
      msg.trim();
      Serial.println("[WS RX] " + msg);
      if (msg.startsWith("CMD:")) {
        String cmd = msg.substring(4);
        cmd.trim();
        sendLoRaCmd(cmd);
        ws.textAll("{\"cmdEcho\":\"" + cmd + "\"}");
      }
    }
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected\n", c->id());
  }
}

// ══════════════════════════════════════════
//  JSON BUILDERS
// ══════════════════════════════════════════
String buildJSON() {
  struct tm ti;
  char ts[25] = "--:--:--";
  if (getLocalTime(&ti)) strftime(ts, sizeof(ts), "%d/%m/%Y %H:%M:%S", &ti);

  unsigned long now   = millis();
  unsigned long upSec = (sd.connected && sd.uptimeRx > 0) ? (now - sd.uptimeRx) / 1000UL : 0;

  String j = "{";
  j += "\"time\":\""    + String(ts)                         + "\",";
  j += "\"temp\":"      + String(sd.temp, 1)                 + ",";
  j += "\"hum\":"       + String(sd.hum,  1)                 + ",";
  j += "\"gas\":"       + String(sd.gas)                     + ",";
  j += "\"flame\":"     + String(sd.flame     ? "true" : "false") + ",";
  j += "\"locked\":"    + String(sd.locked    ? "true" : "false") + ",";
  j += "\"manual\":"    + String(sd.manual    ? "true" : "false") + ",";
  j += "\"mServo\":"    + String(sd.mServo    ? "true" : "false") + ",";
  j += "\"tempAlert\":" + String(sd.tempAlert ? "true" : "false") + ",";
  j += "\"gasAlert\":"  + String(sd.gasAlert  ? "true" : "false") + ",";
  j += "\"connected\":" + String(sd.connected ? "true" : "false") + ",";
  j += "\"rssi\":"      + String(sd.rssi)                    + ",";
  j += "\"snr\":"       + String(sd.snr, 1)                  + ",";
  j += "\"pktCount\":"  + String(sd.pktCount)                + ",";
  j += "\"uptime\":"    + String(upSec)                      + ",";
  j += "\"ip\":\""      + WiFi.localIP().toString()          + "\"";
  j += "}";
  return j;
}

// Returns log entries newest-first
String buildLogJSON() {
  String j = "[";
  bool first = true;
  int total = (logCount < MAX_LOG) ? logCount : MAX_LOG;
  for (int i = 0; i < total; i++) {
    // Walk backwards from most recent
    int idx = (logHead - 1 - i + MAX_LOG) % MAX_LOG;
    if (!first) j += ",";
    first = false;
    // Escape event string
    String ev = eventLog[idx].event;
    ev.replace("\"", "\\\"");
    j += "{\"t\":\"" + String(eventLog[idx].time) + "\","
       + "\"e\":\""  + ev                          + "\","
       + "\"k\":\""  + String(eventLog[idx].type)  + "\"}";
  }
  j += "]";
  return j;
}

// ══════════════════════════════════════════
//  OLED
// ══════════════════════════════════════════
void oledSplash(const String &title) {
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 12, WHITE);
  oled.setTextColor(BLACK); oled.setTextSize(1);
  oled.setCursor(5, 2); oled.print(F("SMART FIRE EXT"));
  oled.setTextColor(WHITE);
  oled.setCursor(10, 20); oled.println(title);
  oled.setCursor(10, 35); oled.println(F("Booting..."));
  oled.display();
  delay(1500);
}

void oledError(const String &msg) {
  oled.clearDisplay();
  oled.setTextColor(WHITE); oled.setTextSize(2);
  oled.setCursor(0, 10); oled.println(F("ERROR!"));
  oled.setTextSize(1); oled.setCursor(0, 38); oled.println(msg);
  oled.display();
}

void oledMessage(const String &l1, const String &l2) {
  oled.clearDisplay();
  oled.setTextColor(WHITE); oled.setTextSize(1);
  oled.setCursor(0, 20); oled.println(l1);
  oled.setCursor(0, 35); oled.println(l2);
  oled.display();
}

void updateOLED(unsigned long now) {
  if (now - tPage > T_PAGE) {
    tPage    = now;
    oledPage = (oledPage + 1) % 4;
  }

  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 11, WHITE);
  oled.setTextColor(BLACK); oled.setTextSize(1);
  const char* titles[] = {"LIVE SENSORS", "SYSTEM", "LORA STATUS", "ALERTS"};
  const uint8_t offsets[] = {18, 32, 20, 34};
  oled.setCursor(offsets[oledPage], 2);
  oled.print(titles[oledPage]);
  oled.setTextColor(WHITE);

  // Connection dot (top-right)
  if (sd.connected) oled.fillCircle(122, 5, 4, WHITE);
  else              oled.drawCircle(122, 5, 4, WHITE);

  int y = 14;
  switch (oledPage) {
    case 0:
      oled.setCursor(0, y);
      oled.printf("Temp : %.1f C\n",  sd.temp);
      oled.printf("Humid: %.1f %%\n", sd.hum);
      oled.printf("Gas  : %d\n",      sd.gas);
      oled.printf("Flame: %s",        sd.flame ? "!! YES !!" : "None");
      break;

    case 1:
      oled.setCursor(0, y);
      oled.printf("Servo: %s\n",  (sd.flame || (sd.manual && sd.mServo)) ? "ON  70deg" : "OFF 0deg");
      oled.printf("Mode : %s\n",  sd.manual  ? "MANUAL"  : "AUTO");
      oled.printf("Lock : %s\n",  sd.locked  ? "LOCKED"  : "OPEN");
      oled.printf("IP: %.15s",    WiFi.localIP().toString().c_str());
      break;

    case 2:
      oled.setCursor(0, y);
      oled.printf("RSSI : %d dBm\n", sd.rssi);
      oled.printf("SNR  : %.1f dB\n", sd.snr);
      oled.printf("Pkts : %d\n",     sd.pktCount);
      oled.printf("Link : %s",       sd.connected ? "ONLINE" : "OFFLINE");
      break;

    case 3:
      oled.setCursor(0, y);
      oled.printf("[%s] FLAME\n",  sd.flame     ? "!!" : "--");
      oled.printf("[%s] GAS\n",    sd.gasAlert  ? "!!" : "--");
      oled.printf("[%s] TEMP\n",   sd.tempAlert ? "!!" : "--");
      oled.printf("[%s] LOCKED",   sd.locked    ? "!!" : "--");
      break;
  }
  oled.display();
}

// ══════════════════════════════════════════
//  DASHBOARD HTML
// ══════════════════════════════════════════
String getDashboardHTML() {
  return R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>🔥 Smart Fire Extinguisher</title>
<style>
:root{
  --bg:#060b14;--card:#0d1520;--card2:#111d2e;--border:#1e2d42;
  --accent:#ef4444;--blue:#3b82f6;--green:#10b981;--yellow:#f59e0b;
  --purple:#8b5cf6;--cyan:#06b6d4;--orange:#f97316;
  --text:#e2e8f0;--muted:#64748b;--muted2:#94a3b8;
}
*{margin:0;padding:0;box-sizing:border-box;}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,Arial,sans-serif;min-height:100vh;overflow-x:hidden;}
body::before{content:'';position:fixed;inset:0;background:radial-gradient(ellipse at 20% 50%,#1a0a0022,transparent 60%),radial-gradient(ellipse at 80% 20%,#0a1a3322,transparent 60%);pointer-events:none;z-index:0;}
nav{background:#07101a;border-bottom:1px solid var(--border);padding:0 24px;display:flex;align-items:center;justify-content:space-between;height:64px;position:sticky;top:0;z-index:200;backdrop-filter:blur(12px);}
.nav-brand{display:flex;align-items:center;gap:12px;font-size:17px;font-weight:800;}
.nav-brand .fire{font-size:26px;animation:fireFlicker 1.8s ease-in-out infinite alternate;}
@keyframes fireFlicker{from{filter:drop-shadow(0 0 6px #ef4444);transform:scale(1);}to{filter:drop-shadow(0 0 14px #f59e0b);transform:scale(1.08);}}
.brand-text{background:linear-gradient(135deg,#ef4444,#f97316);-webkit-background-clip:text;-webkit-text-fill-color:transparent;}
.nav-right{display:flex;align-items:center;gap:20px;}
.nav-conn{display:flex;align-items:center;gap:8px;font-size:13px;padding:6px 14px;border-radius:20px;background:var(--card2);border:1px solid var(--border);}
.dot{width:9px;height:9px;border-radius:50%;background:#ef4444;transition:background .4s;}
.dot.online{background:var(--green);box-shadow:0 0 8px var(--green);}
.nav-time{color:var(--muted2);font-size:12px;font-family:monospace;}
.tab-bar{background:#07101a;border-bottom:1px solid var(--border);display:flex;gap:2px;padding:0 24px;overflow-x:auto;scrollbar-width:none;}
.tab-bar::-webkit-scrollbar{display:none;}
.tab{padding:13px 18px;cursor:pointer;color:var(--muted);font-size:13px;font-weight:600;border-bottom:3px solid transparent;transition:all .2s;white-space:nowrap;user-select:none;}
.tab:hover{color:var(--text);}
.tab.active{color:var(--accent);border-bottom-color:var(--accent);}
.page{display:none;padding:20px;position:relative;z-index:1;animation:fadeIn .3s ease;}
.page.active{display:block;}
@keyframes fadeIn{from{opacity:0;transform:translateY(8px);}to{opacity:1;transform:translateY(0);}}
.grid{display:grid;gap:14px;}
.g4{grid-template-columns:repeat(4,1fr);}
.g3{grid-template-columns:repeat(3,1fr);}
.g2{grid-template-columns:1fr 1fr;}
.card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:18px;transition:border-color .3s;position:relative;overflow:hidden;}
.card::after{content:'';position:absolute;top:0;left:0;right:0;height:1px;background:linear-gradient(90deg,transparent,rgba(255,255,255,.06),transparent);}
.card:hover{border-color:#2a3f5a;}
.card-title{font-size:11px;text-transform:uppercase;letter-spacing:1.2px;color:var(--muted);margin-bottom:10px;display:flex;align-items:center;gap:6px;}
.card-val{font-size:34px;font-weight:800;line-height:1;transition:color .4s;}
.card-unit{font-size:13px;color:var(--muted);margin-top:3px;}
.card-sub{font-size:12px;color:var(--muted);margin-top:6px;}
.badge{display:inline-flex;align-items:center;gap:5px;padding:4px 12px;border-radius:20px;font-size:11px;font-weight:700;letter-spacing:.4px;}
.b-ok  {background:#052e1c;color:#34d399;border:1px solid #065f3744;}
.b-alert{background:#3f0606;color:#f87171;border:1px solid #7f1d1d44;animation:badgePulse 1.5s infinite;}
.b-warn{background:#3d1c00;color:#fbbf24;border:1px solid #92400e44;}
.b-info{background:#0c1f40;color:#93c5fd;border:1px solid #1e3a5f44;}
.b-lock{background:#2c1654;color:#c4b5fd;border:1px solid #4c1d9544;}
.b-muted{background:#1f2937;color:var(--muted2);border:1px solid #37415144;}
@keyframes badgePulse{0%,100%{box-shadow:0 0 0 0 #ef444422;}50%{box-shadow:0 0 0 4px transparent;}}
.section-h{font-size:15px;font-weight:700;margin-bottom:14px;display:flex;align-items:center;gap:10px;color:var(--text);}
.section-h::before{content:'';width:4px;height:18px;background:linear-gradient(180deg,var(--accent),var(--orange));border-radius:2px;display:block;}
.btn{padding:11px 22px;border:none;border-radius:9px;cursor:pointer;font-size:13px;font-weight:700;transition:all .2s;display:inline-flex;align-items:center;gap:8px;letter-spacing:.3px;}
.btn-red  {background:linear-gradient(135deg,#dc2626,#b91c1c);color:#fff;box-shadow:0 4px 12px #dc262640;}
.btn-green{background:linear-gradient(135deg,#059669,#047857);color:#fff;box-shadow:0 4px 12px #05966940;}
.btn-blue {background:linear-gradient(135deg,#2563eb,#1d4ed8);color:#fff;box-shadow:0 4px 12px #2563eb40;}
.btn-gray {background:linear-gradient(135deg,#374151,#1f2937);color:#fff;}
.btn:hover{transform:translateY(-2px);filter:brightness(1.1);}
.btn:active{transform:translateY(0);}
.btn:disabled{opacity:.35;cursor:not-allowed;transform:none;filter:none;}
.alert-card{border-left:4px solid var(--border);transition:background .4s,border-color .4s;}
.alert-card.flame{border-left-color:#ef4444;background:#1c0606;}
.alert-card.gas{border-left-color:#f59e0b;background:#1c0f00;}
.alert-card.temp{border-left-color:#f97316;background:#1c0800;}
canvas{width:100%;display:block;border-radius:8px;}
.log-table{width:100%;border-collapse:collapse;font-size:13px;}
.log-table th{background:#0d1a28;padding:10px 12px;text-align:left;font-size:11px;text-transform:uppercase;letter-spacing:1px;color:var(--muted);border-bottom:1px solid var(--border);}
.log-table td{padding:9px 12px;border-bottom:1px solid #1a2535;}
.log-table tr:hover td{background:#111d2c;}
.servo-wrap{display:flex;flex-direction:column;align-items:center;gap:10px;padding:12px 0;}
.servo-arc{width:90px;height:46px;border:3px solid #2a3f5a;border-bottom:none;border-radius:46px 46px 0 0;position:relative;}
.servo-needle{position:absolute;bottom:0;left:50%;width:3px;height:40px;background:var(--accent);transform-origin:bottom center;transform:translateX(-50%) rotate(-90deg);transition:transform .9s cubic-bezier(.4,0,.2,1);border-radius:2px;}
.flame-ic{font-size:52px;transition:all .3s;display:inline-block;}
.flame-ic.on{animation:fAnim .5s ease-in-out infinite alternate;}
@keyframes fAnim{from{transform:scale(1);filter:drop-shadow(0 0 6px #ef4444);}to{transform:scale(1.18) rotate(5deg);filter:drop-shadow(0 0 18px #f59e0b);}}
.prog-bar{height:7px;border-radius:4px;background:#1e2d42;overflow:hidden;margin-top:8px;}
.prog-fill{height:100%;border-radius:4px;transition:width .6s ease,background .4s;}
.lora-bar{height:10px;border-radius:5px;background:#1e2d42;overflow:hidden;}
.lora-fill{height:100%;border-radius:5px;background:linear-gradient(90deg,#ef4444 0%,#f59e0b 40%,#10b981 100%);transition:width .5s;}
.alarm-banner{display:none;padding:12px 20px;background:linear-gradient(90deg,#7f1d1d,#3f0606);border:1px solid #ef4444;border-radius:10px;margin:10px 20px;font-size:14px;font-weight:700;color:#fca5a5;animation:alarmPulse 1s infinite;}
.alarm-banner.show{display:flex;align-items:center;gap:12px;}
@keyframes alarmPulse{0%,100%{box-shadow:0 0 0 0 #ef444440;}50%{box-shadow:0 0 0 8px transparent;}}
.lock-warn{display:none;padding:12px 16px;background:#1a0d00;border:1px solid #f59e0b;border-radius:9px;margin-bottom:14px;font-size:13px;color:#fcd34d;}
.lock-warn.show{display:flex;align-items:center;gap:10px;}
.cmd-status{font-size:13px;color:var(--green);margin-top:10px;height:18px;transition:opacity .5s;}
.stat-card{background:var(--card2);border:1px solid var(--border);border-radius:10px;padding:14px 16px;text-align:center;}
.stat-val{font-size:28px;font-weight:800;}
.stat-label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.8px;margin-top:3px;}
.info-table{width:100%;font-size:13px;border-collapse:collapse;}
.info-table td{padding:7px 0;border-bottom:1px solid #1a2535;}
.info-table td:first-child{color:var(--muted);width:45%;}
.info-table td:last-child{font-weight:600;color:var(--text);}
.info-table tr:last-child td{border-bottom:none;}
@media(max-width:900px){.g4{grid-template-columns:1fr 1fr;}.g3{grid-template-columns:1fr 1fr;}}
@media(max-width:600px){.g4,.g3,.g2{grid-template-columns:1fr;}.tab{padding:10px 12px;font-size:12px;}}
</style>
</head>
<body>

<nav>
  <div class="nav-brand">
    <span class="fire">🔥</span>
    <span class="brand-text">Smart Fire Extinguisher</span>
  </div>
  <div class="nav-right">
    <div class="nav-conn">
      <div class="dot" id="statusDot"></div>
      <span id="statusText" style="font-size:12px;font-weight:600;">Connecting...</span>
    </div>
    <div class="nav-time" id="navTime">--:--:--</div>
  </div>
</nav>

<div class="alarm-banner" id="alarmBanner">
  <span style="font-size:22px;">🚨</span>
  <span id="alarmText">FIRE ALERT! Extinguisher Activated</span>
</div>

<div class="tab-bar">
  <div class="tab active" onclick="switchTab(0)">🏠 Home</div>
  <div class="tab" onclick="switchTab(1)">📊 Monitoring</div>
  <div class="tab" onclick="switchTab(2)">🧯 Extinguisher</div>
  <div class="tab" onclick="switchTab(3)">📡 LoRa</div>
  <div class="tab" onclick="switchTab(4)">🕹 Manual</div>
  <div class="tab" onclick="switchTab(5)">🔒 Security</div>
</div>

<!-- HOME -->
<div class="page active" id="page0">
  <div class="grid g4" style="margin-bottom:14px;">
    <div class="card">
      <div class="card-title">🌡 Temperature</div>
      <div class="card-val" id="h-temp" style="color:var(--orange)">--</div>
      <div class="card-unit">°C</div>
      <div class="prog-bar"><div class="prog-fill" id="h-tempBar" style="width:0%;background:var(--orange)"></div></div>
      <div class="card-sub" id="h-tempSub">Loading...</div>
    </div>
    <div class="card">
      <div class="card-title">💧 Humidity</div>
      <div class="card-val" id="h-hum" style="color:var(--blue)">--</div>
      <div class="card-unit">%</div>
      <div class="prog-bar"><div class="prog-fill" id="h-humBar" style="width:0%;background:var(--blue)"></div></div>
      <div class="card-sub">Relative Humidity</div>
    </div>
    <div class="card">
      <div class="card-title">💨 Gas (ADC)</div>
      <div class="card-val" id="h-gas" style="color:var(--yellow)">--</div>
      <div class="card-unit">raw / 4095</div>
      <div class="prog-bar"><div class="prog-fill" id="h-gasBar" style="width:0%;background:var(--yellow)"></div></div>
      <div class="card-sub" id="h-gasSub">--</div>
    </div>
    <div class="card alert-card" id="h-flameCard">
      <div class="card-title">🔥 Flame</div>
      <div style="text-align:center;padding:4px 0;">
        <div class="flame-ic" id="h-flameIcon">🔵</div>
      </div>
      <div style="text-align:center;"><span id="h-flameBadge" class="badge b-ok">NO FLAME</span></div>
    </div>
  </div>
  <div class="grid g3">
    <div class="card" style="grid-column:span 2;">
      <div class="section-h">System Overview</div>
      <div class="grid g2" style="margin-bottom:14px;">
        <div>
          <div class="card-title">Extinguisher</div>
          <span id="h-extStatus" class="badge b-ok">STANDBY</span>
          <div style="height:14px"></div>
          <div class="card-title">Operating Mode</div>
          <span id="h-modeStatus" class="badge b-info">AUTO</span>
        </div>
        <div>
          <div class="card-title">RFID Lock</div>
          <span id="h-lockStatus" class="badge b-ok">UNLOCKED</span>
          <div style="height:14px"></div>
          <div class="card-title">LoRa Link</div>
          <span id="h-loraStatus" class="badge b-alert">OFFLINE</span>
        </div>
      </div>
      <div class="grid g4">
        <div class="stat-card"><div class="stat-val" id="h-rssi">--</div><div class="stat-label">RSSI dBm</div></div>
        <div class="stat-card"><div class="stat-val" id="h-snr">--</div><div class="stat-label">SNR dB</div></div>
        <div class="stat-card"><div class="stat-val" id="h-pkts">0</div><div class="stat-label">Packets</div></div>
        <div class="stat-card"><div class="stat-val" id="h-uptime">--</div><div class="stat-label">Uptime</div></div>
      </div>
    </div>
    <div class="card">
      <div class="section-h">Active Alerts</div>
      <div id="h-alerts"><div style="color:var(--muted);font-size:13px;">✅ All Clear</div></div>
      <div style="margin-top:16px;padding-top:14px;border-top:1px solid var(--border);">
        <div class="card-title">Last Update</div>
        <div id="h-lastUpdate" style="font-size:12px;font-family:monospace;color:var(--muted2);">--</div>
      </div>
    </div>
  </div>
</div>

<!-- MONITORING -->
<div class="page" id="page1">
  <div class="section-h">Real-Time Sensor Monitoring</div>
  <div class="grid g3" style="margin-bottom:14px;">
    <div class="card">
      <div class="card-title">🌡 Temperature (°C)</div>
      <div style="display:flex;align-items:baseline;gap:10px;">
        <div class="card-val" id="m-temp">--</div>
        <span id="m-tempBadge" class="badge b-ok">NORMAL</span>
      </div>
      <div class="prog-bar" style="margin:10px 0;"><div class="prog-fill" id="m-tempBar" style="width:0%;background:#ef4444"></div></div>
      <canvas id="tempChart" height="65"></canvas>
    </div>
    <div class="card">
      <div class="card-title">💧 Humidity (%)</div>
      <div style="display:flex;align-items:baseline;gap:10px;">
        <div class="card-val" id="m-hum">--</div>
        <span class="badge b-info">RELATIVE</span>
      </div>
      <div class="prog-bar" style="margin:10px 0;"><div class="prog-fill" id="m-humBar" style="width:0%;background:#3b82f6"></div></div>
      <canvas id="humChart" height="65"></canvas>
    </div>
    <div class="card">
      <div class="card-title">💨 Gas Level (ADC 0–4095)</div>
      <div style="display:flex;align-items:baseline;gap:10px;">
        <div class="card-val" id="m-gas">--</div>
        <span id="m-gasBadge" class="badge b-ok">NORMAL</span>
      </div>
      <div class="prog-bar" style="margin:10px 0;"><div class="prog-fill" id="m-gasBar" style="width:0%;background:#f59e0b"></div></div>
      <canvas id="gasChart" height="65"></canvas>
    </div>
  </div>
  <div class="grid g2">
    <div class="card">
      <div class="card-title">🔥 Flame Sensor (IR)</div>
      <div style="display:flex;align-items:center;gap:20px;margin-top:12px;">
        <div class="flame-ic" id="m-flameIcon">🔵</div>
        <div>
          <div style="font-size:20px;font-weight:700;" id="m-flameText">NO FLAME</div>
          <div style="color:var(--muted);font-size:12px;margin-top:4px;">Active-LOW sensor</div>
          <div style="color:var(--muted);font-size:12px;">Triggers servo + buzzer</div>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">📈 Combined History (last 20)</div>
      <div style="display:flex;gap:14px;margin-bottom:8px;flex-wrap:wrap;">
        <span style="font-size:11px;color:#ef4444;">⬛ Temp</span>
        <span style="font-size:11px;color:#3b82f6;">⬛ Hum</span>
        <span style="font-size:11px;color:#f59e0b;">⬛ Gas (scaled)</span>
      </div>
      <canvas id="combinedChart" height="90"></canvas>
    </div>
  </div>
</div>

<!-- EXTINGUISHER -->
<div class="page" id="page2">
  <div class="section-h">Fire Extinguisher Status</div>
  <div class="grid g3">
    <div class="card alert-card" id="e-flameCard">
      <div class="card-title">🔥 Flame Detection</div>
      <div class="servo-wrap"><div class="flame-ic" id="e-flameIcon">🔵</div></div>
      <div style="text-align:center;"><span id="e-flameBadge" class="badge b-ok">NO FLAME</span></div>
    </div>
    <div class="card">
      <div class="card-title">⚙️ Servo Position</div>
      <div class="servo-wrap">
        <div class="servo-arc"><div class="servo-needle" id="e-servoNeedle"></div></div>
        <div style="font-size:28px;font-weight:800;" id="e-servoDeg">90°</div>
        <span id="e-servoLabel" class="badge b-ok">🟢 STANDBY 0°</span>
      </div>
    </div>
    <div class="card">
      <div class="card-title">📋 Extinguisher Info</div>
      <table class="info-table">
        <tr><td>Status</td>      <td id="e-status">STANDBY</td></tr>
        <tr><td>Mode</td>        <td id="e-mode">AUTO</td></tr>
        <tr><td>RFID Lock</td>   <td id="e-lock">OPEN</td></tr>
        <tr><td>Temp Alert</td>  <td id="e-tempAlert">None</td></tr>
        <tr><td>Gas Alert</td>   <td id="e-gasAlert">None</td></tr>
        <tr><td>Temperature</td> <td id="e-temp">--</td></tr>
        <tr><td>Gas (ADC)</td>   <td id="e-gas">--</td></tr>
        <tr><td>Humidity</td>    <td id="e-hum">--</td></tr>
      </table>
    </div>
  </div>
</div>

<!-- LORA -->
<div class="page" id="page3">
  <div class="section-h">LoRa Communication</div>
  <div class="grid g4" style="margin-bottom:14px;">
    <div class="card">
      <div class="card-title">📶 RSSI</div>
      <div class="card-val" id="l-rssi">--</div>
      <div class="card-unit">dBm</div>
      <div class="card-sub">Signal Strength</div>
    </div>
    <div class="card">
      <div class="card-title">📡 SNR</div>
      <div class="card-val" id="l-snr">--</div>
      <div class="card-unit">dB</div>
      <div class="card-sub">Signal/Noise Ratio</div>
    </div>
    <div class="card">
      <div class="card-title">📦 Packets</div>
      <div class="card-val" id="l-pkts" style="color:var(--cyan)">0</div>
      <div class="card-unit">received</div>
    </div>
    <div class="card">
      <div class="card-title">🔗 Link Status</div>
      <div class="card-val" id="l-linkIcon">❌</div>
      <span id="l-linkBadge" class="badge b-alert">OFFLINE</span>
    </div>
  </div>
  <div class="grid g2">
    <div class="card">
      <div class="card-title">Signal History</div>
      <canvas id="rssiChart" height="100"></canvas>
      <div style="margin-top:14px;">
        <div style="display:flex;justify-content:space-between;font-size:12px;margin-bottom:5px;">
          <span>RSSI: <b id="l-rssiVal">--</b> dBm</span>
          <span id="l-qualityLabel" style="color:var(--green)">--</span>
        </div>
        <div class="lora-bar"><div class="lora-fill" id="l-rssiBarFill" style="width:0%"></div></div>
        <div style="display:flex;justify-content:space-between;font-size:11px;color:var(--muted);margin-top:3px;">
          <span>-120 dBm</span><span>-40 dBm</span>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">LoRa Configuration</div>
      <table class="info-table">
        <tr><td>Frequency</td>        <td>433 MHz</td></tr>
        <tr><td>Spreading Factor</td> <td>SF9</td></tr>
        <tr><td>Bandwidth</td>        <td>125 kHz</td></tr>
        <tr><td>Coding Rate</td>      <td>4/5</td></tr>
        <tr><td>TX Power (TX)</td>    <td>17 dBm</td></tr>
        <tr><td>Module</td>           <td>Ra-02 (AI-Thinker)</td></tr>
        <tr><td>TX Interval</td>      <td>3 seconds</td></tr>
        <tr><td>Timeout</td>          <td>15 seconds</td></tr>
        <tr><td>IP Address</td>       <td id="l-ip">--</td></tr>
      </table>
    </div>
  </div>
</div>

<!-- MANUAL -->
<div class="page" id="page4">
  <div class="section-h">Manual Control</div>
  <div class="grid g2">
    <div class="card">
      <div class="card-title">🕹 Extinguisher Control</div>
      <p style="font-size:13px;color:var(--muted);margin-bottom:18px;line-height:1.6;">
        Manually control the servo. In manual mode, automatic flame detection is overridden.
        System must be <b style="color:var(--text)">UNLOCKED</b> to operate.
      </p>
      <div class="lock-warn" id="ma-lockWarn">
        <span>⚠️</span>
        <span>System is <b>LOCKED</b> via RFID. Scan your card to unlock.</span>
      </div>
      <div style="display:flex;gap:12px;flex-wrap:wrap;margin-bottom:20px;">
        <button class="btn btn-red"   id="btnServoOn"  onclick="sendCmd('SERVO_ON')">🔴 Activate (70°)</button>
        <button class="btn btn-green" id="btnServoOff" onclick="sendCmd('SERVO_OFF')">🟢 Deactivate (0°)</button>
        <button class="btn btn-blue"  onclick="sendCmd('AUTO_MODE')">🔄 Return to Auto</button>
      </div>
      <div id="cmdStatus" class="cmd-status" style="opacity:0;"></div>
      <div style="background:var(--card2);border:1px solid var(--border);border-radius:10px;padding:16px;margin-top:10px;">
        <div class="card-title">Current Servo Status</div>
        <div style="font-size:30px;font-weight:800;margin-bottom:8px;" id="ma-servoDeg">90°</div>
        <span id="ma-servoLabel" class="badge b-ok">🟢 STANDBY 0°</span>
        <div style="margin-top:10px;font-size:13px;color:var(--muted2);" id="ma-modeText">Mode: AUTO</div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">📋 Command Log</div>
      <div id="ma-log" style="max-height:380px;overflow-y:auto;font-size:13px;">
        <div style="color:var(--muted);padding:8px 0;">No commands sent yet</div>
      </div>
    </div>
  </div>
</div>

<!-- SECURITY -->
<div class="page" id="page5">
  <div class="section-h">Security & RFID</div>
  <div class="grid g3" style="margin-bottom:14px;">
    <div class="card">
      <div class="card-title">🔒 Lock Status</div>
      <div style="text-align:center;margin:18px 0;">
        <div style="font-size:56px;transition:all .4s;" id="s-lockIcon">🔓</div>
        <div style="margin-top:10px;"><span id="s-lockBadge" class="badge b-ok">UNLOCKED</span></div>
      </div>
      <p style="font-size:12px;color:var(--muted);text-align:center;">Scan RFID card or key knob to toggle lock</p>
    </div>
    <div class="card">
      <div class="card-title">💳 Registered Credentials</div>
      <div style="margin-top:10px;display:flex;flex-direction:column;gap:10px;">
        <div style="background:var(--card2);border-radius:9px;padding:14px;">
          <div style="font-size:11px;color:var(--muted);margin-bottom:5px;">Key Card UID</div>
          <div style="font-family:monospace;font-size:15px;color:#93c5fd;letter-spacing:2px;">83 4E C6 95</div>
        </div>
        <div style="background:var(--card2);border-radius:9px;padding:14px;">
          <div style="font-size:11px;color:var(--muted);margin-bottom:5px;">Key Knob UID</div>
          <div style="font-family:monospace;font-size:15px;color:#93c5fd;letter-spacing:2px;">63 A1 BE 34</div>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">📊 Security Stats</div>
      <table class="info-table">
        <tr><td>Total RFID Events</td><td id="s-rfidCount">0</td></tr>
        <tr><td>Lock Events</td>      <td id="s-lockCount">0</td></tr>
        <tr><td>Unlock Events</td>    <td id="s-unlockCount">0</td></tr>
        <tr><td>Last RFID Event</td>  <td id="s-lastRFID">None</td></tr>
        <tr><td>Flame Events</td>     <td id="s-flameCount">0</td></tr>
        <tr><td>Gas Events</td>       <td id="s-gasCount">0</td></tr>
        <tr><td>Temp Events</td>      <td id="s-tempCount">0</td></tr>
      </table>
    </div>
  </div>
  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:14px;">
      <div class="card-title" style="margin:0;">📋 Full Event Log</div>
      <button class="btn btn-gray" onclick="clearLogUI()" style="padding:7px 14px;font-size:12px;">🗑 Clear View</button>
    </div>
    <div style="overflow-x:auto;">
      <table class="log-table">
        <thead><tr><th>Time</th><th>Type</th><th>Event</th></tr></thead>
        <tbody id="s-logBody">
          <tr><td colspan="3" style="color:var(--muted);text-align:center;padding:24px;">No events yet</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<script>
const WS_URL = 'ws://' + location.hostname + '/ws';
let ws, data = {}, logData = [];
const MAX_PTS = 20;
const tempBuf = [], humBuf = [], gasBuf = [], rssiBuf = [];

// ── WEBSOCKET ──
function connectWS() {
  ws = new WebSocket(WS_URL);
  ws.onopen  = () => { updateConnStatus(true); };
  ws.onclose = () => { updateConnStatus(false); setTimeout(connectWS, 3000); };
  ws.onerror = () => { ws.close(); };
  ws.onmessage = e => {
    try {
      const d = JSON.parse(e.data);
      if (d.cmdEcho !== undefined) { showCmdStatus('✅ Sent: ' + d.cmdEcho); return; }
      data = d;
      pushChartData();
      updateAll();
    } catch(ex) { console.warn('WS parse error', ex); }
  };
}

function updateConnStatus(ok) {
  el('statusDot').className    = 'dot' + (ok ? ' online' : '');
  el('statusText').textContent = ok ? 'System Online' : 'Reconnecting...';
}

// ── TABS ──
const pages = document.querySelectorAll('.page');
const tabs  = document.querySelectorAll('.tab');
function switchTab(n) {
  pages.forEach((p,i) => p.classList.toggle('active', i===n));
  tabs.forEach( (t,i) => t.classList.toggle('active', i===n));
}

// ── CLOCK ──
setInterval(() => {
  el('navTime').textContent = new Date().toLocaleTimeString('en-IN',{hour12:false});
}, 1000);

// ── MAIN UPDATE ──
function updateAll() {
  updateHome();
  updateMonitoring();
  updateExtinguisher();
  updateLora();
  updateManual();
  updateSecurity();
  drawLineChart('tempChart',     tempBuf, '#ef4444',   0,  80);
  drawLineChart('humChart',      humBuf,  '#3b82f6',   0, 100);
  drawLineChart('gasChart',      gasBuf,  '#f59e0b',   0, 4095);
  drawLineChart('rssiChart',     rssiBuf, '#8b5cf6', -120, -40);
  drawCombinedChart();

  // Alarm banner
  const alarm = !!data.flame || !!data.gasAlert || !!data.tempAlert;
  el('alarmBanner').classList.toggle('show', alarm);
  if (data.flame)      el('alarmText').textContent = '🔥 FIRE DETECTED! Extinguisher at 0°';
  else if(data.gasAlert) el('alarmText').textContent = '💨 GAS ALERT! High gas concentration';
  else if(data.tempAlert) el('alarmText').textContent = '🌡 TEMP ALERT! High temperature';
}

// ── HOME ──
function updateHome() {
  const t = data.temp ?? 0, h = data.hum ?? 0, g = data.gas ?? 0;
  const online = !!data.connected;
  const extOn  = !!data.flame || (!!data.manual && !!data.mServo);
  const locked = !!data.locked;

  el('h-temp').textContent = t.toFixed(1);
  el('h-hum').textContent  = h.toFixed(1);
  el('h-gas').textContent  = g;
  el('h-tempBar').style.width = Math.min(100, (t/80)*100) + '%';
  el('h-humBar').style.width  = Math.min(100, h) + '%';
  el('h-gasBar').style.width  = Math.min(100, (g/4095)*100) + '%';
  el('h-tempSub').textContent = t > 40 ? '⚠️ HIGH' : 'Normal';
  el('h-gasSub').textContent  = g > 2500 ? '⚠️ HIGH' : 'Normal';

  // Flame
  const fi = el('h-flameIcon'), fc = el('h-flameCard'), fb = el('h-flameBadge');
  if (data.flame) {
    fi.textContent = '🔥'; fi.className = 'flame-ic on';
    fb.className = 'badge b-alert'; fb.textContent = '⚠️ FLAME';
    fc.className = 'card alert-card flame';
  } else {
    fi.textContent = '🔵'; fi.className = 'flame-ic';
    fb.className = 'badge b-ok'; fb.textContent = 'NO FLAME';
    fc.className = 'card alert-card';
  }

  // Status badges
  const es = el('h-extStatus');
  if (extOn) { es.className='badge b-alert'; es.textContent='🔴 ACTIVE 70°'; }
  else       { es.className='badge b-ok';    es.textContent='🟢 STANDBY 0°'; }

  const ms = el('h-modeStatus');
  if (data.manual) { ms.className='badge b-warn'; ms.textContent='⚙ MANUAL'; }
  else             { ms.className='badge b-info'; ms.textContent='🔄 AUTO'; }

  const ls = el('h-lockStatus');
  if (locked) { ls.className='badge b-lock'; ls.textContent='🔒 LOCKED'; }
  else        { ls.className='badge b-ok';   ls.textContent='🔓 UNLOCKED'; }

  const lor = el('h-loraStatus');
  if (online) { lor.className='badge b-ok';    lor.textContent='✅ ONLINE'; }
  else        { lor.className='badge b-alert'; lor.textContent='❌ OFFLINE'; }

  el('h-rssi').textContent   = data.rssi ?? '--';
  el('h-snr').textContent    = data.snr  ?? '--';
  el('h-pkts').textContent   = data.pktCount ?? 0;
  el('h-uptime').textContent = fmtUptime(data.uptime ?? 0);
  el('h-lastUpdate').textContent = data.time ?? '--';

  // Active alerts list
  const alerts = [];
  if (data.flame)      alerts.push('<span class="badge b-alert">🔥 FLAME</span>');
  if (data.gasAlert)   alerts.push('<span class="badge b-warn">💨 GAS</span>');
  if (data.tempAlert)  alerts.push('<span class="badge b-warn">🌡 TEMP</span>');
  if (locked)          alerts.push('<span class="badge b-lock">🔒 LOCKED</span>');
  el('h-alerts').innerHTML = alerts.length
    ? '<div style="display:flex;flex-wrap:wrap;gap:8px;">' + alerts.join('') + '</div>'
    : '<div style="color:var(--muted);font-size:13px;">✅ All Clear</div>';
}

// ── MONITORING ──
function updateMonitoring() {
  const t = data.temp ?? 0, h = data.hum ?? 0, g = data.gas ?? 0;
  el('m-temp').textContent = t.toFixed(1);
  el('m-hum').textContent  = h.toFixed(1);
  el('m-gas').textContent  = g;
  el('m-tempBar').style.width = Math.min(100,(t/80)*100) + '%';
  el('m-humBar').style.width  = Math.min(100, h) + '%';
  el('m-gasBar').style.width  = Math.min(100,(g/4095)*100) + '%';

  const tb = el('m-tempBadge');
  if (data.tempAlert) { tb.className='badge b-alert'; tb.textContent='⚠️ HIGH'; }
  else               { tb.className='badge b-ok';    tb.textContent='NORMAL'; }

  const gb = el('m-gasBadge');
  if (data.gasAlert) { gb.className='badge b-warn'; gb.textContent='⚠️ HIGH'; }
  else              { gb.className='badge b-ok';   gb.textContent='NORMAL'; }

  const mfi = el('m-flameIcon'), mft = el('m-flameText');
  if (data.flame) {
    mfi.textContent = '🔥'; mfi.className = 'flame-ic on';
    mft.textContent = 'FLAME DETECTED'; mft.style.color = '#ef4444';
  } else {
    mfi.textContent = '🔵'; mfi.className = 'flame-ic';
    mft.textContent = 'NO FLAME'; mft.style.color = 'var(--green)';
  }
}

// ── EXTINGUISHER ──
function updateExtinguisher() {
  const extOn = !!data.flame || (!!data.manual && !!data.mServo);
  const locked = !!data.locked;

  // Flame card
  const efi = el('e-flameIcon'), efb = el('e-flameBadge'), efc = el('e-flameCard');
  if (data.flame) {
    efi.textContent='🔥'; efi.className='flame-ic on';
    efb.className='badge b-alert'; efb.textContent='⚠️ FLAME';
    efc.className='card alert-card flame';
  } else {
    efi.textContent='🔵'; efi.className='flame-ic';
    efb.className='badge b-ok'; efb.textContent='NO FLAME';
    efc.className='card alert-card';
  }

  // Servo visual needle
  // extOn → 90° servo → needle at RIGHT (+90deg CSS)
  // standby → 0° servo → needle at LEFT (-90deg CSS)
  const deg = extOn ? 70 : 0;
  const cssDeg = extOn ? 0 : -90;
  el('e-servoNeedle').style.transform = `translateX(-50%) rotate(${cssDeg}deg)`;
  el('e-servoDeg').textContent = deg + '°';

  const el2 = el('e-servoLabel');
  if (extOn) { el2.className='badge b-alert'; el2.textContent='🔴 ACTIVE 70°'; }
  else       { el2.className='badge b-ok';    el2.textContent='🟢 STANDBY 0°'; }

  // Info table
  el('e-status').textContent    = extOn ? '🔴 ACTIVE (70°)'  : '🟢 STANDBY (0°)';
  el('e-mode').textContent      = data.manual ? '⚙ MANUAL' : '🔄 AUTO';
  el('e-lock').textContent      = locked ? '🔒 LOCKED' : '🔓 OPEN';
  el('e-tempAlert').textContent = data.tempAlert ? '⚠️ HIGH' : 'Normal';
  el('e-gasAlert').textContent  = data.gasAlert  ? '⚠️ HIGH' : 'Normal';
  el('e-temp').textContent      = (data.temp ?? '--') + ' °C';
  el('e-gas').textContent       = data.gas ?? '--';
  el('e-hum').textContent       = (data.hum  ?? '--') + ' %';
}

// ── LORA ──
function updateLora() {
  const online = !!data.connected;
  el('l-rssi').textContent    = data.rssi ?? '--';
  el('l-snr').textContent     = data.snr  ?? '--';
  el('l-pkts').textContent    = data.pktCount ?? 0;
  el('l-rssiVal').textContent = data.rssi ?? '--';
  el('l-ip').textContent      = data.ip ?? '--';

  el('l-linkIcon').textContent   = online ? '✅' : '❌';
  el('l-linkBadge').className    = 'badge ' + (online ? 'b-ok' : 'b-alert');
  el('l-linkBadge').textContent  = online ? 'ONLINE' : 'OFFLINE';

  if (data.rssi) {
    const pct = Math.max(0, Math.min(100, ((data.rssi + 120) / 80) * 100));
    el('l-rssiBarFill').style.width = pct + '%';
    let q = 'Excellent', c = '#10b981';
    if (data.rssi < -100)      { q = 'Weak'; c = '#ef4444'; }
    else if (data.rssi < -85)  { q = 'Fair'; c = '#f59e0b'; }
    else if (data.rssi < -70)  { q = 'Good'; c = '#3b82f6'; }
    el('l-qualityLabel').textContent = q;
    el('l-qualityLabel').style.color = c;
  }
  drawLineChart('rssiChart', rssiBuf, '#8b5cf6', -120, -40);
}

// ── MANUAL CONTROL ──
function updateManual() {
  const extOn  = !!data.mServo && !!data.manual;
  const locked = !!data.locked;

  // Servo status display
  // SERVO_ON (Extinguisher button) → 0°  (extOn=true)
  // SERVO_OFF (Off button)         → 90° (extOn=false)
  el('ma-servoDeg').textContent = extOn ? '70°' : '0°';

  const sl = el('ma-servoLabel');
  if (extOn) { sl.className='badge b-alert'; sl.textContent='🔴 ACTIVE 70°'; }
  else       { sl.className='badge b-ok';    sl.textContent='🟢 STANDBY 0°'; }

  el('ma-modeText').textContent = 'Mode: ' + (data.manual ? '⚙ MANUAL' : '🔄 AUTO');
  el('ma-modeText').style.color = data.manual ? '#fbbf24' : '#94a3b8';

  // Lock warning and button state
  if (locked) el('ma-lockWarn').classList.add('show');
  else        el('ma-lockWarn').classList.remove('show');
  el('btnServoOn').disabled  = locked;
  el('btnServoOff').disabled = locked;
}

// ── SECURITY ──
function updateSecurity() {
  const locked = !!data.locked;
  el('s-lockIcon').textContent  = locked ? '🔒' : '🔓';
  el('s-lockBadge').className   = 'badge ' + (locked ? 'b-lock' : 'b-ok');
  el('s-lockBadge').textContent = locked ? '🔒 LOCKED' : '🔓 UNLOCKED';
}

// ── LOG ──
function fetchLog() {
  fetch('/api/log').then(r => r.json()).then(logs => {
    logData = logs;
    renderLog();
    updateLogStats();
  }).catch(() => {});
}

function renderLog() {
  const tbody = el('s-logBody');
  if (!logData.length) {
    tbody.innerHTML = '<tr><td colspan="3" style="color:var(--muted);text-align:center;padding:24px;">No events yet</td></tr>';
    return;
  }
  const colors = {FLAME:'#ef4444',GAS:'#f59e0b',TEMP:'#f97316',RFID:'#8b5cf6',INFO:'#3b82f6',CMD:'#06b6d4'};
  tbody.innerHTML = logData.slice(0, 60).map(l => {
    const c = colors[l.k] || '#64748b';
    return `<tr>
      <td style="font-family:monospace;color:var(--muted2);white-space:nowrap;">${l.t}</td>
      <td><span class="badge" style="background:${c}18;color:${c};border:1px solid ${c}33;">${l.k}</span></td>
      <td>${l.e}</td>
    </tr>`;
  }).join('');
}

function updateLogStats() {
  const rfid    = logData.filter(l => l.k === 'RFID');
  const locks   = rfid.filter(l => l.e.includes('LOCKED') && !l.e.includes('UN'));
  const unlocks = rfid.filter(l => l.e.includes('UNLOCKED'));
  const flames  = logData.filter(l => l.k === 'FLAME' && l.e.includes('DETECTED'));
  const gases   = logData.filter(l => l.k === 'GAS'   && l.e.includes('alert'));
  const temps   = logData.filter(l => l.k === 'TEMP'  && l.e.includes('High'));
  el('s-rfidCount').textContent   = rfid.length;
  el('s-lockCount').textContent   = locks.length;
  el('s-unlockCount').textContent = unlocks.length;
  el('s-lastRFID').textContent    = rfid.length ? rfid[0].t : 'None';
  el('s-flameCount').textContent  = flames.length;
  el('s-gasCount').textContent    = gases.length;
  el('s-tempCount').textContent   = temps.length;
}

function clearLogUI() {
  el('s-logBody').innerHTML = '<tr><td colspan="3" style="color:var(--muted);text-align:center;padding:24px;">Log cleared (local view only)</td></tr>';
}

setInterval(fetchLog, 5000);
fetchLog();

// ── CHART DATA ──
function pushChartData() {
  const push = (arr, v) => { arr.push(v); if (arr.length > MAX_PTS) arr.shift(); };
  push(tempBuf, data.temp ?? 0);
  push(humBuf,  data.hum  ?? 0);
  push(gasBuf,  data.gas  ?? 0);
  push(rssiBuf, data.rssi ?? -120);
}

// ── MINI LINE CHART ──
function drawLineChart(id, buf, color, yMin, yMax) {
  const canvas = document.getElementById(id);
  if (!canvas || buf.length < 2) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.offsetWidth || 200, H = canvas.height;
  canvas.width = W;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#060b14'; ctx.fillRect(0, 0, W, H);
  // Grid
  ctx.strokeStyle = '#1e2d42'; ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = (H / 4) * i;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
  }
  const xStep = W / (MAX_PTS - 1);
  const off   = (MAX_PTS - buf.length) * xStep;
  const yS    = v => H - Math.max(0, Math.min(H, ((v - yMin) / (yMax - yMin)) * H));
  // Gradient fill
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, color + '44'); grad.addColorStop(1, color + '00');
  ctx.fillStyle = grad;
  ctx.beginPath(); ctx.moveTo(off, H);
  buf.forEach((v, i) => { const x = off + i*xStep; i===0 ? ctx.lineTo(x,yS(v)) : ctx.lineTo(x,yS(v)); });
  ctx.lineTo(off + (buf.length-1)*xStep, H); ctx.closePath(); ctx.fill();
  // Line
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.lineJoin = 'round';
  ctx.beginPath();
  buf.forEach((v, i) => { const x = off + i*xStep; i===0 ? ctx.moveTo(x,yS(v)) : ctx.lineTo(x,yS(v)); });
  ctx.stroke();
  // Dot
  const lx = off + (buf.length-1)*xStep, ly = yS(buf[buf.length-1]);
  ctx.fillStyle = color; ctx.beginPath(); ctx.arc(lx, ly, 4, 0, Math.PI*2); ctx.fill();
  ctx.fillStyle = '#fff';  ctx.beginPath(); ctx.arc(lx, ly, 1.5, 0, Math.PI*2); ctx.fill();
}

function drawCombinedChart() {
  const canvas = document.getElementById('combinedChart');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.offsetWidth || 300, H = canvas.height;
  canvas.width = W;
  ctx.fillStyle = '#060b14'; ctx.fillRect(0, 0, W, H);
  ctx.strokeStyle = '#1e2d42'; ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = (H/4)*i;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
  }
  const draw = (buf, color, yMin, yMax) => {
    if (buf.length < 2) return;
    const xS = W / (MAX_PTS-1), off = (MAX_PTS-buf.length)*xS;
    const yS = v => H - Math.max(0, Math.min(H, ((v-yMin)/(yMax-yMin))*H));
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.lineJoin = 'round';
    ctx.beginPath();
    buf.forEach((v, i) => { const x = off+i*xS; i===0 ? ctx.moveTo(x,yS(v)) : ctx.lineTo(x,yS(v)); });
    ctx.stroke();
  };
  draw(tempBuf,'#ef4444',0,80);
  draw(humBuf, '#3b82f6',0,100);
  draw(gasBuf, '#f59e0b',0,4095);
}

// ── SEND COMMAND ──
function sendCmd(cmd) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send('CMD:' + cmd);
    showCmdStatus('📤 Sending: ' + cmd);
    addManualLog(cmd);
  } else {
    fetch('/api/cmd', {
      method: 'POST',
      headers: {'Content-Type':'application/x-www-form-urlencoded'},
      body: 'cmd=' + encodeURIComponent(cmd)
    }).then(r => r.json()).then(d => {
      if (d.ok) { showCmdStatus('✅ Sent: ' + cmd); addManualLog(cmd); }
      else       showCmdStatus('❌ Failed: ' + (d.error || 'unknown'));
    }).catch(() => showCmdStatus('❌ Connection error'));
  }
}

function addManualLog(cmd) {
  const labels = {
    SERVO_ON:  '🔴 Extinguisher ACTIVATED (servo → 70°)',
    SERVO_OFF: '🟢 Extinguisher DEACTIVATED (servo → 0°)',
    AUTO_MODE: '🔄 Returned to AUTO mode'
  };
  const div = el('ma-log');
  const wasEmpty = div.innerHTML.includes('No commands');
  const time = new Date().toLocaleTimeString('en-IN', {hour12:false});
  const entry = `<div style="display:flex;gap:12px;padding:9px 0;border-bottom:1px solid var(--border);font-size:13px;">
    <span style="color:var(--muted);font-family:monospace;white-space:nowrap;">${time}</span>
    <span>${labels[cmd] || cmd}</span>
  </div>`;
  div.innerHTML = entry + (wasEmpty ? '' : div.innerHTML);
}

function showCmdStatus(msg) {
  const s = el('cmdStatus');
  s.textContent = msg;
  s.style.opacity = '1';
  setTimeout(() => { s.style.opacity = '0'; }, 3000);
}

// ── UTILS ──
function el(id) { return document.getElementById(id); }

function fmtUptime(s) {
  if (s < 60)   return s + 's';
  if (s < 3600) return Math.floor(s/60) + 'm ' + (s%60) + 's';
  return Math.floor(s/3600) + 'h ' + Math.floor((s%3600)/60) + 'm';
}

// ── INIT ──
connectWS();
</script>
</body>
</html>
)rawhtml";
}
