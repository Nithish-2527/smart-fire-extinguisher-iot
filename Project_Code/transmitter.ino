/*
 * ============================================================
 *  SMART FIRE EXTINGUISHER - TRANSMITTER (ESP32 NodeMCU-32S)
 * ============================================================
 *
 * PIN CONNECTIONS:
 * ─────────────────────────────────────────────────────────
 * LoRa Ra-02 (HSPI bus):
 *   SCK     → GPIO 14   MISO  → GPIO 12
 *   MOSI    → GPIO 13   CS    → GPIO 15
 *   RST     → GPIO 4    DIO0  → GPIO 2
 *
 * DHT22:
 *   DATA    → GPIO 27
 *
 * IR Flame Sensor (active LOW when flame detected):
 *   DO      → GPIO 26
 *
 * MQ135 Gas Sensor:
 *   AO      → GPIO 34   DO    → GPIO 35
 *
 * Buzzer:
 *   Signal  → GPIO 25
 *
 * Servo (MG90S or MG996R):
 *   Signal  → GPIO 33
 *   SERVO 0°  = EXTINGUISHER ON
 *   SERVO 90° = EXTINGUISHER OFF
 *
 * RFID RC522 (VSPI bus):
 *   SCK     → GPIO 18   MISO  → GPIO 19
 *   MOSI    → GPIO 23   SDA   → GPIO 5
 *   RST     → GPIO 17
 *
 * OLED 0.96" I2C:
 *   SDA     → GPIO 21   SCL   → GPIO 22
 *
 * LIBRARIES:
 *   LoRa by Sandeep Mistry
 *   DHT sensor library by Adafruit
 *   Adafruit Unified Sensor
 *   MFRC522 by GithubCommunity
 *   Adafruit SSD1306 + GFX
 *   ESP32Servo
 * ─────────────────────────────────────────────────────────
 */

#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <MFRC522.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ══════════════════════════════════════════
//  PIN DEFINITIONS
// ══════════════════════════════════════════
// LoRa (HSPI)
#define LORA_SCK      14
#define LORA_MISO     12
#define LORA_MOSI     13
#define LORA_CS       15
#define LORA_RST       4
#define LORA_DIO0      2

// DHT22
#define DHT_PIN       27
#define DHT_TYPE      DHT22

// Sensors
#define FLAME_PIN     26   // LOW = flame detected
#define GAS_AO_PIN    34   // Analog 0–4095
#define GAS_DO_PIN    35   // Digital alert

// Outputs
#define BUZZER_PIN    25
#define SERVO_PIN     33

// RFID (VSPI)
#define RFID_SCK      18
#define RFID_MISO     19
#define RFID_MOSI     23
#define RFID_SS        5
#define RFID_RST      17

// OLED
#define OLED_SDA      21
#define OLED_SCL      22
#define SCREEN_W     128
#define SCREEN_H      64
#define OLED_ADDR   0x3C

// ══════════════════════════════════════════
//  THRESHOLDS
// ══════════════════════════════════════════
#define TEMP_HIGH     40.0f
#define GAS_HIGH      2500

// ══════════════════════════════════════════
//  RFID UIDs — update to match YOUR cards
//  Scan a card with a UID reader or check
//  Serial output after first scan attempt.
// ══════════════════════════════════════════
const byte CARD_UID[4] = {0x83, 0x4E, 0xC6, 0x95};
const byte KNOB_UID[4] = {0x63, 0xA1, 0xBE, 0x34};

// ══════════════════════════════════════════
//  SPI INSTANCES
//  Global SPI (VSPI pins) → RFID RC522   (old MFRC522 library uses default SPI)
//  hspi (HSPI pins)       → LoRa Ra-02   (separate bus, no conflict)
// ══════════════════════════════════════════
SPIClass hspi(HSPI);

// ══════════════════════════════════════════
//  OBJECTS
// ══════════════════════════════════════════
DHT              dht(DHT_PIN, DHT_TYPE);
// Old MFRC522 library: no custom SPI constructor.
// We temporarily route default SPI to VSPI pins for init, then switch to HSPI for LoRa.
MFRC522          rfid(RFID_SS, RFID_RST);
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
Servo            extServo;

// ══════════════════════════════════════════
//  STATE
// ══════════════════════════════════════════
bool  sysLocked      = false;
bool  flameOn        = false;
bool  gasAlert       = false;
bool  tempAlert      = false;
bool  manualMode     = false;
bool  manualServoOn  = false;  // true = servo 0° (ON), false = servo 90° (OFF)

float temp = 0.0f, hum = 0.0f;
int   gasRaw = 0;
bool  gasDig = false;

// Buzzer non-blocking state machine
enum BuzzerMode { BZ_IDLE, BZ_BEEP_SEQ, BZ_CONTINUOUS };
BuzzerMode    bzMode       = BZ_IDLE;
int           bzBeepTotal  = 0;
int           bzBeepDone   = 0;
bool          bzToneState  = false;
unsigned long bzLastToggle = 0;
#define BZ_ON_MS   200
#define BZ_OFF_MS  200

// Timers
unsigned long tSensor = 0, tLoRa = 0, tOled = 0, tPage = 0;
#define T_SENSOR  2000
#define T_LORA    3000
#define T_OLED    1000
#define T_PAGE    3500

int oledPage = 0;

// ══════════════════════════════════════════
//  FORWARD DECLARATIONS
// ══════════════════════════════════════════
void readSensors();
void evaluateAlerts();
void startBeeps(int n);
void stopBuzzer();
void runBuzzer(unsigned long now);
void setServo(int deg);
void setServoAuto();
void checkRFID();
bool matchUID(byte *uid, const byte *ref);
void beepRFID();
void beepDeny();
void txLoRa();
void rxLoRa();
void updateOLED(unsigned long now);
void showBootScreen();
void showReady();
void showError(const String &msg);
void showOLEDLine(const String &msg);
void showRFIDFeedback(bool locked);

// ══════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== Smart Fire Extinguisher - TRANSMITTER ==="));

  // I2C → OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[ERR] OLED not found"));
  }
  showBootScreen();

  // DHT22
  dht.begin();

  // GPIO
  pinMode(FLAME_PIN,  INPUT);
  pinMode(GAS_DO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Servo — default to OFF (0°)
  extServo.attach(SERVO_PIN, 500, 2400);
  setServo(0);
  delay(300);

  // ────────────────────────────────────────────────────
  //  RFID: Old MFRC522 library uses the global SPI object.
  //  We begin() the global SPI on VSPI pins so RFID works,
  //  then LoRa uses its own hspi instance (separate bus).
  // ────────────────────────────────────────────────────
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  pinMode(RFID_SS,  OUTPUT);
  digitalWrite(RFID_SS, HIGH);
  delay(50);

  rfid.PCD_Init();
  delay(100);

  byte ver = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (ver == 0x00 || ver == 0xFF) {
    Serial.println(F("[ERR] RFID NOT FOUND — check wiring/power"));
    showOLEDLine("RFID: NOT FOUND");
  } else {
    Serial.printf("[OK]  RFID firmware v%02X\n", ver);
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    showOLEDLine("RFID: OK");
  }

  // ────────────────────────────────────────────────────
  //  LoRa: Init HSPI AFTER RFID so buses don't conflict
  // ────────────────────────────────────────────────────
  hspi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(hspi);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println(F("[ERR] LoRa FAILED"));
    showError("LoRa FAIL");
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(17);
  Serial.println(F("[OK]  LoRa 433 MHz ready"));

  Serial.println(F("[OK]  All systems ready"));
  showReady();
}

// ══════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  if (now - tSensor >= T_SENSOR) {
    tSensor = now;
    readSensors();
    evaluateAlerts();
  }

  checkRFID();
  runBuzzer(now);

  if (now - tLoRa >= T_LORA) {
    tLoRa = now;
    txLoRa();
  }

  rxLoRa();

  if (now - tOled >= T_OLED) {
    tOled = now;
    updateOLED(now);
  }

  setServoAuto();
}

// ══════════════════════════════════════════
//  SENSORS
// ══════════════════════════════════════════
void readSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temp = t;
  if (!isnan(h)) hum  = h;

  flameOn = (digitalRead(FLAME_PIN) == LOW);
  gasRaw  = analogRead(GAS_AO_PIN);
  gasDig  = (digitalRead(GAS_DO_PIN) == HIGH);

  Serial.printf("[SEN] T=%.1f H=%.1f G=%d F=%d LOCK=%d MANUAL=%d MServo=%d\n",
    temp, hum, gasRaw, flameOn, sysLocked, manualMode, manualServoOn);
}

// ══════════════════════════════════════════
//  ALERT LOGIC
// ══════════════════════════════════════════
void evaluateAlerts() {
  if (sysLocked) {
    stopBuzzer();
    return;
  }

  // Flame → continuous buzzer
  if (flameOn) {
    if (bzMode != BZ_CONTINUOUS) {
      bzMode       = BZ_CONTINUOUS;
      bzToneState  = true;
      bzLastToggle = millis();
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println(F("[!] FLAME → CONTINUOUS BUZZER"));
    }
    return;
  } else if (bzMode == BZ_CONTINUOUS) {
    stopBuzzer();
    Serial.println(F("[i] Flame cleared — buzzer stopped"));
  }

  bool newTemp = (temp > TEMP_HIGH);
  bool newGas  = (gasRaw > GAS_HIGH || gasDig);

  if (newTemp && !tempAlert) {
    tempAlert = true;
    if (bzMode == BZ_IDLE) startBeeps(3);
    Serial.println(F("[!] TEMP ALERT → 3 beeps"));
  }
  if (!newTemp) tempAlert = false;

  if (newGas && !gasAlert) {
    gasAlert = true;
    if (bzMode == BZ_IDLE) startBeeps(2);
    Serial.println(F("[!] GAS ALERT → 2 beeps"));
  }
  if (!newGas) gasAlert = false;
}

void startBeeps(int n) {
  bzBeepTotal  = n;
  bzBeepDone   = 0;
  bzMode       = BZ_BEEP_SEQ;
  bzToneState  = true;
  bzLastToggle = millis();
  digitalWrite(BUZZER_PIN, HIGH);
}

void stopBuzzer() {
  bzMode      = BZ_IDLE;
  bzToneState = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void runBuzzer(unsigned long now) {
  if (bzMode == BZ_IDLE || bzMode == BZ_CONTINUOUS) return;

  unsigned long elapsed = now - bzLastToggle;
  unsigned long target  = bzToneState ? (unsigned long)BZ_ON_MS : (unsigned long)BZ_OFF_MS;
  if (elapsed < target) return;

  bzLastToggle = now;
  bzToneState  = !bzToneState;
  digitalWrite(BUZZER_PIN, bzToneState ? HIGH : LOW);

  if (!bzToneState) {
    bzBeepDone++;
    if (bzBeepDone >= bzBeepTotal) {
      bzMode = BZ_IDLE;
      Serial.printf("[BZ] Done %d beeps\n", bzBeepDone);
    }
  }
}

// ══════════════════════════════════════════
//  SERVO
//  0°  = Extinguisher ON  (press Extinguisher)
//  90° = Extinguisher OFF (press Off)
// ══════════════════════════════════════════
void setServo(int deg) {
  extServo.write(deg);
}

void setServoAuto() {
  if (sysLocked) {
    setServo(0);    // locked → OFF (0°)
    return;
  }
  if (manualMode) {
    // manualServoOn=true  → SERVO_ON cmd  → 70° (Extinguisher ON)
    // manualServoOn=false → SERVO_OFF cmd → 0°  (Extinguisher OFF)
    setServo(manualServoOn ? 70 : 0);
    return;
  }
  // Auto mode: flame → 70° (ON), no flame → 0° (OFF)
  setServo(flameOn ? 70 : 0);
}

// ══════════════════════════════════════════
//  RFID
// ══════════════════════════════════════════
void checkRFID() {
  // Reset the VSPI bus before each RFID check to avoid SPI conflicts
  // after LoRa transactions on HSPI
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  byte *uid    = rfid.uid.uidByte;
  bool  isAuth = matchUID(uid, CARD_UID) || matchUID(uid, KNOB_UID);

  Serial.printf("[RFID] UID: %02X:%02X:%02X:%02X → %s\n",
    uid[0], uid[1], uid[2], uid[3], isAuth ? "AUTH" : "DENY");

  if (isAuth) {
    sysLocked = !sysLocked;
    if (sysLocked) {
      manualMode    = false;
      manualServoOn = false;
    }
    beepRFID();
    showRFIDFeedback(sysLocked);
    Serial.printf("[RFID] System %s\n", sysLocked ? "LOCKED" : "UNLOCKED");
  } else {
    beepDeny();
    Serial.println(F("[RFID] Access DENIED"));
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

bool matchUID(byte *uid, const byte *ref) {
  for (int i = 0; i < 4; i++) {
    if (uid[i] != ref[i]) return false;
  }
  return true;
}

void beepRFID() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(80);
    digitalWrite(BUZZER_PIN, LOW);  delay(80);
  }
}

void beepDeny() {
  digitalWrite(BUZZER_PIN, HIGH); delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}

// ══════════════════════════════════════════
//  LORA TX — send sensor data to receiver
// ══════════════════════════════════════════
void txLoRa() {
  String pkt = "T:"  + String(temp, 1)
    + ",H:"  + String(hum, 1)
    + ",G:"  + String(gasRaw)
    + ",F:"  + (flameOn       ? "1" : "0")
    + ",L:"  + (sysLocked     ? "1" : "0")
    + ",MA:" + (manualMode    ? "1" : "0")
    + ",MS:" + (manualServoOn ? "1" : "0")
    + ",TA:" + (tempAlert     ? "1" : "0")
    + ",GA:" + (gasAlert      ? "1" : "0");

  LoRa.beginPacket();
  LoRa.print(pkt);
  LoRa.endPacket();
  Serial.println("[TX] " + pkt);
}

// ══════════════════════════════════════════
//  LORA RX — receive commands from receiver
//  SERVO_ON  → manual mode, servo 0°  (Extinguisher ON)
//  SERVO_OFF → manual mode, servo 90° (Extinguisher OFF)
//  AUTO_MODE → return to automatic mode
// ══════════════════════════════════════════
void rxLoRa() {
  int sz = LoRa.parsePacket();
  if (!sz) return;

  String cmd = "";
  while (LoRa.available()) cmd += (char)LoRa.read();
  cmd.trim();
  Serial.println("[RX] " + cmd);

  if (cmd == "SERVO_ON") {
    manualMode    = true;
    manualServoOn = true;
    Serial.println(F("[CMD] Manual → Extinguisher ON (0°)"));
  } else if (cmd == "SERVO_OFF") {
    manualMode    = true;
    manualServoOn = false;
    Serial.println(F("[CMD] Manual → Extinguisher OFF (90°)"));
  } else if (cmd == "AUTO_MODE") {
    manualMode    = false;
    manualServoOn = false;
    Serial.println(F("[CMD] Auto mode restored"));
  } else if (cmd == "PING") {
    // just respond with next scheduled TX
  } else {
    Serial.println("[RX] Unknown: " + cmd);
  }
}

// ══════════════════════════════════════════
//  OLED HELPERS
// ══════════════════════════════════════════
static int bootY = 28;

void showOLEDLine(const String &msg) {
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, bootY);
  oled.print(msg);
  oled.display();
  bootY += 10;
}

void showBootScreen() {
  bootY = 28;
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 12, WHITE);
  oled.setTextColor(BLACK); oled.setTextSize(1);
  oled.setCursor(4, 2);  oled.print(F("SMART FIRE SYSTEM"));
  oled.setTextColor(WHITE);
  oled.setCursor(0, 16); oled.println(F("  Initializing..."));
  oled.setCursor(0, 28); oled.println(F("  LoRa   : Setup"));
  oled.setCursor(0, 38); oled.println(F("  RFID   : Setup"));
  oled.setCursor(0, 48); oled.println(F("  Sensors: Setup"));
  oled.display();
  delay(1500);
}

void showReady() {
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 12, WHITE);
  oled.setTextColor(BLACK); oled.setTextSize(1);
  oled.setCursor(18, 2); oled.print(F("SYSTEM READY"));
  oled.setTextColor(WHITE);
  oled.setTextSize(2);
  oled.setCursor(20, 25); oled.print(F("ONLINE"));
  oled.setTextSize(1);
  oled.setCursor(10, 52); oled.print(F("Monitoring Active"));
  oled.display();
  delay(1500);
}

void showError(const String &msg) {
  oled.clearDisplay();
  oled.setTextColor(WHITE); oled.setTextSize(2);
  oled.setCursor(0, 10); oled.println(F("!! ERROR"));
  oled.setTextSize(1);
  oled.setCursor(0, 38); oled.println(msg);
  oled.display();
}

void showRFIDFeedback(bool locked) {
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 12, WHITE);
  oled.setTextColor(BLACK); oled.setTextSize(1);
  oled.setCursor(25, 2); oled.print(F("RFID ACCESS"));
  oled.setTextColor(WHITE);

  if (locked) {
    oled.fillRect(20, 18, 88, 30, WHITE);
    oled.setTextColor(BLACK); oled.setTextSize(2);
    oled.setCursor(25, 23); oled.print(F("LOCKED"));
    oled.setTextColor(WHITE); oled.setTextSize(1);
    oled.setCursor(10, 53); oled.print(F("All outputs stopped"));
  } else {
    oled.setTextSize(2);
    oled.setCursor(5, 23); oled.print(F("UNLOCKED"));
    oled.setTextSize(1);
    oled.setCursor(20, 53); oled.print(F("System Active"));
  }
  oled.display();
  delay(1800);
}

// ══════════════════════════════════════════
//  OLED LIVE PAGES
// ══════════════════════════════════════════
void updateOLED(unsigned long now) {
  if (now - tPage > T_PAGE) {
    tPage   = now;
    oledPage = (oledPage + 1) % 3;
  }

  oled.clearDisplay();

  // Header bar
  oled.fillRect(0, 0, 128, 11, WHITE);
  oled.setTextColor(BLACK); oled.setTextSize(1);
  switch (oledPage) {
    case 0: oled.setCursor(22, 2); oled.print(F("SENSORS"));       break;
    case 1: oled.setCursor(14, 2); oled.print(F("SYSTEM STATUS")); break;
    case 2: oled.setCursor(28, 2); oled.print(F("ALERTS"));        break;
  }
  oled.setTextColor(WHITE);

  // Lock warning strip
  int yBase = 13;
  if (sysLocked) {
    oled.fillRect(0, 12, 128, 9, WHITE);
    oled.setTextColor(BLACK);
    oled.setCursor(22, 13); oled.print(F("[ SYSTEM LOCKED ]"));
    oled.setTextColor(WHITE);
    yBase = 23;
  }

  bool extOn = flameOn || (manualMode && manualServoOn);

  switch (oledPage) {
    case 0:
      oled.setCursor(0, yBase);
      oled.printf("Temp  : %.1f C\n", temp);
      oled.printf("Humid : %.1f %%\n", hum);
      oled.printf("Gas   : %d\n", gasRaw);
      oled.printf("Flame : %s", flameOn ? "*** YES ***" : "None");
      break;

    case 1:
      oled.setCursor(0, yBase);
      oled.printf("Servo : %s\n", extOn ? "0deg  ON" : "90deg OFF");
      oled.printf("Mode  : %s\n", manualMode ? "MANUAL" : "AUTO");
      oled.printf("Buzzer: %s\n", (bzMode != BZ_IDLE) ? "ACTIVE" : "Quiet");
      oled.printf("Lock  : %s", sysLocked ? "LOCKED" : "OPEN");
      break;

    case 2:
      oled.setCursor(0, yBase);
      oled.printf("[%s] FLAME\n", flameOn   ? "!!" : "--");
      oled.printf("[%s] GAS\n",   gasAlert  ? "!!" : "--");
      oled.printf("[%s] TEMP\n",  tempAlert ? "!!" : "--");
      oled.printf("[%s] LOCK",    sysLocked ? "!!" : "--");
      break;
  }

  oled.display();
}
