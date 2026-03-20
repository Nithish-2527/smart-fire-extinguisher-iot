// ============================================================
//  SMART FIRE EXTINGUISHER SYSTEM — TRANSMITTER  (FIXED)
//  Hardware: ESP32 + LoRa RA-02 (HSPI) + DHT22 + MQ135
//            + IR Flame + RFID RC522 (VSPI) + Servo + OLED
//
//  FIXES APPLIED:
//   1. [CRITICAL] SPI init order — VSPI (RFID) must be
//      initialized BEFORE HSPI (LoRa). Original code called
//      SPI.begin() AFTER LoRa.setSPI(hspi), which corrupted
//      the VSPI bus and broke RFID entirely.
//   2. Debounce delay moved inside checkRFID() — the 500ms
//      delay was blocking the main loop, starving all sensors.
//   3. Servo + buzzer now explicitly forced OFF at the top of
//      the LOCKED branch before any early return.
//   4. Buzzer guaranteed LOW when no flame, regardless of lock
//      state — previously stayed HIGH if flame cleared while locked.
//   5. Loop delay increased to 1000ms for stable LoRa pacing;
//      DHT22 needs ≥500ms between reads anyway.
//   6. NaN temperature guarded before LoRa payload — sends
//      -99.9 as a sentinel so the receiver can detect sensor error.
// ============================================================

#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
//  PIN DEFINITIONS  (DO NOT CHANGE)
// ============================================================

// LoRa RA-02 — HSPI
#define LORA_SCK    14
#define LORA_MISO   12
#define LORA_MOSI   13
#define LORA_CS     15
#define LORA_RST     4
#define LORA_DIO0    2

// DHT22
#define DHT_PIN     27
#define DHT_TYPE    DHT22

// IR Flame Sensor (Active LOW → flame = LOW)
#define FLAME_PIN   26

// MQ135
#define MQ135_AO    34   // Analog output
#define MQ135_DO    35   // Digital output (LOW = gas detected)

// Buzzer
#define BUZZER_PIN  25

// Servo (90° = Extinguisher ON, 0° = Extinguisher OFF)
#define SERVO_PIN   33
#define SERVO_ON    90
#define SERVO_OFF    0

// RFID RC522 — VSPI (default SPI bus on ESP32)
#define RFID_SCK    18
#define RFID_MISO   19
#define RFID_MOSI   23
#define RFID_SDA     5
#define RFID_RST    17

// OLED — I2C
#define OLED_SDA    21
#define OLED_SCL    22
#define SCREEN_W   128
#define SCREEN_H    64

// ============================================================
//  THRESHOLDS
// ============================================================
#define TEMP_THRESHOLD   40.0   // °C — above this = abnormal

// ============================================================
//  OBJECTS
// ============================================================
SPIClass          hspi(HSPI);
DHT               dht(DHT_PIN, DHT_TYPE);
MFRC522           rfid(RFID_SDA, RFID_RST);
Servo             extServo;
Adafruit_SSD1306  display(SCREEN_W, SCREEN_H, &Wire, -1);

// ============================================================
//  STATE FLAGS
// ============================================================
bool rfidLocked  = false;   // true = LOCKED
bool gasBuzzed   = false;   // one-shot gas alert flag
bool tempBuzzed  = false;   // one-shot temp alert flag

// ============================================================
//  HELPERS — BUZZER
// ============================================================
void beepOnce() {
  digitalWrite(BUZZER_PIN, HIGH); delay(250);
  digitalWrite(BUZZER_PIN, LOW);  delay(150);
}

void beepTwice() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(250);
    digitalWrite(BUZZER_PIN, LOW);  delay(150);
  }
}

// ============================================================
//  HELPERS — OLED
// ============================================================
void oledStartup(const char* line1, const char* line2 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2) display.println(line2);
  display.display();
  delay(1500);
}

void updateOLED(float temp, int gas, int flame,
                bool locked, bool extOn) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Row 0 — Temperature
  display.print(F("Temp : "));
  if (!isnan(temp)) {
    display.print(temp, 1);
    display.println(F(" C"));
  } else {
    display.println(F("ERROR"));
  }

  // Row 1 — Gas (raw ADC value)
  display.print(F("Gas  : ")); display.println(gas);

  // Row 2 — Flame
  display.print(F("Flame: ")); display.println(flame);

  // Row 3 — RFID Status
  display.print(F("RFID : "));
  display.println(locked ? F("LOCKED") : F("UNLOCKED"));

  // Row 4 — Extinguisher Status
  display.print(F("EXT  : "));
  display.println(extOn ? F("ON") : F("OFF"));

  // Row 5 — LoRa indicator
  display.println(F("[ LORA TX ]"));

  display.display();
}

// ============================================================
//  HELPERS — RFID
//  FIX 2: Debounce delay is now LOCAL to this function.
//          It no longer blocks the entire main loop.
// ============================================================
void checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  rfidLocked = !rfidLocked;          // Toggle LOCK / UNLOCK

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // FIX 2: delay is fine here — it only blocks on a real card tap,
  // not on every loop iteration as before.
  delay(500);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // --- Pin modes ---
  pinMode(FLAME_PIN,  INPUT);
  pinMode(MQ135_DO,   INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // --- OLED (I2C — must come before SPI inits) ---
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init failed"));
    while (true);
  }
  oledStartup("SYSTEM ON");

  // --- DHT22 ---
  dht.begin();

  // --- Servo ---
  extServo.attach(SERVO_PIN);
  extServo.write(SERVO_OFF);

  // ============================================================
  //  FIX 1 — CORRECT SPI BUS INITIALIZATION ORDER
  //
  //  VSPI (RFID RC522) MUST be initialized FIRST.
  //  Original code called SPI.begin() AFTER LoRa.setSPI(hspi),
  //  which caused SPI.begin() to re-init the bus with wrong
  //  settings, silently breaking RFID communication.
  //
  //  Rule: initialize VSPI → rfid.PCD_Init() → then HSPI → LoRa
  // ============================================================

  // --- STEP 1: RFID RC522 on VSPI (default SPI bus) ---
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  rfid.PCD_Init();
  delay(50);  // Give RC522 time to stabilize after init

  // Optional: verify RFID is alive before continuing
  byte rfidVersion = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (rfidVersion == 0x00 || rfidVersion == 0xFF) {
    oledStartup("RFID FAILED!", "Check wiring");
    Serial.println(F("RFID init failed — check wiring on pins 5,17,18,19,23"));
    // Do NOT hang — continue so LoRa/sensors still work
  } else {
    Serial.print(F("RFID OK. Firmware v"));
    Serial.println(rfidVersion, HEX);
    oledStartup("RFID OK");
  }

  // --- STEP 2: LoRa RA-02 on HSPI (secondary SPI bus) ---
  hspi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(hspi);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    oledStartup("LORA FAILED!", "Check wiring");
    Serial.println(F("LoRa init failed"));
    while (true);
  }

  oledStartup("LORA ACTIVATED");
  Serial.println(F("System ready."));
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {

  // ── 1. Read Sensors ────────────────────────────────────────
  float temp    = dht.readTemperature();
  int   gasRaw  = analogRead(MQ135_AO);
  bool  gasHigh = (digitalRead(MQ135_DO) == LOW);  // LOW = gas abnormal
  bool  flame   = (digitalRead(FLAME_PIN) == LOW);  // Active LOW
  bool  tempHigh = (!isnan(temp) && temp > TEMP_THRESHOLD);

  // ── 2. RFID Check ──────────────────────────────────────────
  checkRFID();

  // ── 3. Determine ext / buzzer behaviour ────────────────────
  bool extOn = false;

  if (rfidLocked) {
    // ── LOCKED: force servo OFF and buzzer OFF immediately ──
    // FIX 3: These writes are guaranteed to execute in LOCKED
    // state, preventing servo/buzzer from staying active.
    extServo.write(SERVO_OFF);
    digitalWrite(BUZZER_PIN, LOW);   // FIX 4: ensure buzzer is LOW
    extOn      = false;
    gasBuzzed  = false;
    tempBuzzed = false;

  } else {
    // ── UNLOCKED: normal sensor-driven operation ────────────

    // Servo — activate extinguisher only on flame detection
    if (flame) {
      extServo.write(SERVO_ON);
      extOn = true;
    } else {
      extServo.write(SERVO_OFF);
      extOn = false;
    }

    // Buzzer — priority: flame > gas > temp
    if (flame) {
      // Continuous HIGH while flame persists
      digitalWrite(BUZZER_PIN, HIGH);
      // Reset one-shot flags so alerts re-trigger after clear
      gasBuzzed  = false;
      tempBuzzed = false;

    } else {
      // FIX 4: Explicitly ensure buzzer is LOW when no flame.
      // This handles the case where flame clears while locked
      // (buzzer was left HIGH) and system then unlocks.
      digitalWrite(BUZZER_PIN, LOW);

      // Gas alert — beep ONCE (one-shot per event)
      if (gasHigh && !gasBuzzed) {
        beepOnce();
        gasBuzzed = true;
      }
      if (!gasHigh) gasBuzzed = false;   // Reset on clearance

      // Temp alert — beep TWICE (one-shot per event)
      if (tempHigh && !tempBuzzed) {
        beepTwice();
        tempBuzzed = true;
      }
      if (!tempHigh) tempBuzzed = false; // Reset on clearance
    }
  }

  // ── 4. Update OLED ─────────────────────────────────────────
  updateOLED(temp, gasRaw, flame ? 1 : 0, rfidLocked, extOn);

  // ── 5. Build & Send LoRa Packet ────────────────────────────
  // Format: temp,gas,flame,rfid_status,ext_status
  // rfid_status : 1 = UNLOCKED, 0 = LOCKED
  // ext_status  : 1 = ON, 0 = OFF

  // FIX 6: Replace NaN with sentinel -99.9 so receiver knows
  // the DHT22 read failed instead of getting garbage/empty data.
  float txTemp = isnan(temp) ? -99.9 : temp;

  int rfidStatus = rfidLocked ? 0 : 1;
  int extStatus  = extOn      ? 1 : 0;

  String payload = String(txTemp, 1)     + "," +
                   String(gasRaw)         + "," +
                   String(flame ? 1 : 0)  + "," +
                   String(rfidStatus)     + "," +
                   String(extStatus);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  Serial.print(F("TX: "));
  Serial.println(payload);

  // FIX 5: 1000ms delay — DHT22 requires ≥500ms between reads
  // for accurate measurements. 500ms was too tight and caused
  // sporadic NaN readings under load.
  delay(1000);
}