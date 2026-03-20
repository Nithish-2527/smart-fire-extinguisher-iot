// ============================================================
//  SMART FIRE EXTINGUISHER SYSTEM — RECEIVER
//  Hardware: ESP32 + LoRa RA-02 (VSPI) + OLED 0.96" I2C
//  NO WiFi / NO Web Dashboard
// ============================================================

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
//  PIN DEFINITIONS  (DO NOT CHANGE)
// ============================================================

// LoRa RA-02 — VSPI
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
#define LORA_CS      5
#define LORA_RST    14
#define LORA_DIO0    2

// OLED — I2C
#define OLED_SDA    21
#define OLED_SCL    22
#define SCREEN_W   128
#define SCREEN_H    64

// ============================================================
//  OBJECTS
// ============================================================
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

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
                int rfidStatus, int extStatus) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Row 0 — Temperature
  display.print(F("Temp : "));
  display.print(temp, 1);
  display.println(F(" C"));

  // Row 1 — Gas raw ADC
  display.print(F("Gas  : "));
  display.println(gas);

  // Row 2 — Flame
  display.print(F("Flame: "));
  display.println(flame);

  // Row 3 — RFID Status
  display.print(F("RFID : "));
  display.println(rfidStatus ? F("UNLOCKED") : F("LOCKED"));

  // Row 4 — Extinguisher Status
  display.print(F("EXT  : "));
  display.println(extStatus ? F("ON") : F("OFF"));

  // Row 5 — LoRa indicator
  display.println(F("[ LORA RX ]"));

  display.display();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // --- OLED ---
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init failed"));
    while (true);
  }
  oledStartup("RECEIVER ON");

  // --- LoRa RA-02 (VSPI) ---
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    oledStartup("LORA FAILED!", "Check wiring");
    Serial.println(F("LoRa init failed"));
    while (true);
  }

  oledStartup("LORA ACTIVATED");
  Serial.println(F("Receiver ready. Listening..."));
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {

  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;   // Nothing received — keep polling

  // ── 1. Read incoming packet ───────────────────────────────
  String received = "";
  while (LoRa.available()) {
    received += (char)LoRa.read();
  }
  received.trim();

  Serial.print(F("RX: "));
  Serial.println(received);

  // ── 2. Parse payload using sscanf ────────────────────────
  // Expected format: temp,gas,flame,rfid_status,ext_status
  // Example        : 30.5,1200,1,1,1
  float temp  = 0.0;
  int   gas   = 0;
  int   flame = 0;
  int   rfid  = 0;
  int   ext   = 0;

  int parsed = sscanf(received.c_str(),
                      "%f,%d,%d,%d,%d",
                      &temp, &gas, &flame, &rfid, &ext);

  if (parsed != 5) {
    // Malformed packet — show error but keep last valid display
    Serial.print(F("Parse error — fields parsed: "));
    Serial.println(parsed);
    return;
  }

  // ── 3. Clamp & validate values ───────────────────────────
  flame = constrain(flame, 0, 1);
  rfid  = constrain(rfid,  0, 1);
  ext   = constrain(ext,   0, 1);

  // ── 4. Update OLED display ───────────────────────────────
  updateOLED(temp, gas, flame, rfid, ext);
}
