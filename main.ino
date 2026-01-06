/*
  Solar-assisted e-ink firmware (single-file)
  - WeAct 2.9" B/W/R (GxEPD2_3C)
  - BLE text uploader for notes & commands
  - BH1750 ambient -> front-light PWM
  - Rotary encoder navigation + push (wake)
  - Battery ADC reading (voltage divider)
  - Multiple pages: Dashboard, Notes, System
  - Persistent notes in NVS (Preferences)
  - Deep sleep after INACTIVITY_MIN minutes (encoder wakes), BLE off by default
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <time.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <BH1750.h>

#include <GxEPD2_3C.h>          // 3-color driver
#include <Adafruit_GFX.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// ========== CONFIG ==========
// Pins (change only if required)
#define EPD_CS   5
#define EPD_DC   17
#define EPD_RST  16
#define EPD_BUSY 4
#define EPD_SCK  18
#define EPD_MOSI 23

#define ENC_A_PIN     14
#define ENC_B_PIN     27
#define ENC_BTN_PIN   33  // RTC-capable, used for ext0 wake

#define I2C_SDA       21
#define I2C_SCL       22

#define LED_PWM_PIN   25  // MOSFET gate (PWM)
#define LEDC_CH       0
#define LEDC_FREQ     5000
#define LEDC_RES      8   // 0..255

#define BAT_ADC_PIN   35  // ADC reading (via divider)
const float DIVIDER_FACTOR = 3.0f; // set to actual divider ratio (Vbat = ADC*DIVIDER_FACTOR)
const float BAT_EMPTY_V = 3.0f;
const float BAT_FULL_V  = 4.2f;

// Deep sleep inactivity
const uint32_t INACTIVITY_MIN = 30; // minutes -> 30 minutes

// BLE service & char UUIDs
#define SERVICE_UUID   "e1230001-1234-5678-1234-56789abcdef0"
#define CHAR_TEXT_UUID "e1230002-1234-5678-1234-56789abcdef0"

// Display (GxEPD2_3C with 2.9" template)
GxEPD2_3C<GxEPD2_290c, GxEPD2_290c::HEIGHT> display(GxEPD2_290c(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// BH1750
BH1750 lightMeter;

// Persistent storage
Preferences prefs;
const char *PREF_NS = "einkapp";
const int MAX_NOTES = 16; // store up to 16 notes
const char *NOTES_COUNT_KEY = "notes_cnt";

// BLE state
BLEServer* bleServer = nullptr;
BLECharacteristic* textChar = nullptr;
bool bleEnabled = false;
unsigned long bleEnableTs = 0;
const unsigned long BLE_ENABLE_TIMEOUT_MS = 2*60*1000UL; // auto-disable BLE if idle ~2 min

// UI state
int currentPage = 0;
const int NUM_PAGES = 4; // 0:Dashboard,1:Notes list,2:Note view,3:System
int selectedNoteIndex = 0;

// Encoder
volatile long encoderPos = 0;
volatile bool encoderMoved = false;
long lastEncoderPos = 0;
unsigned long lastActivityMs = 0;

// Front-light auto mapping
const float LUX_THRESHOLD_ON = 40.0f; // below this -> light on
const int LIGHT_PWM_ON = 200;
const int LIGHT_PWM_MIN = 0;

// WiFi / time
bool wifiConfigured = false;

// Forward declarations
void drawDashboard();
void drawNotesList();
void drawNote(int idx);
void drawSystem();
void saveNote(int idx, const String &text);
String readNote(int idx);
int loadNotesCount();
void setNotesCount(int n);
void addNote(const String &text);
void removeNote(int idx);

// ========== HELPERS ==========
// Simple battery % mapping linear
int batteryPercentFromVoltage(float v) {
  if (v <= BAT_EMPTY_V) return 0;
  if (v >= BAT_FULL_V) return 100;
  return (int)round((v - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0);
}

float readBatteryVoltage() {
  uint16_t raw = analogRead(BAT_ADC_PIN); // ADC1
  float v = (raw / 4095.0f) * 3.3f * DIVIDER_FACTOR;
  return v;
}

// Time (NTP) - optional. We store WiFi creds in prefs "ssid","pass" if provided earlier by user
bool trySyncTimeNTP() {
  if (!prefs.begin(PREF_NS, true)) return false;
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  prefs.end();
  if (ssid.length() == 0) return false;
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long start = millis();
  while (millis() - start < 10000) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    return false;
  }
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 5000)) {
    WiFi.disconnect();
    return false;
  }
  // Leave WiFi connected briefly; user code can disconnect later
  WiFi.disconnect();
  return true;
}

// ========== DISPLAY DRAW FUNCTIONS ==========
void drawCenteredText(const String &t, int y) {
  display.setCursor(10, y);
  display.println(t);
}

void drawDashboard() {
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    // Time (simple)
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char buf[32];
      strftime(buf, sizeof(buf), "%H:%M", &timeinfo);
      display.setCursor(10, 30);
      display.setTextSize(3);
      display.print(buf);
      display.setTextSize(1);
      display.setCursor(160, 30);
      char d[32];
      strftime(d, sizeof(d), "%a %d %b", &timeinfo);
      display.print(d);
    } else {
      display.setCursor(10, 30);
      display.setTextSize(2);
      display.print("No Time");
    }

    // Battery
    float vb = readBatteryVoltage();
    int pct = batteryPercentFromVoltage(vb);
    display.setTextSize(1);
    display.setCursor(10, 80);
    display.printf("Battery: %.2fV %d%%", vb, pct);

    // Next note (if exists)
    int n = loadNotesCount();
    display.setCursor(10, 100);
    if (n > 0) {
      String next = readNote(0);
      if (next.length() > 30) next = next.substring(0, 27) + "...";
      display.printf("Next: %s", next.c_str());
    } else {
      display.print("No notes");
    }

    // BLE state
    display.setCursor(10, 125);
    display.printf("BLE: %s", bleEnabled ? "ON" : "OFF");
  } while (display.nextPage());
  display.hibernate();
}

void drawNotesList() {
  int n = loadNotesCount();
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print("Notes:");
    for (int i = 0; i < min(n, 6); ++i) {
      int y = 30 + 14 * (i+1);
      String t = readNote(i);
      if (t.length() > 30) t = t.substring(0,27) + "...";
      if (i == selectedNoteIndex) {
        display.setTextColor(GxEPD_RED);
      } else {
        display.setTextColor(GxEPD_BLACK);
      }
      display.setCursor(10, y);
      display.print(i+1);
      display.print(": ");
      display.print(t);
    }
  } while (display.nextPage());
  display.hibernate();
}

void drawNote(int idx) {
  String t = readNote(idx);
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print("Note ");
    display.print(idx+1);
    display.setCursor(10, 40);
    // wrap naive
    int lineY = 40;
    int pos = 0;
    while (pos < (int)t.length()) {
      String chunk = t.substring(pos, min(pos+40, (int)t.length()));
      display.setCursor(10, lineY);
      display.print(chunk);
      lineY += 12;
      pos += 40;
    }
  } while (display.nextPage());
  display.hibernate();
}

void drawSystem() {
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(10, 20);
    display.print("System:");
    display.setCursor(10, 40);
    display.printf("Notes stored: %d", loadNotesCount());
    display.setCursor(10, 60);
    display.printf("BLE: %s", bleEnabled ? "ON" : "OFF");
    display.setCursor(10, 80);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char buf[64];
      strftime(buf, sizeof(buf), "%c", &timeinfo);
      display.print(buf);
    } else {
      display.print("Time: not set");
    }
  } while (display.nextPage());
  display.hibernate();
}

// ========== STORAGE (Preferences) ==========
void initStorage() {
  prefs.begin(PREF_NS, false); // RW
  if (!prefs.isKey(NOTES_COUNT_KEY)) {
    setNotesCount(0);
  }
}

int loadNotesCount() {
  return prefs.getInt(NOTES_COUNT_KEY, 0);
}

void setNotesCount(int n) {
  prefs.putInt(NOTES_COUNT_KEY, n);
}

String readNote(int idx) {
  char key[16];
  snprintf(key, sizeof(key), "note%d", idx);
  return prefs.getString(key, "");
}

void saveNote(int idx, const String &text) {
  char key[16];
  snprintf(key, sizeof(key), "note%d", idx);
  prefs.putString(key, text);
}

void addNote(const String &text) {
  int n = loadNotesCount();
  if (n >= MAX_NOTES) {
    // overwrite last
    saveNote(MAX_NOTES-1, text);
    setNotesCount(MAX_NOTES);
  } else {
    saveNote(n, text);
    setNotesCount(n+1);
  }
}

void removeNote(int idx) {
  int n = loadNotesCount();
  if (idx < 0 || idx >= n) return;
  for (int i = idx; i < n-1; ++i) {
    saveNote(i, readNote(i+1));
  }
  prefs.remove(String("note") + String(n-1));
  setNotesCount(n-1);
}

// ========== BLE CALLBACKS ==========
class TextCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) {
    std::string s = characteristic->getValue();
    if (s.length() == 0) return;
    String v = String(s.c_str());
    // Command parsing:
    // NOTE:<text>  -> append note
    // TIME:<epoch> -> set time
    // CLEAR_NOTES   -> clear
    // BLE:OFF       -> disable BLE
    if (v.startsWith("NOTE:")) {
      String body = v.substring(5);
      addNote(body);
      drawNotesList();
    } else if (v.startsWith("TIME:")) {
      // set time from epoch seconds
      String epochS = v.substring(5);
      unsigned long epoch = epochS.toInt();
      struct timeval tv = { (time_t)epoch, 0 };
      settimeofday(&tv, NULL);
      drawDashboard();
    } else if (v == "CLEAR_NOTES") {
      int n = loadNotesCount();
      for (int i = 0; i < n; ++i) {
        prefs.remove(String("note") + String(i));
      }
      setNotesCount(0);
      drawNotesList();
    } else if (v == "BLE:OFF") {
      // disable BLE
      if (bleEnabled) {
        BLEDevice::deinit(true);
        bleEnabled = false;
        drawDashboard();
      }
    } else {
      // default: add as note
      addNote(v);
      drawNotesList();
    }
    lastActivityMs = millis();
  }
};

// ========== BLE CONTROL ==========
// Start BLE server & characteristic (call when enabling BLE)
void bleStart() {
  if (bleEnabled) return;
  BLEDevice::init("EInk_Companion");
  bleServer = BLEDevice::createServer();
  BLEService *service = bleServer->createService(SERVICE_UUID);
  textChar = service->createCharacteristic(
    CHAR_TEXT_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  textChar->addDescriptor(new BLE2902());
  textChar->setCallbacks(new TextCallback());
  service->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
  bleEnabled = true;
  bleEnableTs = millis();
  drawDashboard();
}

// Stop BLE (call to disable)
void bleStop() {
  if (!bleEnabled) return;
  BLEDevice::deinit(true);
  bleEnabled = false;
  drawDashboard();
}

// ========== ENCODER (simple polling in loop for reliability) ==========
void IRAM_ATTR encoderISR() {
  // minimal handler: mark moved; read in loop
  encoderMoved = true;
}

void setupEncoderPins() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);
}

// Debounced button read
bool readButtonPressed() {
  static unsigned long lastPress = 0;
  if (digitalRead(ENC_BTN_PIN) == LOW) {
    if (millis() - lastPress > 50) {
      lastPress = millis();
      return true;
    }
  }
  return false;
}

// ========== FRONT-LIGHT (LEDC) ==========
void setupLight() {
  ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(LED_PWM_PIN, LEDC_CH);
  ledcWrite(LEDC_CH, 0);
}

void autoAdjustLight() {
  static unsigned long lastRun = 0;
  if (millis() - lastRun < 1000) return;
  lastRun = millis();
  float lux = 0;
  if (lightMeter.measurementReady()) {
    lux = lightMeter.readLightLevel();
  } else {
    lux = lightMeter.readLightLevel();
  }
  if (lux < LUX_THRESHOLD_ON) {
    ledcWrite(LEDC_CH, LIGHT_PWM_ON);
  } else {
    ledcWrite(LEDC_CH, LIGHT_PWM_MIN);
  }
}

// ========== POWER / SLEEP ==========
void goToDeepSleep() {
  // brief UI update so user sees we're sleeping
  ledcWrite(LEDC_CH, 0);
  delay(20);
  // enable ext0 wake on button (LOW)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)ENC_BTN_PIN, 0);
  // set timer wake as backup (e.g., 24 hours) - not necessary but safe
  //esp_sleep_enable_timer_wakeup(24ULL * 3600ULL * 1000000ULL);
  Serial.println("Entering deep sleep...");
  prefs.end();
  display.hibernate();
  esp_deep_sleep_start();
}

// ========== SETUP & LOOP ==========
void setup() {
  Serial.begin(115200);
  delay(100);
  // In case we woke from deep sleep, prefs needs begin later
  prefs.begin(PREF_NS, false);

  Wire.begin(I2C_SDA, I2C_SCL);
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  pinMode(BAT_ADC_PIN, INPUT);
  analogReadResolution(12);

  setupLight();
  setupEncoderPins();

  // Display init
  display.init();
  display.setRotation(1);

  initStorage();

  // Try NTP if WiFi creds present (optional)
  if (prefs.isKey("ssid")) {
    if (trySyncTimeNTP()) {
      Serial.println("Time synced via NTP");
    } else {
      Serial.println("NTP sync failed");
    }
  }

  // initial draw
  drawDashboard();

  lastActivityMs = millis();
}

void loop() {
  // encoder handling: poll on interrupt mark
  if (encoderMoved) {
    encoderMoved = false;
    // read quadrature to update encoderPos
    static int lastA = HIGH;
    static int lastB = HIGH;
    int a = digitalRead(ENC_A_PIN);
    int b = digitalRead(ENC_B_PIN);
    if (a != lastA || b != lastB) {
      if (a == b) encoderPos++;
      else encoderPos--;
      lastA = a; lastB = b;
      lastActivityMs = millis();
    }
  }

  // encoder position change => page change
  if (encoderPos != lastEncoderPos) {
    if (encoderPos > lastEncoderPos) {
      currentPage = (currentPage + 1) % NUM_PAGES;
    } else {
      currentPage = (currentPage - 1 + NUM_PAGES) % NUM_PAGES;
    }
    lastEncoderPos = encoderPos;
    // redraw page
    switch(currentPage) {
      case 0: drawDashboard(); break;
      case 1: drawNotesList(); break;
      case 2:
        drawNote(selectedNoteIndex);
        break;
      case 3:
        drawSystem(); break;
    }
  }

  // Button push -> short press / long press
  static unsigned long btnDownTs = 0;
  static bool btnDown = false;
  int btn = digitalRead(ENC_BTN_PIN);
  if (btn == LOW && !btnDown) {
    btnDown = true;
    btnDownTs = millis();
  } else if (btn == HIGH && btnDown) {
    unsigned long held = millis() - btnDownTs;
    btnDown = false;
    lastActivityMs = millis();
    if (held < 800) {
      // short press: select / view / toggle light
      if (currentPage == 1) {
        // open selected note
        drawNote(selectedNoteIndex);
        currentPage = 2;
      } else if (currentPage == 2) {
        // back to list
        currentPage = 1;
        drawNotesList();
      } else {
        // toggle manual light
        static bool manualLightOn = false;
        manualLightOn = !manualLightOn;
        if (manualLightOn) ledcWrite(LEDC_CH, LIGHT_PWM_ON);
        else ledcWrite(LEDC_CH, 0);
      }
    } else {
      // long press: enable BLE (advertise) for pairing/writing
      if (!bleEnabled) {
        bleStart();
        Serial.println("BLE enabled by long press");
        lastActivityMs = millis();
      } else {
        // disable BLE
        bleStop();
        Serial.println("BLE disabled");
      }
    }
  }

  // BLE auto-timeout if enabled
  if (bleEnabled && (millis() - bleEnableTs > BLE_ENABLE_TIMEOUT_MS)) {
    // auto-stop if no activity
    // Note: if you want BLE to remain until user manually disables, increase timeout or remove this.
    bleStop();
  }

  // Auto adjust light (use BH1750)
  autoAdjustLight();

  // Periodically update battery display while on dashboard or system
  static unsigned long lastBatMs = 0;
  if (millis() - lastBatMs > 15000) {
    lastBatMs = millis();
    if (currentPage == 0) drawDashboard();
    if (currentPage == 3) drawSystem();
  }

  // Sleep decision
  if (!bleEnabled) { // BLE prevents sleeping (we stay awake while BLE enabled)
    unsigned long idleMs = millis() - lastActivityMs;
    if (idleMs > INACTIVITY_MIN * 60UL * 1000UL) {
      // go to deep sleep (encoder button wakes)
      drawDashboard(); // final state
      delay(200);
      goToDeepSleep();
    }
  }

  delay(20);
}
