/*
  - WeAct 2.9" B/W (GxEPD2_BW)
  - BLE text uploader for notes, WiFi setup, and commands
  - LittleFS-backed notes with file-based storage
  - BH1750 ambient -> front-light PWM with hysteresis
  - Rotary encoder navigation + push (wake)
  - Battery ADC reading (voltage divider)
  - Pages: Dashboard, Notes, Note Viewer, System, OTA
  - Optional NTP sync and ArduinoOTA over WiFi
  - Deep sleep after INACTIVITY_MIN minutes (encoder wakes), BLE off by default
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <time.h>

#include <FS.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <BH1750.h>

#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// ========== CONFIG ==========
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
const float DIVIDER_FACTOR = 3.0f;
const float BAT_EMPTY_V = 3.0f;
const float BAT_FULL_V  = 4.2f;

const uint32_t INACTIVITY_MIN = 30; // minutes

#define SERVICE_UUID   "e1230001-1234-5678-1234-56789abcdef0"
#define CHAR_TEXT_UUID "e1230002-1234-5678-1234-56789abcdef0"

static const char *DEVICE_NAME = "EInk_Companion";
static const int MAX_NOTES = 128;
static const int NOTES_PER_PAGE = 6;
static const int NOTE_CHARS_PER_LINE = 38;
static const int NOTE_VISIBLE_LINES = 6;
static const char *NOTES_DIR = "/notes";
static const char *NOTE_SEQ_KEY = "note_seq";
static const char *WIFI_SSID_KEY = "ssid";
static const char *WIFI_PASS_KEY = "pass";
static const char *LIGHT_MODE_KEY = "light_mode";

enum PageMode : uint8_t {
  PAGE_DASHBOARD = 0,
  PAGE_NOTES = 1,
  PAGE_NOTE_VIEW = 2,
  PAGE_SYSTEM = 3,
  PAGE_OTA = 4,
};

enum LightMode : uint8_t {
  LIGHT_MODE_AUTO = 0,
  LIGHT_MODE_FORCED_ON = 1,
  LIGHT_MODE_FORCED_OFF = 2,
};

// Display (GxEPD2_BW with 2.9" template)
GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// Sensors / storage
BH1750 lightMeter;
Preferences prefs;

// BLE state
BLEServer *bleServer = nullptr;
BLECharacteristic *textChar = nullptr;
bool bleEnabled = false;
unsigned long bleEnableTs = 0;
const unsigned long BLE_ENABLE_TIMEOUT_MS = 2 * 60 * 1000UL;

// WiFi / OTA state
bool wifiConnected = false;
bool otaActive = false;
unsigned long otaStartTs = 0;
const unsigned long OTA_ENABLE_TIMEOUT_MS = 20 * 60 * 1000UL;

// UI state
PageMode currentPage = PAGE_DASHBOARD;
int selectedNoteIndex = 0;
int noteViewIndex = 0;
int notesPageOffset = 0;
int noteScrollLine = 0;

// Encoder state
volatile int32_t encoderDelta = 0;
volatile uint8_t encoderLastState = 0;
int32_t encoderAccum = 0;

// Front-light state
const float LUX_THRESHOLD_ON = 40.0f;
const float LUX_THRESHOLD_OFF = 55.0f;
const int LIGHT_PWM_ON = 200;
const int LIGHT_PWM_MIN = 0;
LightMode lightMode = LIGHT_MODE_AUTO;
bool frontLightOn = false;
unsigned long lastLightSampleMs = 0;

// Notes index
String noteFiles[MAX_NOTES];
int noteCount = 0;
uint32_t nextNoteSeq = 1;
bool filesystemReady = false;

// Timing / refresh
unsigned long lastActivityMs = 0;
unsigned long lastDashboardRefreshMs = 0;
unsigned long lastSystemRefreshMs = 0;
unsigned long lastOtaRefreshMs = 0;

// ========== HELPERS ==========
int batteryPercentFromVoltage(float v) {
  if (v <= BAT_EMPTY_V) return 0;
  if (v >= BAT_FULL_V) return 100;
  return (int)round((v - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0f);
}

float readBatteryVoltage() {
  uint16_t raw = analogRead(BAT_ADC_PIN);
  return (raw / 4095.0f) * 3.3f * DIVIDER_FACTOR;
}

String normalizeNoteText(String text) {
  text.replace("\r", " ");
  text.replace("\n", " ");
  text.replace("\t", " ");
  text.trim();
  return text;
}

String notePathForSeq(uint32_t seq) {
  char path[48];
  snprintf(path, sizeof(path), "%s/note_%08lu.txt", NOTES_DIR, (unsigned long)seq);
  return String(path);
}

String readFileText(const String &path) {
  File file = LittleFS.open(path, FILE_READ);
  if (!file) return "";
  String text = file.readString();
  file.close();
  return text;
}

String notePreviewFromText(String text) {
  text = normalizeNoteText(text);
  if (text.length() <= 32) return text;
  return text.substring(0, 29) + "...";
}

String currentLightModeLabel() {
  switch (lightMode) {
    case LIGHT_MODE_FORCED_ON: return "ON";
    case LIGHT_MODE_FORCED_OFF: return "OFF";
    default: return "AUTO";
  }
}

String currentTimeLabel() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "No Time";
  char buf[32];
  strftime(buf, sizeof(buf), "%H:%M", &timeinfo);
  return String(buf);
}

String currentDateLabel() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "";
  char buf[32];
  strftime(buf, sizeof(buf), "%a %d %b", &timeinfo);
  return String(buf);
}

String uptimeLabel() {
  uint32_t seconds = millis() / 1000UL;
  uint32_t minutes = seconds / 60UL;
  uint32_t hours = minutes / 60UL;
  minutes %= 60UL;
  seconds %= 60UL;
  char buf[32];
  snprintf(buf, sizeof(buf), "%luh %lum %lus", (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
  return String(buf);
}

String bytesLabel(uint32_t bytes) {
  if (bytes >= 1024UL * 1024UL) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f MB", bytes / 1024.0f / 1024.0f);
    return String(buf);
  }
  if (bytes >= 1024UL) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f KB", bytes / 1024.0f);
    return String(buf);
  }
  return String(bytes) + " B";
}

void drawFrame(const String &title, const String &subtitle = "") {
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(1);
  display.setCursor(8, 12);
  display.print(title);
  if (subtitle.length() > 0) {
    display.setCursor(170, 12);
    display.print(subtitle);
  }
  display.drawFastHLine(0, 16, 296, GxEPD_BLACK);
}

void drawFooter(const String &text) {
  display.drawFastHLine(0, 116, 296, GxEPD_BLACK);
  display.setTextSize(1);
  display.setCursor(8, 126);
  display.print(text);
}

void ensureFilesystems() {
  if (!filesystemReady) {
    filesystemReady = LittleFS.begin(true);
  }
  if (!filesystemReady) return;

  if (!LittleFS.exists(NOTES_DIR)) {
    LittleFS.mkdir(NOTES_DIR);
  }

  nextNoteSeq = prefs.getUInt(NOTE_SEQ_KEY, 1);
  lightMode = (LightMode)prefs.getUChar(LIGHT_MODE_KEY, LIGHT_MODE_AUTO);
}

void refreshNotesIndex() {
  noteCount = 0;
  if (!filesystemReady) return;

  File root = LittleFS.open(NOTES_DIR);
  if (!root || !root.isDirectory()) {
    return;
  }

  for (File file = root.openNextFile(); file && noteCount < MAX_NOTES; file = root.openNextFile()) {
    if (file.isDirectory()) {
      continue;
    }
    String path = String(file.name());
    if (!path.startsWith("/")) {
      path = String(NOTES_DIR) + "/" + path;
    }
    noteFiles[noteCount++] = path;
  }
  root.close();

  for (int i = 0; i < noteCount - 1; ++i) {
    for (int j = i + 1; j < noteCount; ++j) {
      if (noteFiles[j].compareTo(noteFiles[i]) < 0) {
        String tmp = noteFiles[i];
        noteFiles[i] = noteFiles[j];
        noteFiles[j] = tmp;
      }
    }
  }

  if (noteCount == 0) {
    selectedNoteIndex = 0;
    noteViewIndex = 0;
    notesPageOffset = 0;
    noteScrollLine = 0;
    return;
  }

  if (selectedNoteIndex >= noteCount) selectedNoteIndex = noteCount - 1;
  if (selectedNoteIndex < 0) selectedNoteIndex = 0;
  if (noteViewIndex >= noteCount) noteViewIndex = noteCount - 1;
  if (noteViewIndex < 0) noteViewIndex = 0;
  if (notesPageOffset > selectedNoteIndex) notesPageOffset = (selectedNoteIndex / NOTES_PER_PAGE) * NOTES_PER_PAGE;
  if (notesPageOffset < 0) notesPageOffset = 0;
}

int loadNotesCount() {
  return noteCount;
}

String readNote(int idx) {
  if (idx < 0 || idx >= noteCount) return "";
  return readFileText(noteFiles[idx]);
}

bool deleteNote(int idx) {
  if (idx < 0 || idx >= noteCount) return false;
  bool removed = LittleFS.remove(noteFiles[idx]);
  refreshNotesIndex();
  return removed;
}

void clearAllNotes() {
  if (!filesystemReady) return;
  for (int i = 0; i < noteCount; ++i) {
    LittleFS.remove(noteFiles[i]);
  }
  refreshNotesIndex();
}

bool addNote(const String &text) {
  if (!filesystemReady) ensureFilesystems();
  if (!filesystemReady) return false;

  if (noteCount >= MAX_NOTES) {
    LittleFS.remove(noteFiles[0]);
    refreshNotesIndex();
  }

  String noteText = normalizeNoteText(text);
  if (noteText.length() == 0) return false;

  String path = notePathForSeq(nextNoteSeq++);
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file) return false;
  file.print(noteText);
  file.close();

  prefs.putUInt(NOTE_SEQ_KEY, nextNoteSeq);
  refreshNotesIndex();
  return true;
}

void storeWiFiCredentials(const String &ssid, const String &pass) {
  prefs.putString(WIFI_SSID_KEY, ssid);
  prefs.putString(WIFI_PASS_KEY, pass);
}

bool loadWiFiCredentials(String &ssid, String &pass) {
  ssid = prefs.getString(WIFI_SSID_KEY, "");
  pass = prefs.getString(WIFI_PASS_KEY, "");
  return ssid.length() > 0;
}

bool connectWiFiForServices(uint32_t timeoutMs = 15000UL) {
  String ssid;
  String pass;
  if (!loadWiFiCredentials(ssid, pass)) {
    return false;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return true;
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.begin(ssid.c_str(), pass.c_str());

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      return true;
    }
    delay(100);
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
  return false;
}

void disconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true, true);
  }
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
}

bool syncTimeFromNTP() {
  if (!connectWiFiForServices()) {
    return false;
  }

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  bool ok = getLocalTime(&timeinfo, 5000);
  if (!otaActive) {
    disconnectWiFi();
  }
  return ok;
}

void setLightMode(LightMode mode) {
  lightMode = mode;
  prefs.putUChar(LIGHT_MODE_KEY, (uint8_t)lightMode);
}

void setFrontLight(bool on) {
  if (frontLightOn == on) return;
  frontLightOn = on;
  ledcWrite(LEDC_CH, on ? LIGHT_PWM_ON : LIGHT_PWM_MIN);
}

void toggleLightMode() {
  switch (lightMode) {
    case LIGHT_MODE_AUTO:
      setLightMode(LIGHT_MODE_FORCED_ON);
      setFrontLight(true);
      break;
    case LIGHT_MODE_FORCED_ON:
      setLightMode(LIGHT_MODE_FORCED_OFF);
      setFrontLight(false);
      break;
    case LIGHT_MODE_FORCED_OFF:
    default:
      setLightMode(LIGHT_MODE_AUTO);
      break;
  }
}

void autoAdjustLight() {
  if (millis() - lastLightSampleMs < 1000) return;
  lastLightSampleMs = millis();

  switch (lightMode) {
    case LIGHT_MODE_FORCED_ON:
      setFrontLight(true);
      return;
    case LIGHT_MODE_FORCED_OFF:
      setFrontLight(false);
      return;
    case LIGHT_MODE_AUTO:
    default:
      break;
  }

  float lux = lightMeter.readLightLevel();
  if (lux < 0) lux = 0;
  if (!frontLightOn && lux < LUX_THRESHOLD_ON) {
    setFrontLight(true);
  } else if (frontLightOn && lux > LUX_THRESHOLD_OFF) {
    setFrontLight(false);
  }
}

String pageName(PageMode page) {
  switch (page) {
    case PAGE_DASHBOARD: return "Dashboard";
    case PAGE_NOTES: return "Notes";
    case PAGE_NOTE_VIEW: return "Note";
    case PAGE_SYSTEM: return "System";
    case PAGE_OTA: return "OTA";
    default: return "EInk";
  }
}

PageMode cycleMainPage(PageMode page, int step) {
  int index = 0;
  switch (page) {
    case PAGE_DASHBOARD: index = 0; break;
    case PAGE_NOTES: index = 1; break;
    case PAGE_SYSTEM: index = 2; break;
    case PAGE_OTA: index = 3; break;
    default: index = 0; break;
  }
  index = (index + step) % 4;
  if (index < 0) index += 4;

  switch (index) {
    case 0: return PAGE_DASHBOARD;
    case 1: return PAGE_NOTES;
    case 2: return PAGE_SYSTEM;
    default: return PAGE_OTA;
  }
}

void drawDashboard();
void drawNotesList();
void drawNote(int idx);
void drawSystem();
void drawOtaPage();
void drawCurrentPage();

// ========== DISPLAY DRAW FUNCTIONS ==========
void drawDashboard() {
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    drawFrame("Dashboard", bleEnabled ? "BLE on" : "BLE off");

    display.setTextSize(3);
    display.setCursor(8, 46);
    display.print(currentTimeLabel());

    display.setTextSize(1);
    display.setCursor(168, 28);
    String dateLabel = currentDateLabel();
    display.print(dateLabel.length() > 0 ? dateLabel : "No time");

    float batteryVoltage = readBatteryVoltage();
    int batteryPct = batteryPercentFromVoltage(batteryVoltage);
    display.setCursor(8, 76);
    display.printf("Battery: %.2fV (%d%%)", batteryVoltage, batteryPct);

    String lightLabel = currentLightModeLabel();
    display.setCursor(8, 90);
    display.printf("Notes: %d  Light: %s", loadNotesCount(), lightLabel.c_str());

    display.setCursor(8, 104);
    if (noteCount > 0) {
      String preview = notePreviewFromText(readNote(noteCount - 1));
      display.print("Latest: ");
      display.print(preview.length() > 0 ? preview : "(empty)");
    } else {
      display.print("No notes stored");
    }

    drawFooter("Short: light  Long: BLE  Encoder: pages");
  } while (display.nextPage());
  display.hibernate();
  lastDashboardRefreshMs = millis();
}

void drawNotesList() {
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    drawFrame("Notes", String(loadNotesCount()) + " stored");

    if (noteCount == 0) {
      display.setTextSize(1);
      display.setTextColor(GxEPD_BLACK);
      display.setCursor(8, 44);
      display.print("No notes yet.");
      display.setCursor(8, 58);
      display.print("Send NOTE:... over BLE or use OTA page.");
      drawFooter("Long: BLE  Short: page  Encoder: pages");
    } else {
      if (notesPageOffset > selectedNoteIndex) {
        notesPageOffset = (selectedNoteIndex / NOTES_PER_PAGE) * NOTES_PER_PAGE;
      }
      int endIndex = min(noteCount, notesPageOffset + NOTES_PER_PAGE);
      for (int i = notesPageOffset; i < endIndex; ++i) {
        int row = i - notesPageOffset;
        int y = 32 + row * 14;
        bool selected = (i == selectedNoteIndex);
        if (selected) {
          display.fillRect(6, y - 10, 284, 13, GxEPD_BLACK);
          display.setTextColor(GxEPD_WHITE);
        } else {
          display.setTextColor(GxEPD_BLACK);
        }
        display.setTextSize(1);
        display.setCursor(10, y);
        display.printf("%d. ", i + 1);
        String preview = notePreviewFromText(readNote(i));
        display.print(preview.length() > 0 ? preview : "(empty)");
      }
      int totalPages = (noteCount + NOTES_PER_PAGE - 1) / NOTES_PER_PAGE;
      int pageNumber = (selectedNoteIndex / NOTES_PER_PAGE) + 1;
      drawFooter("Short: open  Long: BLE  Page " + String(pageNumber) + "/" + String(totalPages));
    }
  } while (display.nextPage());
  display.hibernate();
}

void drawNote(int idx) {
  String text = readNote(idx);
  if (text.length() == 0) text = "(empty note)";

  int totalLines = max(1, (text.length() + NOTE_CHARS_PER_LINE - 1) / NOTE_CHARS_PER_LINE);
  if (noteScrollLine < 0) noteScrollLine = 0;
  if (noteScrollLine > max(0, totalLines - NOTE_VISIBLE_LINES)) {
    noteScrollLine = max(0, totalLines - NOTE_VISIBLE_LINES);
  }

  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    drawFrame(String("Note ") + String(idx + 1), String(noteScrollLine + 1) + "/" + String(totalLines));

    int lineY = 32;
    for (int line = 0; line < NOTE_VISIBLE_LINES; ++line) {
      int sourceLine = noteScrollLine + line;
      if (sourceLine >= totalLines) break;
      int start = sourceLine * NOTE_CHARS_PER_LINE;
      int end = min(start + NOTE_CHARS_PER_LINE, (int)text.length());
      String chunk = text.substring(start, end);
      display.setTextSize(1);
      display.setTextColor(GxEPD_BLACK);
      display.setCursor(8, lineY);
      display.print(chunk);
      lineY += 12;
    }

    drawFooter("Short: back  Long: delete  Encoder: scroll");
  } while (display.nextPage());
  display.hibernate();
}

void drawSystem() {
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    drawFrame("System", uptimeLabel());

    display.setTextSize(1);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(8, 30);
    display.printf("Notes: %d", loadNotesCount());

    display.setCursor(8, 42);
    display.printf("BLE: %s  OTA: %s", bleEnabled ? "on" : "off", otaActive ? "on" : "off");

    display.setCursor(8, 54);
    display.printf("WiFi: %s  Light: %s", wifiConnected ? "on" : "off", currentLightModeLabel().c_str());

    display.setCursor(8, 66);
    display.printf("Heap: %u", (unsigned)ESP.getFreeHeap());

    display.setCursor(8, 78);
    display.printf("FS: %s", filesystemReady ? "ready" : "off");

    if (filesystemReady) {
      uint32_t totalBytes = LittleFS.totalBytes();
      uint32_t usedBytes = LittleFS.usedBytes();
      display.setCursor(8, 90);
      display.printf("Storage: %s / %s", bytesLabel(usedBytes).c_str(), bytesLabel(totalBytes).c_str());
    }

    drawFooter("Short: OTA  Long: BLE  Encoder: pages");
  } while (display.nextPage());
  display.hibernate();
  lastSystemRefreshMs = millis();
}

void drawOtaPage() {
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    drawFrame("OTA", otaActive ? "ready" : "idle");

    display.setTextSize(1);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(8, 30);
    if (otaActive && wifiConnected && WiFi.status() == WL_CONNECTED) {
      display.print("IP: ");
      display.print(WiFi.localIP().toString());
    } else {
      display.print("WiFi credentials are required for OTA.");
    }

    display.setCursor(8, 42);
    display.print("Hostname: ");
    display.print(DEVICE_NAME);

    display.setCursor(8, 54);
    display.print("Short press: start/stop OTA");

    display.setCursor(8, 66);
    display.print("Long press: BLE toggle");

    display.setCursor(8, 78);
    display.print(otaActive ? "ArduinoOTA is live" : "Use BLE to store WIFI:ssid|pass");

    display.setCursor(8, 90);
    display.print("Keep the device awake during uploads.");

    drawFooter("Encoder: pages  Short: OTA  Long: BLE");
  } while (display.nextPage());
  display.hibernate();
  lastOtaRefreshMs = millis();
}

void drawCurrentPage() {
  switch (currentPage) {
    case PAGE_DASHBOARD: drawDashboard(); break;
    case PAGE_NOTES: drawNotesList(); break;
    case PAGE_NOTE_VIEW: drawNote(noteViewIndex); break;
    case PAGE_SYSTEM: drawSystem(); break;
    case PAGE_OTA: drawOtaPage(); break;
  }
}

// ========== OTA CONTROL ==========
void setupOtaHandlers() {
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.onStart([]() {
    setFrontLight(false);
    display.hibernate();
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA complete");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress * 100U) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
}

void stopOtaSession() {
  otaActive = false;
  disconnectWiFi();
  drawCurrentPage();
}

bool startOtaSession() {
  if (otaActive) {
    return true;
  }

  if (bleEnabled) {
    bleStop();
  }

  if (!connectWiFiForServices()) {
    drawOtaPage();
    return false;
  }

  setupOtaHandlers();
  ArduinoOTA.begin();
  otaActive = true;
  otaStartTs = millis();
  currentPage = PAGE_OTA;
  drawOtaPage();
  return true;
}

// ========== BLE CALLBACKS ==========
class TextCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    std::string payload = characteristic->getValue();
    if (payload.empty()) return;

    String command = normalizeNoteText(String(payload.c_str()));
    bool redraw = false;

    if (command.startsWith("NOTE:")) {
      addNote(command.substring(5));
      redraw = true;
    } else if (command.startsWith("DEL:")) {
      int idx = command.substring(4).toInt();
      if (idx > 0) {
        deleteNote(idx - 1);
        redraw = true;
      }
    } else if (command.startsWith("OPEN:")) {
      int idx = command.substring(5).toInt();
      if (idx > 0 && idx <= noteCount) {
        noteViewIndex = idx - 1;
        noteScrollLine = 0;
        currentPage = PAGE_NOTE_VIEW;
        redraw = true;
      }
    } else if (command.startsWith("VIEW:")) {
      int idx = command.substring(5).toInt();
      if (idx > 0 && idx <= noteCount) {
        noteViewIndex = idx - 1;
        noteScrollLine = 0;
        currentPage = PAGE_NOTE_VIEW;
        redraw = true;
      }
    } else if (command == "CLEAR_NOTES") {
      clearAllNotes();
      redraw = true;
    } else if (command.startsWith("WIFI:")) {
      String value = command.substring(5);
      int sep = value.indexOf('|');
      String ssid = sep >= 0 ? value.substring(0, sep) : value;
      String pass = sep >= 0 ? value.substring(sep + 1) : "";
      ssid.trim();
      pass.trim();
      storeWiFiCredentials(ssid, pass);
      redraw = true;
    } else if (command == "WIFI:OFF") {
      storeWiFiCredentials("", "");
      redraw = true;
    } else if (command == "TIME:SYNC") {
      syncTimeFromNTP();
      redraw = true;
    } else if (command.startsWith("TIME:")) {
      unsigned long epoch = command.substring(5).toInt();
      struct timeval tv = { (time_t)epoch, 0 };
      settimeofday(&tv, nullptr);
      redraw = true;
    } else if (command == "LIGHT:AUTO") {
      setLightMode(LIGHT_MODE_AUTO);
      redraw = true;
    } else if (command == "LIGHT:ON") {
      setLightMode(LIGHT_MODE_FORCED_ON);
      setFrontLight(true);
      redraw = true;
    } else if (command == "LIGHT:OFF") {
      setLightMode(LIGHT_MODE_FORCED_OFF);
      setFrontLight(false);
      redraw = true;
    } else if (command == "OTA:START") {
      redraw = startOtaSession();
    } else if (command == "OTA:STOP") {
      stopOtaSession();
      redraw = true;
    } else if (command == "BLE:OFF") {
      bleStop();
      redraw = true;
    } else {
      addNote(command);
      redraw = true;
    }

    lastActivityMs = millis();
    if (redraw) {
      drawCurrentPage();
    }
  }
};

// ========== BLE CONTROL ==========
void bleStart() {
  if (bleEnabled || otaActive) return;

  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  BLEService *service = bleServer->createService(SERVICE_UUID);
  textChar = service->createCharacteristic(CHAR_TEXT_UUID, BLECharacteristic::PROPERTY_WRITE);
  textChar->addDescriptor(new BLE2902());
  textChar->setCallbacks(new TextCallback());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  bleEnabled = true;
  bleEnableTs = millis();
  drawCurrentPage();
}

void bleStop() {
  if (!bleEnabled) return;
  BLEDevice::deinit(true);
  bleEnabled = false;
  drawCurrentPage();
}

// ========== ENCODER / INPUT ==========
void IRAM_ATTR encoderISR() {
  uint8_t a = (uint8_t)digitalRead(ENC_A_PIN);
  uint8_t b = (uint8_t)digitalRead(ENC_B_PIN);
  uint8_t state = (a << 1) | b;
  uint8_t transition = (encoderLastState << 2) | state;

  if (transition == 0b1101 || transition == 0b0100 || transition == 0b0010 || transition == 0b1011) {
    encoderDelta++;
  } else if (transition == 0b1110 || transition == 0b0111 || transition == 0b0001 || transition == 0b1000) {
    encoderDelta--;
  }
  encoderLastState = state;
}

void setupEncoderPins() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);
  encoderLastState = ((uint8_t)digitalRead(ENC_A_PIN) << 1) | (uint8_t)digitalRead(ENC_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);
}

PageMode mainPageForStep(int step) {
  return cycleMainPage(currentPage, step);
}

void ensureSelectedNoteVisible() {
  if (noteCount == 0) {
    selectedNoteIndex = 0;
    notesPageOffset = 0;
    return;
  }

  if (selectedNoteIndex < 0) selectedNoteIndex = 0;
  if (selectedNoteIndex >= noteCount) selectedNoteIndex = noteCount - 1;
  if (selectedNoteIndex < notesPageOffset) {
    notesPageOffset = (selectedNoteIndex / NOTES_PER_PAGE) * NOTES_PER_PAGE;
  }
  if (selectedNoteIndex >= notesPageOffset + NOTES_PER_PAGE) {
    notesPageOffset = (selectedNoteIndex / NOTES_PER_PAGE) * NOTES_PER_PAGE;
  }
}

void handleEncoderStep(int step) {
  if (otaActive) return;
  if (step == 0) return;

  if (currentPage == PAGE_NOTES) {
    if (noteCount == 0) {
      currentPage = mainPageForStep(step);
      drawCurrentPage();
      return;
    }

    selectedNoteIndex += step;
    ensureSelectedNoteVisible();
    drawNotesList();
    return;
  }

  if (currentPage == PAGE_NOTE_VIEW) {
    String noteText = readNote(noteViewIndex);
    int totalLines = max(1, (noteText.length() + NOTE_CHARS_PER_LINE - 1) / NOTE_CHARS_PER_LINE);
    noteScrollLine += step;
    if (noteScrollLine < 0) noteScrollLine = 0;
    if (noteScrollLine > max(0, totalLines - NOTE_VISIBLE_LINES)) {
      noteScrollLine = max(0, totalLines - NOTE_VISIBLE_LINES);
    }
    drawNote(noteViewIndex);
    return;
  }

  currentPage = mainPageForStep(step);
  drawCurrentPage();
}

void handleShortPress() {
  lastActivityMs = millis();

  switch (currentPage) {
    case PAGE_DASHBOARD:
      toggleLightMode();
      drawDashboard();
      break;
    case PAGE_NOTES:
      if (noteCount > 0) {
        noteViewIndex = selectedNoteIndex;
        noteScrollLine = 0;
        currentPage = PAGE_NOTE_VIEW;
        drawNote(noteViewIndex);
      }
      break;
    case PAGE_NOTE_VIEW:
      currentPage = PAGE_NOTES;
      drawNotesList();
      break;
    case PAGE_SYSTEM:
      currentPage = PAGE_OTA;
      drawOtaPage();
      break;
    case PAGE_OTA:
      if (otaActive) {
        stopOtaSession();
      } else {
        startOtaSession();
      }
      break;
  }
}

void handleLongPress() {
  lastActivityMs = millis();

  if (currentPage == PAGE_NOTE_VIEW) {
    if (deleteNote(noteViewIndex)) {
      currentPage = PAGE_NOTES;
      if (selectedNoteIndex >= noteCount) selectedNoteIndex = max(0, noteCount - 1);
      ensureSelectedNoteVisible();
      drawNotesList();
    }
    return;
  }

  if (currentPage == PAGE_OTA) {
    if (otaActive) {
      stopOtaSession();
    } else {
      startOtaSession();
    }
    return;
  }

  if (!bleEnabled) {
    bleStart();
  } else {
    bleStop();
  }
}

// ========== LIGHT / POWER ==========
void setupLight() {
  ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(LED_PWM_PIN, LEDC_CH);
  ledcWrite(LEDC_CH, 0);
}

void goToDeepSleep() {
  setFrontLight(false);
  if (bleEnabled) {
    bleStop();
  }
  if (otaActive) {
    stopOtaSession();
  }

  delay(20);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)ENC_BTN_PIN, 0);
  Serial.println("Entering deep sleep...");
  prefs.end();
  display.hibernate();
  esp_deep_sleep_start();
}

// ========== SETUP & LOOP ==========
void setup() {
  Serial.begin(115200);
  delay(100);

  prefs.begin(PREF_NS, false);

  Wire.begin(I2C_SDA, I2C_SCL);
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  pinMode(BAT_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  setupLight();
  setupEncoderPins();

  display.init();
  display.setRotation(1);

  ensureFilesystems();
  refreshNotesIndex();
  setFrontLight(false);

  String ssid;
  String pass;
  if (loadWiFiCredentials(ssid, pass)) {
    syncTimeFromNTP();
  }

  drawCurrentPage();
  lastActivityMs = millis();
}

void loop() {
  int32_t delta = 0;
  noInterrupts();
  delta = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (delta != 0) {
    encoderAccum += delta;
    while (encoderAccum >= 4) {
      handleEncoderStep(1);
      encoderAccum -= 4;
      lastActivityMs = millis();
    }
    while (encoderAccum <= -4) {
      handleEncoderStep(-1);
      encoderAccum += 4;
      lastActivityMs = millis();
    }
  }

  static bool btnDown = false;
  static unsigned long btnDownTs = 0;
  int btn = digitalRead(ENC_BTN_PIN);
  if (btn == LOW && !btnDown) {
    btnDown = true;
    btnDownTs = millis();
  } else if (btn == HIGH && btnDown) {
    btnDown = false;
    unsigned long held = millis() - btnDownTs;
    if (held < 800) {
      handleShortPress();
    } else {
      handleLongPress();
    }
  }

  if (bleEnabled && (millis() - bleEnableTs > BLE_ENABLE_TIMEOUT_MS)) {
    bleStop();
  }

  if (otaActive) {
    ArduinoOTA.handle();
    if (millis() - otaStartTs > OTA_ENABLE_TIMEOUT_MS) {
      stopOtaSession();
    }
  }

  autoAdjustLight();

  if (millis() - lastDashboardRefreshMs > 60000UL && currentPage == PAGE_DASHBOARD) {
    drawDashboard();
  }
  if (millis() - lastSystemRefreshMs > 60000UL && currentPage == PAGE_SYSTEM) {
    drawSystem();
  }
  if (millis() - lastOtaRefreshMs > 30000UL && currentPage == PAGE_OTA) {
    drawOtaPage();
  }

  if (!bleEnabled && !otaActive) {
    unsigned long idleMs = millis() - lastActivityMs;
    if (idleMs > INACTIVITY_MIN * 60UL * 1000UL) {
      drawDashboard();
      delay(200);
      goToDeepSleep();
    }
  }

  delay(20);
}
