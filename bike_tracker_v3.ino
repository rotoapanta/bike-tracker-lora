#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"
#include <math.h>

#define FW_NAME    "BikeTracker"
#define FW_VERSION "PRO v1.0"

// ===================== OBJETOS =====================
HT_st7735 st7735;
TinyGPSPlus gps;

// ===================== CONFIG =====================
#define RIDER_ID       "R2"     // cambiar en la otra placa a R1
#define VGNSS_CTRL     3
#define GPS_RX_PIN     33
#define GPS_TX_PIN     34
#define GPS_BAUD       115200

#define LORA_SCK_PIN   9
#define LORA_MISO_PIN  11
#define LORA_MOSI_PIN  10
#define LORA_CS_PIN    8
#define LORA_RST_PIN   12
#define LORA_BUSY_PIN  13
#define LORA_DIO1_PIN  14

static const float RF_FREQUENCY_MHZ = 915.0;
static const float LORA_BW_KHZ      = 125.0;
static const uint8_t LORA_SF        = 7;
static const uint8_t LORA_CR        = 5;
static const uint8_t LORA_SYNC_WORD = 0x12;
static const int8_t  TX_POWER_DBM   = 10;
static const uint16_t PREAMBLE_LEN  = 8;

// ===================== TIEMPOS =====================
const unsigned long displayInterval      = 700;
const unsigned long sendInterval         = 3000;
const unsigned long screenSwitchInterval = 3000;
const unsigned long peerTimeoutMs        = 10000;

// ===================== ALERTAS =====================
const double DIST_WARNING_M  = 30.0;
const double DIST_CRITICAL_M = 60.0;

enum AlertLevel {
  ALERT_NONE = 0,
  ALERT_NORMAL,
  ALERT_WARNING,
  ALERT_CRITICAL
};

// ===================== ESTRUCTURAS =====================
struct RiderState {
  String riderId = "";
  double lat = 0.0;
  double lon = 0.0;
  double speedKmh = 0.0;
  double courseDeg = 0.0;
  int satellites = 0;
  float batteryVoltage = 4.02f;
  bool gpsValid = false;
  unsigned long lastUpdateMs = 0;
};

// ===================== GLOBALES =====================
SPIClass spiLoRa(FSPI);
SX1262 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

RiderState selfState;
RiderState peerState;

volatile bool radioFlag = false;
bool loraEnabled = false;

int currentScreen = 0;
const int totalScreens = 3;

unsigned long lastDisplay = 0;
unsigned long lastSend = 0;
unsigned long lastScreenSwitch = 0;

String lastLoRaStatus = "LoRa init";
String lastRxMessage  = "No RX";

int16_t lastRssi = 0;
float lastSnr = 0.0f;
int txCount = 0;
int rxCount = 0;

double distanceMeters = -1.0;
bool distanceValid = false;
AlertLevel currentAlert = ALERT_NONE;

// ===================== ISR =====================
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setRadioFlag(void) {
  radioFlag = true;
}

// ===================== UTILS =====================
double deg2rad(double deg) {
  return deg * PI / 180.0;
}

double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);

  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dLon / 2.0) * sin(dLon / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

bool peerIsFresh() {
  if (peerState.riderId.length() == 0) return false;
  return (millis() - peerState.lastUpdateMs) <= peerTimeoutMs;
}

float readBatteryVoltage() {
  return 4.02f;
}

String shortText(const String& s, int maxLen) {
  if ((int)s.length() <= maxLen) return s;
  return s.substring(0, maxLen);
}

String alertText(AlertLevel a) {
  switch (a) {
    case ALERT_NORMAL:   return "NORMAL";
    case ALERT_WARNING:  return "WARN";
    case ALERT_CRITICAL: return "CRIT";
    default:             return "---";
  }
}

// ===================== INIT =====================
void init_tft() {
  st7735.st7735_init();
  st7735.st7735_fill_screen(ST7735_BLACK);
  delay(100);
}

void init_gps() {
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  delay(100);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS init OK");
}

bool init_lora() {
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

  int state = radio.begin(
    RF_FREQUENCY_MHZ,
    LORA_BW_KHZ,
    LORA_SF,
    LORA_CR,
    LORA_SYNC_WORD,
    TX_POWER_DBM,
    PREAMBLE_LEN
  );

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("radio.begin failed, code=");
    Serial.println(state);
    lastLoRaStatus = "LoRa FAIL";
    return false;
  }

  radio.setCRC(false);
  radio.setDio1Action(setRadioFlag);

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("startReceive failed, code=");
    Serial.println(state);
    lastLoRaStatus = "RX FAIL";
    return false;
  }

  lastLoRaStatus = "LoRa OK";
  Serial.println("LoRa init OK");
  return true;
}

// ===================== GPS =====================
void read_gps() {
  unsigned long start = millis();
  while (Serial1.available() > 0 && (millis() - start) < 10) {
    gps.encode(Serial1.read());
  }
}

void update_self_state() {
  selfState.riderId = RIDER_ID;
  selfState.lastUpdateMs = millis();
  selfState.batteryVoltage = readBatteryVoltage();

  selfState.gpsValid = gps.location.isValid();

  if (gps.location.isValid()) {
    selfState.lat = gps.location.lat();
    selfState.lon = gps.location.lng();
  } else {
    selfState.lat = 0.0;
    selfState.lon = 0.0;
  }

  selfState.speedKmh  = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  selfState.courseDeg = gps.course.isValid() ? gps.course.deg() : 0.0;
  selfState.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
}

// ===================== PARSER =====================
bool parsePeerPacket(const String& sRaw) {
  String s = sRaw;

  s.trim();
  s.replace("\r", "");
  s.replace("\n", "");

  String parts[9];
  int idx = 0;
  int start = 0;

  for (int i = 0; i < s.length(); i++) {
    if (s[i] == ',') {
      if (idx < 9) {
        parts[idx++] = s.substring(start, i);
        start = i + 1;
      }
    }
  }

  parts[idx++] = s.substring(start);

  if (idx < 9) return false;
  if (parts[0] != "BKT") return false;
  if (parts[1] == RIDER_ID) return false;

  peerState.riderId        = parts[1];
  peerState.lat            = parts[2].toDouble();
  peerState.lon            = parts[3].toDouble();
  peerState.speedKmh       = parts[4].toDouble();
  peerState.courseDeg      = parts[5].toDouble();
  peerState.satellites     = parts[6].toInt();
  peerState.batteryVoltage = parts[7].toFloat();
  peerState.gpsValid       = !(peerState.lat == 0.0 && peerState.lon == 0.0);
  peerState.lastUpdateMs   = millis();

  lastRxMessage = parts[8];

  return true;
}

// ===================== CALCULOS =====================
void update_distance_and_alert() {
  distanceValid = false;
  distanceMeters = -1.0;
  currentAlert = ALERT_NONE;

  if (!(selfState.gpsValid && peerIsFresh() && peerState.gpsValid)) {
    return;
  }

  distanceMeters = haversineMeters(
    selfState.lat, selfState.lon,
    peerState.lat, peerState.lon
  );
  distanceValid = true;

  if (distanceMeters >= DIST_CRITICAL_M) {
    currentAlert = ALERT_CRITICAL;
  } else if (distanceMeters >= DIST_WARNING_M) {
    currentAlert = ALERT_WARNING;
  } else {
    currentAlert = ALERT_NORMAL;
  }
}

// ===================== TFT =====================
void draw_screen_1() {
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0,  "1/3 MAIN");
  st7735.st7735_write_str(0, 15, "NODE:" + String(RIDER_ID));
  st7735.st7735_write_str(0, 30, "VEL:" + String(selfState.speedKmh, 1));

  if (distanceValid) {
    st7735.st7735_write_str(0, 45, "DST:" + String(distanceMeters, 1) + "m");
  } else {
    st7735.st7735_write_str(0, 45, "DST:---");
  }

  st7735.st7735_write_str(0, 60, "ALR:" + alertText(currentAlert));
}

void draw_screen_2() {
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0,  "2/3 LORA");
  st7735.st7735_write_str(0, 15, "STA:" + shortText(lastLoRaStatus, 10));
  st7735.st7735_write_str(0, 30, "RSSI:" + String(lastRssi));
  st7735.st7735_write_str(0, 45, "RX#:" + String(rxCount));
  st7735.st7735_write_str(0, 60, shortText(lastRxMessage, 12));
}

void draw_screen_3() {
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0,  "3/3 GPS");
  st7735.st7735_write_str(0, 15, "GPS:" + String(selfState.gpsValid ? "OK" : "NO"));
  st7735.st7735_write_str(0, 30, "SAT:" + String(selfState.satellites));
  st7735.st7735_write_str(0, 45, "PEER:" + String(peerIsFresh() ? peerState.riderId : "---"));
  st7735.st7735_write_str(0, 60, "TX:" + String(txCount));
}

void update_display() {
  if (millis() - lastScreenSwitch >= screenSwitchInterval) {
    lastScreenSwitch = millis();
    currentScreen++;
    if (currentScreen >= totalScreens) currentScreen = 0;

    Serial.print("Cambio de pantalla a: ");
    Serial.println(currentScreen);
  }

  switch (currentScreen) {
    case 0: draw_screen_1(); break;
    case 1: draw_screen_2(); break;
    case 2: draw_screen_3(); break;
    default:
      currentScreen = 0;
      draw_screen_1();
      break;
  }
}

// ===================== LORA =====================
void send_lora_packet() {
  if (!loraEnabled) return;

  txCount++;

  String msg = selfState.gpsValid ? "OK" : "NOFIX";

  String payload = "BKT," +
                   String(RIDER_ID) + "," +
                   String(selfState.lat, 6) + "," +
                   String(selfState.lon, 6) + "," +
                   String(selfState.speedKmh, 1) + "," +
                   String(selfState.courseDeg, 1) + "," +
                   String(selfState.satellites) + "," +
                   String(selfState.batteryVoltage, 2) + "," +
                   msg;

  Serial.print("Sending: ");
  Serial.println(payload);

  int state = radio.transmit(payload);
  delay(50);

  if (state == RADIOLIB_ERR_NONE) {
    lastLoRaStatus = "TX OK";
  } else {
    lastLoRaStatus = "TX FAIL";
    Serial.print("TX fail code=");
    Serial.println(state);
  }

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    lastLoRaStatus = "RX FAIL";
    Serial.print("startReceive fail code=");
    Serial.println(state);
  }
}

void handle_lora_receive() {
  if (!loraEnabled) return;
  if (!radioFlag) return;

  radioFlag = false;

  String str;
  int state = radio.readData(str);

  if (state == RADIOLIB_ERR_NONE) {
    lastRssi = radio.getRSSI();
    lastSnr  = radio.getSNR();

    Serial.print("RX: ");
    Serial.print(str);
    Serial.print(" | RSSI: ");
    Serial.print(lastRssi);
    Serial.print(" | SNR: ");
    Serial.println(lastSnr);

    if (parsePeerPacket(str)) {
      rxCount++;
      lastLoRaStatus = "RX OK";
    } else {
      Serial.println("WARN: paquete no valido, ignorado");
      // NO cambiar lastLoRaStatus a RX BAD
      // así conservas el último estado bueno
    }
  } else {
    Serial.print("readData fail code=");
    Serial.println(state);
  }

  int rxState = radio.startReceive();
  if (rxState != RADIOLIB_ERR_NONE) {
    Serial.print("restart RX fail code=");
    Serial.println(rxState);
    lastLoRaStatus = "RX FAIL";
  }
}

// ===================== DEBUG =====================
void print_status() {
  Serial.println("======== BIKE TRACKER PRO ========");
  Serial.println("FW: " + String(FW_NAME) + " " + String(FW_VERSION));
  Serial.println("RIDER: " + String(RIDER_ID));
  Serial.println("GPS: " + String(selfState.gpsValid ? "OK" : "NO"));
  Serial.println("SAT: " + String(selfState.satellites));
  Serial.println("LAT: " + String(selfState.lat, 6));
  Serial.println("LON: " + String(selfState.lon, 6));
  Serial.println("VEL: " + String(selfState.speedKmh, 1));
  Serial.println("LORA: " + lastLoRaStatus);

  if (peerIsFresh()) {
    Serial.println("PEER: " + peerState.riderId);
    Serial.println("RSSI: " + String(lastRssi));
  } else {
    Serial.println("PEER: ---");
  }

  if (distanceValid) {
    Serial.println("DIST: " + String(distanceMeters, 1) + " m");
    Serial.println("ALERT: " + alertText(currentAlert));
  } else {
    Serial.println("DIST: ---");
  }

  Serial.println("==================================");
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  init_tft();

  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, FW_NAME);
  st7735.st7735_write_str(0, 15, FW_VERSION);
  st7735.st7735_write_str(0, 30, "Node " + String(RIDER_ID));

  init_gps();

  loraEnabled = init_lora();
  st7735.st7735_write_str(0, 45, loraEnabled ? "LoRa OK" : "LoRa FAIL");
  delay(1200);

  currentScreen = 0;
  lastScreenSwitch = millis();
  lastDisplay = millis();
  lastSend = millis();

  draw_screen_1();

  Serial.println("System init complete");
}

// ===================== LOOP =====================
void loop() {
  unsigned long now = millis();

  if (now - lastDisplay >= displayInterval) {
    lastDisplay = now;
    update_display();
  }

  read_gps();
  update_self_state();
  handle_lora_receive();
  update_distance_and_alert();

  if (loraEnabled && (now - lastSend >= sendInterval)) {
    lastSend = now;
    send_lora_packet();
  }

  static unsigned long dbg = 0;
  if (now - dbg >= 2000) {
    dbg = now;
    print_status();
  }
}