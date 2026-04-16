#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"
#include <math.h>

#define FW_NAME    "BikeTracker"
#define FW_VERSION "v2.2"

HT_st7735 st7735;
TinyGPSPlus gps;

#define RIDER_ID       "R2"
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

SPIClass spiLoRa(FSPI);
SX1262 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

static const float RF_FREQUENCY_MHZ = 915.0;
static const float LORA_BW_KHZ      = 125.0;
static const uint8_t LORA_SF        = 7;
static const uint8_t LORA_CR        = 5;
static const uint8_t LORA_SYNC_WORD = 0x12;
static const int8_t  TX_POWER_DBM   = 10;
static const uint16_t PREAMBLE_LEN  = 8;

const unsigned long displayInterval      = 1000;
const unsigned long sendInterval         = 3000;
const unsigned long screenSwitchInterval = 3000;
const unsigned long peerTimeoutMs        = 10000;

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

RiderState selfState;
RiderState peerState;

volatile bool radioFlag = false;

int currentScreen = 0;
const int totalScreens = 3;
unsigned long lastDisplay = 0;
unsigned long lastSend = 0;
unsigned long lastScreenSwitch = 0;

String lastLoRaStatus = "LoRa init";
String lastRxMessage  = "No RX";
int16_t lastRssi = 0;
float lastSnr = 0.0f;
int txNumber = 0;

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setRadioFlag(void) {
  radioFlag = true;
}

bool peerIsFresh() {
  if (peerState.riderId.length() == 0) return false;
  return (millis() - peerState.lastUpdateMs) <= peerTimeoutMs;
}

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
  spiLoRa.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
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

void read_gps() {
  unsigned long start = millis();
  while (Serial1.available() > 0 && (millis() - start) < 10) {
    gps.encode(Serial1.read());
  }
}

void update_self_state() {
  selfState.riderId = RIDER_ID;
  selfState.lastUpdateMs = millis();
  selfState.gpsValid = gps.location.isValid();

  if (gps.location.isValid()) {
    selfState.lat = gps.location.lat();
    selfState.lon = gps.location.lng();
  } else {
    selfState.lat = 0.0;
    selfState.lon = 0.0;
  }

  selfState.speedKmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  selfState.courseDeg = gps.course.isValid() ? gps.course.deg() : 0.0;
  selfState.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
}

bool parsePeerPacket(const String& sRaw) {
  String s = sRaw;
  s.trim();

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

  if (idx < 8) return false;
  parts[idx++] = s.substring(start);

  if (idx != 9) return false;
  if (parts[0] != "BKT") return false;
  if (parts[1] == RIDER_ID) return false;

  peerState.riderId      = parts[1];
  peerState.lat          = parts[2].toDouble();
  peerState.lon          = parts[3].toDouble();
  peerState.speedKmh     = parts[4].toDouble();
  peerState.courseDeg    = parts[5].toDouble();
  peerState.satellites   = parts[6].toInt();
  peerState.batteryVoltage = parts[7].toFloat();
  peerState.gpsValid     = !(peerState.lat == 0.0 && peerState.lon == 0.0);
  peerState.lastUpdateMs = millis();
  lastRxMessage = parts[8];

  return true;
}

void draw_screen_1() {
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0,  "SCREEN 1");
  st7735.st7735_write_str(0, 15, "Node " + String(RIDER_ID));
  st7735.st7735_write_str(0, 30, "VEL:" + String(selfState.speedKmh, 1));
  st7735.st7735_write_str(0, 45, "GPS:" + String(selfState.gpsValid ? "OK" : "NO"));
  st7735.st7735_write_str(0, 60, "SAT:" + String(selfState.satellites));
}

void draw_screen_2() {
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0,  "SCREEN 2");
  st7735.st7735_write_str(0, 15, "LoRa:" + lastLoRaStatus);
  st7735.st7735_write_str(0, 30, "RSSI:" + String(lastRssi));
  st7735.st7735_write_str(0, 45, "MSG:");
  st7735.st7735_write_str(0, 60, lastRxMessage);
}

void draw_screen_3() {
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0,  "SCREEN 3");
  st7735.st7735_write_str(0, 15, "LAT:" + String(selfState.lat, 3));
  st7735.st7735_write_str(0, 30, "LON:" + String(selfState.lon, 3));
  st7735.st7735_write_str(0, 45, "PEER:" + String(peerIsFresh() ? peerState.riderId : "---"));
  st7735.st7735_write_str(0, 60, "TX#" + String(txNumber));
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

void send_lora_packet() {
  txNumber++;

  String payload = "BKT," +
                   String(RIDER_ID) + "," +
                   String(selfState.lat, 6) + "," +
                   String(selfState.lon, 6) + "," +
                   String(selfState.speedKmh, 1) + "," +
                   String(selfState.courseDeg, 1) + "," +
                   String(selfState.satellites) + "," +
                   String(selfState.batteryVoltage, 2) + ",OK";

  Serial.print("Sending: ");
  Serial.println(payload);

  int state = radio.transmit(payload);
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
  if (!radioFlag) return;
  radioFlag = false;

  String str;
  int state = radio.readData(str);

  if (state == RADIOLIB_ERR_NONE) {
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();

    Serial.print("RX: ");
    Serial.println(str);

    if (parsePeerPacket(str)) {
      lastLoRaStatus = "RX OK";
    } else {
      lastLoRaStatus = "RX BAD";
    }
  }

  radio.startReceive();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  init_tft();

  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, FW_NAME);
  st7735.st7735_write_str(0, 15, FW_VERSION);
  st7735.st7735_write_str(0, 30, "Node " + String(RIDER_ID));

  init_gps();

  st7735.st7735_write_str(0, 45, "LoRa SKIP");
  delay(1200);

  currentScreen = 0;
  lastScreenSwitch = millis();
  lastDisplay = millis();
  lastSend = millis();

  draw_screen_1();

  Serial.println("System init complete");
}

void loop() {
  unsigned long now = millis();

  if (now - lastDisplay >= displayInterval) {
    lastDisplay = now;
    update_display();
  }

  read_gps();
  update_self_state();

  static unsigned long dbg = 0;
  if (now - dbg >= 1000) {
    dbg = now;
    Serial.print("Loop OK | screen=");
    Serial.print(currentScreen);
    Serial.print(" | gpsValid=");
    Serial.print(selfState.gpsValid);
    Serial.print(" | sats=");
    Serial.println(selfState.satellites);
  }
}
