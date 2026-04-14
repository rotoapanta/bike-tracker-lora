/*******************************************************
 * Proyecto:   Bike Tracker LoRa (ESP32-S3)
 * Archivo:    bike_tracker_v1_alerts.ino
 * Versión:    v1.1
 * Fecha:      2026-04-13
 *
 * Descripción:
 * Sistema de seguimiento entre bicicletas utilizando
 * módulos Wireless Tracker (ESP32-S3 + SX1262 + GPS).
 *
 * Funcionalidades:
 * - Lectura de GPS (latitud, longitud, velocidad, rumbo)
 * - Comunicación LoRa P2P (915 MHz)
 * - Intercambio de datos entre ciclistas
 * - Cálculo de distancia entre riders (Haversine)
 * - Visualización en pantalla TFT (ST7735)
 * - Sistema multipantalla automático (1/3, 2/3, 3/3)
 * - Alertas de separación
 *
 * Pantallas:
 * 1/3: Datos principales (velocidad, distancia, peer, GPS)
 * 2/3: Estado (LoRa, RSSI, mensaje, batería)
 * 3/3: GPS detalle (lat, lon, rumbo, satélites)
 * ALERT: Pantalla temporal de alerta de separación
 *
 * Hardware:
 * - Wireless Tracker Heltec
 * - ESP32-S3
 * - SX1262 LoRa (915 MHz)
 * - GNSS integrado
 * - TFT 0.96" 80x160 ST7735
 *
 * Autor:      Roberto Toapanta
 *
 * Notas:
 * - Cambiar RIDER_ID a "R1" o "R2" según el nodo
 * - Frecuencia LoRa configurada para 915 MHz
 * - Intervalo TX: 3 segundos
 * - Cambio automático de pantalla: 3 segundos
 *******************************************************/

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"
#include <math.h>

#define FW_NAME    "BikeTracker"
#define FW_VERSION "v1.1"

HT_st7735 st7735;
TinyGPSPlus gps;

// ===================== CONFIG GENERAL =====================
#define RIDER_ID                   "R2"   // En la otra bici cambiar a "R1"
#define VGNSS_CTRL                 3

#define RF_FREQUENCY               915000000   // 915 MHz
#define TX_OUTPUT_POWER            10          // dBm

#define LORA_BANDWIDTH             0           // 125 kHz
#define LORA_SPREADING_FACTOR      7           // SF7
#define LORA_CODINGRATE            1           // 4/5
#define LORA_PREAMBLE_LENGTH       8
#define LORA_SYMBOL_TIMEOUT        0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false

#define BUFFER_SIZE                160

const unsigned long displayInterval      = 1000;
const unsigned long sendInterval         = 3000;
const unsigned long peerTimeoutMs        = 10000;
const unsigned long screenSwitchInterval = 3000;

// ===================== ALERTAS =====================
const double DIST_WARNING_M  = 30.0;
const double DIST_CRITICAL_M = 60.0;
const unsigned long alertScreenDurationMs = 2500;

enum SeparationAlertLevel {
  ALERT_NONE = 0,
  ALERT_NORMAL,
  ALERT_WARNING,
  ALERT_CRITICAL
};

SeparationAlertLevel currentAlertLevel = ALERT_NONE;
SeparationAlertLevel lastPrintedAlertLevel = ALERT_NONE;

double lastDistanceMeters = -1.0;
bool distanceValid = false;

bool showAlertScreen = false;
unsigned long alertScreenUntil = 0;
String alertTitle = "";
String alertLine1 = "";
String alertLine2 = "";

// ===================== PANTALLAS =====================
int currentScreen = 0;
const int totalScreens = 3;
unsigned long lastScreenSwitch = 0;

// ===================== OBJETOS LORA =====================
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

typedef enum
{
  LOWPOWER,
  STATE_RX,
  STATE_TX
} States_t;

States_t state;

// ===================== ESTADO =====================
struct RiderState {
  String riderId;
  double lat;
  double lon;
  double speedKmh;
  double courseDeg;
  int satellites;
  float batteryVoltage;
  bool gpsValid;
  unsigned long lastUpdateMs;
};

RiderState selfState;
RiderState peerState;

int16_t Rssi = 0;
int16_t rxSize = 0;
int16_t txNumber = 0;
int16_t rxNumber = 0;

unsigned long lastDisplay = 0;
unsigned long lastSend = 0;

String lastLoRaStatus = "LoRa init";
String lastRxMessage  = "No RX";
String selfMessage    = "OK";

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

// Temporal. Luego reemplazar por lectura real.
float readBatteryVoltage() {
  return 4.02f;
}

// ===================== ALERTAS =====================
void triggerAlertScreen(const String& title, const String& line1, const String& line2) {
  alertTitle = title;
  alertLine1 = line1;
  alertLine2 = line2;
  showAlertScreen = true;
  alertScreenUntil = millis() + alertScreenDurationMs;
}

void updateDistanceAndAlerts() {
  distanceValid = false;
  lastDistanceMeters = -1.0;

  if (!(peerIsFresh() && selfState.gpsValid && peerState.gpsValid)) {
    currentAlertLevel = ALERT_NONE;
    return;
  }

  lastDistanceMeters = haversineMeters(
    selfState.lat, selfState.lon,
    peerState.lat, peerState.lon
  );
  distanceValid = true;

  SeparationAlertLevel newLevel = ALERT_NORMAL;

  if (lastDistanceMeters >= DIST_CRITICAL_M) {
    newLevel = ALERT_CRITICAL;
  } else if (lastDistanceMeters >= DIST_WARNING_M) {
    newLevel = ALERT_WARNING;
  } else {
    newLevel = ALERT_NORMAL;
  }

  // Solo disparar pantalla de alerta cuando cambia a un estado más importante
  if (newLevel != currentAlertLevel) {
    if (newLevel == ALERT_WARNING) {
      triggerAlertScreen(
        "ALERTA",
        "SEPARACION",
        String(lastDistanceMeters, 1) + "m"
      );
    } else if (newLevel == ALERT_CRITICAL) {
      triggerAlertScreen(
        "CRITICA",
        "MUY LEJOS",
        String(lastDistanceMeters, 1) + "m"
      );
    }
  }

  currentAlertLevel = newLevel;
}

String getAlertText() {
  switch (currentAlertLevel) {
    case ALERT_NORMAL:   return "NORMAL";
    case ALERT_WARNING:  return "WARNING";
    case ALERT_CRITICAL: return "CRITICAL";
    default:             return "---";
  }
}

// ===================== LORA CALLBACKS =====================
void OnTxDone(void)
{
  Serial.println("TX done");
  lastLoRaStatus = "TX OK";
  state = STATE_RX;
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.println("TX Timeout");
  lastLoRaStatus = "TX Timeout";
  state = STATE_TX;
}

bool parsePeerPacket(const char* packet) {
  String s = String(packet);
  s.trim();

  // Formato:
  // BKT,R1,lat,lon,speed,course,sats,bat,msg
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

  peerState.riderId        = parts[1];
  peerState.lat            = parts[2].toDouble();
  peerState.lon            = parts[3].toDouble();
  peerState.speedKmh       = parts[4].toDouble();
  peerState.courseDeg      = parts[5].toDouble();
  peerState.satellites     = parts[6].toInt();
  peerState.batteryVoltage = parts[7].toFloat();

  if (peerState.lat == 0.0 && peerState.lon == 0.0) {
    peerState.gpsValid = false;
  } else {
    peerState.gpsValid = true;
  }

  peerState.lastUpdateMs = millis();
  lastRxMessage = parts[8];

  return true;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rxNumber++;
  Rssi = rssi;
  rxSize = size;

  if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';

  Radio.Sleep();

  Serial.printf("RX: %s | RSSI: %d | SNR: %d\n", rxpacket, rssi, snr);

  if (parsePeerPacket((char*)rxpacket)) {
    lastLoRaStatus = "RX OK";
  } else {
    lastLoRaStatus = "RX BAD";
  }

  state = STATE_RX;
}

// ===================== INIT =====================
void init_gps()
{
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  delay(100);

  Serial1.begin(115200, SERIAL_8N1, 33, 34);
  Serial.println("GPS init OK");
}

void init_lora()
{
  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  state = STATE_RX;
  lastLoRaStatus = "LoRa 915 OK";
  Serial.println("LoRa init OK");
}

void init_tft()
{
  st7735.st7735_init();
  st7735.st7735_fill_screen(ST7735_BLACK);
  delay(100);
  st7735.st7735_write_str(0, 0, (String)"Boot...");
}

// ===================== GPS =====================
void read_gps()
{
  while (Serial1.available() > 0)
  {
    gps.encode(Serial1.read());
  }
}

void update_self_state()
{
  selfState.riderId        = RIDER_ID;
  selfState.batteryVoltage = readBatteryVoltage();
  selfState.lastUpdateMs   = millis();

  if (gps.location.isValid()) {
    selfState.lat      = gps.location.lat();
    selfState.lon      = gps.location.lng();
    selfState.gpsValid = true;
  } else {
    selfState.gpsValid = false;
  }

  if (gps.speed.isValid()) {
    selfState.speedKmh = gps.speed.kmph();
  } else {
    selfState.speedKmh = 0.0;
  }

  if (gps.course.isValid()) {
    selfState.courseDeg = gps.course.deg();
  } else {
    selfState.courseDeg = 0.0;
  }

  if (gps.satellites.isValid()) {
    selfState.satellites = gps.satellites.value();
  } else {
    selfState.satellites = 0;
  }
}

// ===================== TFT =====================
void draw_alert_screen()
{
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0,  "ALERT");
  st7735.st7735_write_str(0, 20, alertTitle);
  st7735.st7735_write_str(0, 40, alertLine1);
  st7735.st7735_write_str(0, 60, alertLine2);
}

void draw_screen_1()
{
  st7735.st7735_fill_screen(ST7735_BLACK);

  st7735.st7735_write_str(0, 0, "1/3 BIKE " + String(RIDER_ID));
  st7735.st7735_write_str(0, 15, "VEL:" + String(selfState.speedKmh, 1) + "km/h");

  if (distanceValid) {
    st7735.st7735_write_str(0, 30, "DST:" + String(lastDistanceMeters, 1) + "m");
  } else {
    st7735.st7735_write_str(0, 30, "DST:---");
  }

  st7735.st7735_write_str(0, 45, "PEER:" + (peerIsFresh() ? peerState.riderId : "---"));

  String gpsLine = "GPS:" + String(selfState.gpsValid ? "OK" : "NO");
  gpsLine += " S:" + String(selfState.satellites);
  st7735.st7735_write_str(0, 60, gpsLine);
}

void draw_screen_2()
{
  st7735.st7735_fill_screen(ST7735_BLACK);

  st7735.st7735_write_str(0, 0, "2/3 STATUS");
  st7735.st7735_write_str(0, 15, "LORA:" + lastLoRaStatus);
  st7735.st7735_write_str(0, 30, "RSSI:" + String(Rssi));
  st7735.st7735_write_str(0, 45, "MSG:" + lastRxMessage);
  st7735.st7735_write_str(0, 60, "BAT:" + String(selfState.batteryVoltage, 2) + "V");
}

void draw_screen_3()
{
  st7735.st7735_fill_screen(ST7735_BLACK);

  st7735.st7735_write_str(0, 0, "3/3 GPS");

  if (selfState.gpsValid) {
    st7735.st7735_write_str(0, 15, "LAT:" + String(selfState.lat, 3));
    st7735.st7735_write_str(0, 30, "LON:" + String(selfState.lon, 3));
    st7735.st7735_write_str(0, 45, "CRS:" + String(selfState.courseDeg, 1));
    st7735.st7735_write_str(0, 60, "SAT:" + String(selfState.satellites));
  } else {
    st7735.st7735_write_str(0, 15, "NO GPS FIX");
    st7735.st7735_write_str(0, 30, "SAT:" + String(selfState.satellites));
    st7735.st7735_write_str(0, 45, "CRS:" + String(selfState.courseDeg, 1));
    st7735.st7735_write_str(0, 60, "WAIT...");
  }
}

void update_display()
{
  if (showAlertScreen) {
    if (millis() <= alertScreenUntil) {
      draw_alert_screen();
      return;
    } else {
      showAlertScreen = false;
    }
  }

  if (millis() - lastScreenSwitch >= screenSwitchInterval) {
    currentScreen++;
    if (currentScreen >= totalScreens) {
      currentScreen = 0;
    }
    lastScreenSwitch = millis();
  }

  switch (currentScreen)
  {
    case 0:
      draw_screen_1();
      break;
    case 1:
      draw_screen_2();
      break;
    case 2:
      draw_screen_3();
      break;
  }
}

// ===================== LORA =====================
void send_lora_packet()
{
  txNumber++;

  String payload;

  if (selfState.gpsValid) {
    payload = "BKT," +
              String(RIDER_ID) + "," +
              String(selfState.lat, 6) + "," +
              String(selfState.lon, 6) + "," +
              String(selfState.speedKmh, 1) + "," +
              String(selfState.courseDeg, 1) + "," +
              String(selfState.satellites) + "," +
              String(selfState.batteryVoltage, 2) + "," +
              selfMessage;
  } else {
    payload = "BKT," +
              String(RIDER_ID) + ",0,0," +
              String(selfState.speedKmh, 1) + "," +
              String(selfState.courseDeg, 1) + "," +
              String(selfState.satellites) + "," +
              String(selfState.batteryVoltage, 2) + ",NOFIX";
  }

  payload.toCharArray(txpacket, BUFFER_SIZE);

  Serial.printf("Sending: %s\n", txpacket);
  lastLoRaStatus = "TX:" + String(txNumber);

  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
  state = LOWPOWER;
}

void handle_lora()
{
  switch (state)
  {
    case STATE_TX:
      send_lora_packet();
      break;

    case STATE_RX:
      Radio.Rx(0);
      state = LOWPOWER;
      break;

    case LOWPOWER:
      Radio.IrqProcess();
      break;

    default:
      break;
  }
}

// ===================== DEBUG =====================
void print_status()
{
  Serial.println("======== BIKE TRACKER ========");
  Serial.println("FW: " + String(FW_NAME) + " " + String(FW_VERSION));
  Serial.println("RIDER: " + String(RIDER_ID));
  Serial.println("GPS: " + String(selfState.gpsValid ? "OK" : "NO"));
  Serial.println("LAT: " + String(selfState.lat, 6));
  Serial.println("LON: " + String(selfState.lon, 6));
  Serial.println("VEL: " + String(selfState.speedKmh, 1));
  Serial.println("SAT: " + String(selfState.satellites));
  Serial.println("BAT: " + String(selfState.batteryVoltage, 2));

  if (distanceValid) {
    Serial.println("PEER: " + peerState.riderId);
    Serial.println("DIST: " + String(lastDistanceMeters, 1) + " m");
    Serial.println("ALERT: " + getAlertText());
    Serial.println("MSG: " + lastRxMessage);
  } else if (peerIsFresh()) {
    Serial.println("PEER: " + peerState.riderId);
    Serial.println("DIST: NOFIX");
    Serial.println("ALERT: ---");
    Serial.println("MSG: " + lastRxMessage);
  } else {
    Serial.println("PEER: NO DATA");
    Serial.println("ALERT: ---");
  }

  Serial.println("LORA: " + lastLoRaStatus);
  Serial.println("==============================");
}

void print_alert_changes()
{
  if (currentAlertLevel != lastPrintedAlertLevel) {
    switch (currentAlertLevel) {
      case ALERT_NORMAL:
        Serial.println("[ALERT] Estado NORMAL");
        break;
      case ALERT_WARNING:
        Serial.println("[ALERT] WARNING: separacion media");
        break;
      case ALERT_CRITICAL:
        Serial.println("[ALERT] CRITICAL: separacion alta");
        break;
      default:
        break;
    }
    lastPrintedAlertLevel = currentAlertLevel;
  }
}

// ===================== SETUP / LOOP =====================
void setup()
{
  Serial.begin(115200);
  delay(100);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  selfState = {};
  peerState = {};

  init_tft();
  init_gps();
  init_lora();

  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, FW_NAME);
  st7735.st7735_write_str(0, 15, FW_VERSION);
  st7735.st7735_write_str(0, 30, "Node " + String(RIDER_ID));
  delay(1200);

  Serial.println("System init complete");
}

void loop()
{
  read_gps();
  update_self_state();
  handle_lora();
  updateDistanceAndAlerts();
  print_alert_changes();

  unsigned long now = millis();

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 2000) {
    lastPrint = now;
    print_status();
  }

  if (now - lastDisplay >= displayInterval)
  {
    lastDisplay = now;
    update_display();
  }

  if (now - lastSend >= sendInterval)
  {
    lastSend = now;
    state = STATE_TX;
  }
}