// Sketch 7
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>
#include <TinyGPSPlus.h>
#include <esp_sleep.h>

constexpr int SDA_PIN = 21, SCL_PIN = 22;

// ---------------- Hardware ----------------
HardwareSerial LTE(1);
TinyGPSPlus gps;
Adafruit_ADS1115 ads;

// ADS1115
constexpr uint8_t ADS_ADDR = 0x48;
constexpr uint8_t ADS_CH   = 0;
constexpr float   R1       = 100000.0f;
constexpr float   R2       = 47000.0f;
constexpr float   DIVIDER  = (R1 + R2) / R2;
constexpr float   VBAT_CAL = 7.14 / 7.10;
constexpr float LOW_BATT_V       = 6.80f;
constexpr float LOW_BATT_CLEAR_V = 7.20f;

// MPU6050
constexpr int MPU_INT_PIN = 33;
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint32_t CONT_MS            = 2000;   // Prüffenster
constexpr uint16_t CONT_SAMPLE_MS     = 25;     // ~40 Hz
constexpr uint32_t IGNORE_MS          = 1800;   // Stoß-Abklingen ignorieren
constexpr float    LIN_THR_G          = 0.030f; // |a|-Abweichung von 1 g (aktiv)
constexpr float    GYRO_THR_DPS       = 6.5f;   // Gyro-Magnituden-Schwelle (aktiv)
constexpr float    A_RMS_G            = 0.022f; // RMS-Energie über das Fenster
constexpr uint32_t CONFIRM_WIN_MS     = CONT_MS;
constexpr int      CONFIRM_MIN_HITS   = 4;
constexpr float    A_JERK_G           = 0.055f; // Δ|a| zwischen zwei Samples (g)
constexpr float    A_LIN_G            = 0.030f; // |a|-Abweichung zur Basis (g)
constexpr float    GYRO_DPS           = 6.5f;   // °/s
constexpr uint8_t MOT_THR = 4;
constexpr uint8_t MOT_DUR = 20;

// SIM7600
constexpr int LTE_RX = 26, LTE_TX = 27;  constexpr long LTE_BAUD = 115200;
constexpr int DTR_PIN = 25;              // SIM7600 DTR (HIGH = darf schlafen)
constexpr int RI_PIN  = 32;              // RI -> EXT0 (LOW)
const char* PHONE = "+41764419599";
uint32_t lastSmsMs = 0;
constexpr uint32_t TRACK_PERIOD_MS = 60000; // alle 60 s Standort-SMS

// NEO-M8N
constexpr int GPS_RX = 17, GPS_TX = 16;  constexpr long GPS_BAUD = 9600;
constexpr uint32_t GPS_FIX_TIMEOUT = 90000;
constexpr uint32_t GPS_MIN_VALID  = 2000;

// PIEZO
constexpr int PIEZO_PIN = 18;
constexpr uint32_t SIREN_MS = 2000;                // 0 = Sirene aus

// Zustände (Deep-Sleep-persistent)
RTC_DATA_ATTR bool ARMED = true;
RTC_DATA_ATTR bool LOW_BATT_NOTIFIED = false;
RTC_DATA_ATTR bool TRACKING = false;   // NEU: true, sobald nach einem Alarm Tracking laufen soll

// ADS1115 Funktion
static float socFromVcell(float vcell){
  if (vcell <= 3.30f) return 5.0f;
  if (vcell >= 4.15f) return 100.0f;
  return (vcell - 3.30f) * (95.0f / (4.15f - 3.30f)) + 5.0f;
}

bool readBattery(float &vbat, float &soc){
  long acc = 0;
  for(int i=0;i<8;i++){ acc += ads.readADC_SingleEnded(ADS_CH); delay(2); }
  int16_t raw = (int16_t)(acc/8);
  const float LSB_V = 0.000125f;
  float vadc = raw * LSB_V;
  vbat = vadc * DIVIDER * VBAT_CAL;
  float vcell = vbat * 0.5f;
  soc = socFromVcell(vcell);
  return true;
}

// MPU6050 Funktion
bool mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission()==0;
}
uint8_t mpuRead8(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,(uint8_t)1);
  return Wire.read();
}
bool mpuInitMotion(){
  if (!mpuWrite(0x6B, 0x00)) return false;   // PWR_MGMT_1: wake
  mpuWrite(0x6A, 0x00);                      // USER_CTRL
  mpuWrite(0x1A, 0x06);                      // DLPF ~2.6 Hz (robust gg. Vibration)
  mpuWrite(0x1C, 0b00000100);                // ACCEL HPF ~0.63 Hz
  mpuWrite(0x1F, MOT_THR);                   // MOT_THR
  mpuWrite(0x20, MOT_DUR);                   // MOT_DUR (ms)
  mpuWrite(0x69, 0b00010000);                // MOT_DETECT_CTRL
  mpuWrite(0x37, 0b00110000);                // INT_PIN_CFG (latch/clear)
  mpuWrite(0x38, 0b01000000);                // INT_ENABLE (MOT_EN)
  return (mpuRead8(0x75)==0x68);             // WHO_AM_I
}

// SIM7600 Funktion
String readUntil(const char* needle, uint32_t timeoutMs=10000){
  String buf; uint32_t t0=millis(); size_t n=strlen(needle);
  while (millis()-t0<timeoutMs){
    while (LTE.available()){
      char c = LTE.read(); buf += c;
      if (buf.length()>=n && buf.endsWith(needle)) return buf;
    }
    delay(1);
  }
  return buf;
}
inline void at(const char* cmd){ LTE.print(cmd); LTE.print("\r\n"); }
bool atOK(uint32_t to=5000){ String r=readUntil("OK\r\n",to); return r.indexOf("OK")>=0; }
bool waitCEREG(uint32_t to=30000){
  uint32_t t0=millis();
  while (millis()-t0<to){
    at("AT+CEREG?"); String r=readUntil("OK\r\n",1500);
    if (r.indexOf("+CEREG: 0,1")>=0 || r.indexOf("+CEREG: 0,5")>=0) return true;
    delay(800);
  }
  return false;
}
bool sendSMS(const String& text, const char* phone){
  at("AT"); if (!atOK()) return false;
  at("ATE0"); atOK();
  at("AT+CMEE=2"); atOK();
  at("AT+CPIN?"); if (!atOK()) return false;
  if (!waitCEREG()) return false;
  at("AT+CSCS=\"GSM\""); atOK();
  at("AT+CMGF=1"); atOK();
  at("AT+CSMP=17,167,0,0"); atOK();

  String cmd = String("AT+CMGS=\"") + phone + "\"";
  at(cmd.c_str());
  if (readUntil(">", 12000).indexOf('>') < 0) return false;

  LTE.print(text); LTE.write(26);                // Ctrl+Z
  String resp = readUntil("OK\r\n", 30000);
  return (resp.indexOf("+CMGS:")>=0 && resp.indexOf("OK")>=0);
}
void simSleep(bool enable){
  if (enable){
    // Auto-Sleep aktivieren und Modem schlafen lassen
    at("AT+CSCLK=1"); atOK();   // Slow-Clock erlaubt (nur wirksam bei DTR=HIGH)
    digitalWrite(DTR_PIN, HIGH);  // HIGH = darf schlafen
    delay(50);
  } else {
    // Aufwecken: DTR LOW und kurz antippen
    digitalWrite(DTR_PIN, LOW);   // LOW = aktiv
    delay(50);
    at("AT"); readUntil("OK\r\n", 1000); // Wake-Poke
  }
}

// NEO-M8N GPS: Fix holen 
bool getGpsFix(double &lat, double &lon, uint32_t timeoutMs=GPS_FIX_TIMEOUT){
  uint32_t t0=millis(), validSince=0;
  while (millis()-t0<timeoutMs){
    while (Serial2.available()) gps.encode(Serial2.read());
    if (gps.location.isValid()){
      if (!validSince) validSince=millis();
      if (millis()-validSince>=GPS_MIN_VALID){ lat=gps.location.lat(); lon=gps.location.lng(); return true; }
    }
    delay(10);
  }
  return false;
}

// SMS nur exakt "ON"/"OFF" , Speicher Cleanup
bool handleSmsCommands(){
  at("AT"); atOK();
  at("ATE0"); atOK();
  at("AT+CMEE=2"); atOK();
  at("AT+CSCS=\"GSM\""); atOK();
  at("AT+CMGF=1"); atOK();
  at("AT+CFGRI=1"); atOK(); // RI-Puls sicherstellen
  at("AT+CPMS=\"ME\",\"ME\",\"ME\""); if(!atOK()){ at("AT+CPMS=\"SM\",\"SM\",\"SM\""); atOK(); }

  // Ungelesene lesen
  at("AT+CMGL=\"REC UNREAD\"");
  String resp = readUntil("OK\r\n",6000);
  if (resp.length()==0) return false;
  String U = resp; U.toUpperCase();
  U.replace("\nON\n","\r\nON\r\n"); U.replace("\nOFF\n","\r\nOFF\r\n"); 
  int posON  = U.lastIndexOf("\r\nON\r\n");
  int posOFF = U.lastIndexOf("\r\nOFF\r\n");
  bool found=false, activate=ARMED;
  if (posON>=0 || posOFF>=0){ activate = (posON>posOFF); found=true; }
  at("AT+CMGD=1,4"); atOK();      // Speicher löschen
  if (found){
    ARMED = activate;
    String ack = activate
      ? "Ueberwachung aktiviert!\r\nZum Deaktivieren mit \"OFF\" antworten."
      : "Ueberwachung deaktiviert!\r\nZum Aktivieren mit \"ON\" antworten.";
    bool ok = sendSMS(ack, PHONE);
    if (!ok){ delay(1500); sendSMS(ack, PHONE); }
  }

  return found;
}

// Bewegung MPU6050
bool confirmMotion(uint32_t window_ms, int min_hits, int /*unused*/){
  const float ACC_LSB_PER_G    = 16384.0f; // ±2g
  const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps

  auto readMagnitudes = [&](float &a_g, float &g_dps){
    int16_t ax,ay,az,gx,gy,gz;
    // Accel
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR,(uint8_t)6);
    ax=(Wire.read()<<8)|Wire.read(); ay=(Wire.read()<<8)|Wire.read(); az=(Wire.read()<<8)|Wire.read();
    // Gyro
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR,(uint8_t)6);
    gx=(Wire.read()<<8)|Wire.read(); gy=(Wire.read()<<8)|Wire.read(); gz=(Wire.read()<<8)|Wire.read();
    // Euclid-Normen (lageunabhängig)
    a_g   = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az) / ACC_LSB_PER_G;
    g_dps = sqrtf((float)gx*gx + (float)gy*gy + (float)gz*gz) / GYRO_LSB_PER_DPS;
  };

  float a_last=0, g_last=0; readMagnitudes(a_last, g_last);
  float a_base = a_last;             // langsame Basis (≈1 g)
  int   hits   = 0;
  float rms_acc = 0.0f; int rms_n=0;
  const uint32_t start = millis();
  uint32_t nextTick = start;

  while (true){
    // festen Takt halten
    uint32_t now = millis();
    if (now < nextTick) delay(nextTick - now);
    nextTick += CONT_SAMPLE_MS;

    float a,g; readMagnitudes(a,g);

    // Ereignisse: Jerk, lineare Abweichung, Rotation
    bool jerkHit = fabsf(a - a_last) > A_JERK_G;
    bool linHit  = fabsf(a - a_base) > A_LIN_G;
    bool gyroHit = g > GYRO_DPS;

    uint32_t elapsed = millis() - start;
    if (elapsed >= IGNORE_MS){
      if (jerkHit || linHit || gyroHit) hits++;
      float d = a - a_base; rms_acc += d*d; rms_n++;
    }

    // Basis langsam nachführen (~1/16)
    a_base += (a - a_base) * (1.0f/16.0f);
    a_last = a;

    if (elapsed >= window_ms) break;
  }

  float a_rms = (rms_n>0) ? sqrtf(rms_acc / rms_n) : 0.0f;
  return (hits >= min_hits) && (a_rms >= A_RMS_G);
}

// Sirene, Alarm, Low-Battery
void siren(uint32_t ms){
  if (ms==0) return;
  uint32_t t0=millis();
  while (millis()-t0<ms){
    for (int f=1800; f<=3200; f+=80){ tone(PIEZO_PIN,f); delay(12); }
    noTone(PIEZO_PIN); delay(25);
    for (int f=3200; f>=1800; f-=80){ tone(PIEZO_PIN,f); delay(12); }
    noTone(PIEZO_PIN); delay(25);
  }
}
void checkLowBatteryAndNotify(){
  float v=0,s=0; if(!readBattery(v,s)) return;
  if(!LOW_BATT_NOTIFIED && v>0 && v<=LOW_BATT_V){
    String m = "Batterie niedrig\n" + String(v,2) + "V (" + String(s,0) + "%)";
    bool ok = sendSMS(m, PHONE); if(!ok){ delay(1500); sendSMS(m, PHONE); }
    LOW_BATT_NOTIFIED = true;
  } else if (LOW_BATT_NOTIFIED && v>=LOW_BATT_CLEAR_V){
    LOW_BATT_NOTIFIED = false;
  }
}
void alarmSequence(){
  if (SIREN_MS>0){ Serial.println("[ALARM] Sirene …"); siren(SIREN_MS); }

  float vbat=0,soc=0; (void)readBattery(vbat,soc);
  double lat=0,lon=0; bool fix=getGpsFix(lat,lon,GPS_FIX_TIMEOUT);

  String msg; msg.reserve(220);
  if (fix){
    msg  = "!! ALARM !!\r\n";
    msg += "Bewegung erkannt!\r\n";
    msg += "Batteriezustand=" + String(soc,0) + "%\r\n";
    msg += "https://maps.google.com/?q=" + String(lat,6) + "," + String(lon,6);
  } else {
    msg  = "!! ALARM !!\r\n";
    msg += "Bewegung erkannt!\r\n";
    msg += "Batteriezustand=" + String(soc,0) + "%\r\n";
    msg += "Kein GPS-Fix.";
  }

  Serial.println("[SMS] Sende …");
  bool ok = sendSMS(msg, PHONE); if(!ok){ delay(1500); ok=sendSMS(msg, PHONE); }
  Serial.println(ok? "[SMS] OK" : "[SMS] Fehler");

  checkLowBatteryAndNotify();
}

// Deep-Sleep
void goDeepSleep(){
  (void)mpuRead8(0x3A);  // INT_STATUS quittieren
  esp_sleep_enable_ext0_wakeup((gpio_num_t)RI_PIN, 0);                          // RI -> LOW
  if (!TRACKING){
    esp_sleep_enable_ext1_wakeup(1ULL<<MPU_INT_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);}    // MPU INT -> HIGH
  if (TRACKING && ARMED){
    esp_sleep_enable_timer_wakeup(TRACK_PERIOD_MS * 1000ULL);
  }
  simSleep(true);
  Serial.flush(); delay(40);
  esp_deep_sleep_start();
}


// Setup
void setup(){
  Serial.begin(115200); delay(120);
  Serial.println("\n=== Bike-Alarm — Cleaned + SMS-Tracking ===");

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  pinMode(RI_PIN, INPUT);
  pinMode(DTR_PIN, OUTPUT);
  digitalWrite(DTR_PIN, HIGH);              // SIM schlafen lassen

  if (!ads.begin(ADS_ADDR)) {
    Serial.println("ADS1115 nicht gefunden!");
  } else {
    ads.setGain(GAIN_ONE);
    Serial.println("ADS1115 Start");
  }

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  LTE.begin(LTE_BAUD, SERIAL_8N1, LTE_RX, LTE_TX);
  at("AT+CFGRI=1"); atOK();
  (void)mpuInitMotion();

  auto cause = esp_sleep_get_wakeup_cause();
  // SMS-Wake
  if (cause==ESP_SLEEP_WAKEUP_EXT0){
    Serial.println("[BOOT] Wake durch SMS (RI)");
    simSleep(false);
    handleSmsCommands();
    if (!ARMED){
      TRACKING = false;
      Serial.println("[INFO] Tracking deaktiviert (OFF)");
    }
    checkLowBatteryAndNotify();
    goDeepSleep(); return;
  }
  // Bewegungs-Wake
  if (cause==ESP_SLEEP_WAKEUP_EXT1){
    uint64_t wmask=esp_sleep_get_ext1_wakeup_status();
    if (wmask & (1ULL<<MPU_INT_PIN)){
      if (!ARMED){ goDeepSleep(); return; }
      if (!confirmMotion(CONFIRM_WIN_MS, CONFIRM_MIN_HITS, 0)){
        goDeepSleep(); return;
      }
      simSleep(false);        // LTE Modem aufwecken
      alarmSequence();        // Alarmablauf starten
      TRACKING = true;        // GPS Tracking aktivieren
      goDeepSleep(); return;
    }
  }
  // Periodisches Wake für SMS-Tracking
  if (cause == ESP_SLEEP_WAKEUP_TIMER){
    Serial.println("[BOOT] Wake durch Timer (Tracking)");
    if (!TRACKING || !ARMED){ goDeepSleep(); return; }
    simSleep(false);
    // Kurzer Fix-Versuch, sonst "kein Fix"
    double lat=0, lon=0; bool fix=getGpsFix(lat,lon,30000);
    String msg;
    if (fix){
      msg  = "Zum Deaktivieren mit \"OFF\" antworten.\r\n";
      msg += "Standort:\r\n";
      msg += "https://maps.google.com/?q=" + String(lat,6) + "," + String(lon,6);
    } else {
      msg  = "Zum Deaktivieren mit \"OFF\" antworten.\r\n";
      msg += "Standort:\r\n";
      msg += "Kein GPS-Fix.";
    }
    sendSMS(msg, PHONE);
    goDeepSleep(); return;
  }

  // Power-On/Reset -> schlafen
  Serial.println("[BOOT] Power-On/Reset");
  goDeepSleep();
}

void loop(){ /* leer */ }
