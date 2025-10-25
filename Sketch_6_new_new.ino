// Sketch_6_SLEEP_WAKE.ino
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>
#include <TinyGPSPlus.h>
#include <esp_sleep.h>

constexpr int     SDA_PIN  = 21, SCL_PIN = 22; // ADS1115 und MPU6050

// ADS1115
Adafruit_ADS1115 ads;
constexpr uint8_t ADS_ADDR = 0x48;
constexpr uint8_t ADS_CH   = 0;
constexpr float   R1       = 100000.0f;
constexpr float   R2       = 47000.0f;
constexpr float   DIVIDER  = (R1 + R2) / R2;
constexpr float   VBAT_CAL = 7.14 / 7.10;

// MPU6050
constexpr int MPU_INT_PIN = 33;              // INT vom MPU
constexpr uint8_t MPU_ADDR = 0x68;
constexpr float ACC_LSB   = 16384.0f; // ±2g
constexpr float GY_LSB    = 131.0f;
constexpr float A_DELTA_G = 0.06f;   // 
constexpr float G_DPS_THR = 8.0f;
constexpr uint32_t WINDOW_MS = 2000;
constexpr uint32_t SAMPLE_MS = 1000;
constexpr int MIN_HITS = 2;
constexpr uint32_t SMS_COOLDOWN_MS = 60000;

// SIM7600
HardwareSerial LTE(1);
constexpr int LTE_RX = 26;     
constexpr int LTE_TX = 27;     
constexpr long LTE_BAUD = 115200;
constexpr int DTR_PIN = 25;    
const char* PHONE = "+41764419599";
uint32_t lastSmsMs = 0;

// NEO-M8N
TinyGPSPlus gps;
constexpr int GPS_RX = 17;         // ESP32 RX  an TX vom GPS
constexpr int GPS_TX = 16;         // ESP32 TX  an RX vom GPS
constexpr long GPS_BAUD = 9600;
constexpr uint32_t GPS_FIX_TIMEOUT = 90000;  // 90 s max warten
constexpr uint32_t GPS_MIN_VALID   = 2000;   // 2 s stabile Daten vor OK

// PIEZO
constexpr int PIEZO_PIN = 18;
constexpr uint32_t SIREN_MS = 2000; // Dauer Sirene vor SMS

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
bool mpuWrite(uint8_t r, uint8_t v){
  Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
  return Wire.endTransmission()==0;
}
uint8_t mpuRead8(uint8_t r){
  Wire.beginTransmission(MPU_ADDR); Wire.write(r);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,(uint8_t)1);
  return Wire.read();
}

bool mpuInitMotion(){
  if (!mpuWrite(0x6B, 0x00)) return false;    
  mpuWrite(0x6A, 0x00);                    
  mpuWrite(0x1A, 0x06);             
  mpuWrite(0x1C, 0b00000100);            
  const uint8_t MOT_THR = 4;            
  const uint8_t MOT_DUR = 20;             
  mpuWrite(0x1F, MOT_THR);                 
  mpuWrite(0x20, MOT_DUR);                  
  mpuWrite(0x69, 0b00010000);               
  mpuWrite(0x37, 0b00110000);                
  mpuWrite(0x38, 0b01000000);               
  (void)mpuRead8(0x3A);                   
  return (mpuRead8(0x75) == 0x68);
}

void readAccelGyro(float &a_g, float &g_dps){
  int16_t ax,ay,az,gx,gy,gz;
  // Accel
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,(uint8_t)6);
  ax=(Wire.read()<<8)|Wire.read(); ay=(Wire.read()<<8)|Wire.read(); az=(Wire.read()<<8)|Wire.read();
  // Gyro
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,(uint8_t)6);
  gx=(Wire.read()<<8)|Wire.read(); gy=(Wire.read()<<8)|Wire.read(); gz=(Wire.read()<<8)|Wire.read();

  a_g   = sqrtf((float)ax*ax+(float)ay*ay+(float)az*az)/ACC_LSB;
  g_dps = sqrtf((float)gx*gx+(float)gy*gy+(float)gz*gz)/GY_LSB;
}

// SIM7600 Funktion
String readUntil(const char* needle, uint32_t timeout_ms=10000){
  String buf; uint32_t t0=millis(); size_t n=strlen(needle);
  while(millis()-t0<timeout_ms){
    while(LTE.available()){
      char c=LTE.read(); buf+=c;
      if(buf.length()>=n && buf.endsWith(needle)) return buf;
    }
    delay(1);
  }
  return buf;
}
inline void at(const char* cmd){ LTE.print(cmd); LTE.print("\r\n"); }
bool atOK(uint32_t to=5000){ String r=readUntil("OK\r\n",to); return r.indexOf("OK")>=0; }
bool waitCEREG(uint32_t to=30000){
  uint32_t t0=millis();
  while(millis()-t0<to){
    at("AT+CEREG?"); String r=readUntil("OK\r\n",1500);
    if(r.indexOf("+CEREG: 0,1")>=0 || r.indexOf("+CEREG: 0,5")>=0) return true;
    delay(800);
  }
  return false;
}
bool sendSMS(const String& text, const char* phone){
  while(LTE.available()) LTE.read(); // Puffer leeren
  at("AT");               if(!atOK(3000)) return false;
  at("ATE0");             atOK();
  at("AT+CMEE=2");        atOK();
  at("AT+CPIN?");         if(readUntil("OK\r\n",5000).indexOf("READY")<0) return false;
  if(!waitCEREG())        return false;
  at("AT+CSCS=\"GSM\"");  atOK();
  at("AT+CMGF=1");        atOK();
  at("AT+CSMP=17,167,0,0"); atOK();

  String cmd = String("AT+CMGS=\"") + phone + "\"";
  at(cmd.c_str());
  if(readUntil(">",12000).indexOf('>')<0) return false;

  LTE.print(text);
  LTE.write(26); // Ctrl+Z

  String resp = readUntil("OK\r\n",30000);
  return (resp.indexOf("+CMGS:")>=0 && resp.indexOf("OK")>=0);
}

// NEO-M8N GPS: Fix holen 
bool getGpsFix(double &lat, double &lon, uint32_t timeout_ms = GPS_FIX_TIMEOUT){
  uint32_t t0 = millis(), validSince = 0;
  while(millis()-t0 < timeout_ms){
    while(Serial2.available()) gps.encode(Serial2.read());
    if(gps.location.isValid()){
      if(!validSince) validSince = millis();
      if(millis()-validSince >= GPS_MIN_VALID){
        lat = gps.location.lat();
        lon = gps.location.lng();
        return true;
      }
    }
    delay(10);
  }
  return false;
}

// Sirene
void siren(uint32_t ms){
  if(ms==0) return;
  uint32_t t0=millis();
  while(millis()-t0<ms){
    for(int f=1800; f<=3200; f+=80){ tone(PIEZO_PIN,f); delay(12); }
    noTone(PIEZO_PIN); delay(25);
    for(int f=3200; f>=1800; f-=80){ tone(PIEZO_PIN,f); delay(12); }
    noTone(PIEZO_PIN); delay(25);
  }
}

// Bewegung MPU6050
bool confirmMotion(uint32_t window_ms = WINDOW_MS, int min_hits = MIN_HITS){
  uint32_t start=millis(), nextTick=start;
  int hits=0, samples=0;
  while(millis()-start < window_ms){
    if(millis() < nextTick) delay(nextTick - millis());
    nextTick += SAMPLE_MS;
    samples++;

    float a,g; readAccelGyro(a,g);
    bool moving = (fabsf(a-1.0f) > A_DELTA_G) || (g > G_DPS_THR);
    if(moving) hits++;

    Serial.printf("a|g: %.3f g | %.1f dps -> %s (hits %d/%d)\n", a, g, moving?"MOVE":"idle", hits, samples);
  }
  return hits >= min_hits;
}

// Modem Schlaf/Wach 
void simSleep(bool enable){
  digitalWrite(DTR_PIN, enable ? HIGH : LOW); // HIGH=Sleep, LOW=Wach
  delay(50);
}

// Deep-Sleep vorbereiten
void goDeepSleep(){
  (void)mpuRead8(0x3A);  
  esp_sleep_enable_ext1_wakeup(1ULL << MPU_INT_PIN, ESP_EXT1_WAKEUP_ANY_HIGH); 
  simSleep(true);         
  Serial.flush(); delay(40);
  esp_deep_sleep_start();
}

// Setup
void setup(){
  Serial.begin(115200); delay(120);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);        
  pinMode(DTR_PIN, OUTPUT); digitalWrite(DTR_PIN, HIGH); 

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  LTE.begin(LTE_BAUD, SERIAL_8N1, LTE_RX, LTE_TX);

  if(!mpuInitMotion()) Serial.println("MPU motion init failed");

  if (!ads.begin(ADS_ADDR)) {
    Serial.println("ADS1115 nicht gefunden!");
  } else {
    ads.setGain(GAIN_ONE);
    Serial.println("ADS1115 Start");
  }

  // Wake-Ursache prüfen
  auto cause = esp_sleep_get_wakeup_cause();
  if(cause == ESP_SLEEP_WAKEUP_EXT1){
    Serial.println("Wake durch MPU-INT");
    //  filtern
    if(!confirmMotion()){
      Serial.println("False positive -> zurück in Sleep");
      goDeepSleep(); return;
    }
    // Modem aufwecken
    simSleep(false);
    // Sirene
    siren(SIREN_MS);
    // Akku
    float v=0,p=0; readBattery(v,p);
    // GPS
    double lat=0, lon=0;
    bool fix = getGpsFix(lat, lon, GPS_FIX_TIMEOUT);
    // SMS
    String msg = "ALARM: Bewegung erkannt!\r\nBatterie=" + String(p,0) + "% (" + String(v,2) + "V)\r\n";
    if(fix){
      msg += "https://maps.google.com/?q=" + String(lat,6) + "," + String(lon,6);
    }else{
      msg += "Kein GPS-Fix.";
    }
    Serial.println(">>> Sende SMS …");
    bool ok = sendSMS(msg, PHONE);
    Serial.println(ok ? "SMS erfolgreich gesendet." : "SMS Senden fehlgeschlagen.");
    // Wieder schlafen
    goDeepSleep(); return;
  }
  Serial.println("Power-On -> Deep-Sleep, warte auf Bewegung …");
  goDeepSleep();
}

// Loop
void loop(){ /* wird im Deep-Sleep-Design nicht genutzt */ }
