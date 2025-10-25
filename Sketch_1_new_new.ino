// Sketch_1_ADS1115
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADS1115 Einstellungen
Adafruit_ADS1115 ads;
constexpr int SDA_PIN = 21, SCL_PIN = 22;
constexpr uint8_t ADS_ADDR = 0x48;
constexpr uint8_t ADS_CH   = 0;
constexpr float R1 = 100000.0f;
constexpr float R2 = 47000.0f;
constexpr float DIVIDER = (R1 + R2) / R2;
constexpr float VBAT_CAL = 7.14 / 7.08;

// ADS1115
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

// Setup / Loop
void setup(){
  Serial.begin(115200); delay(120);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  if (!ads.begin(ADS_ADDR)) {
    Serial.println("ADS1115 nicht gefunden!");
  } else {
    ads.setGain(GAIN_ONE);
    Serial.println("ADS1115 Start");
  }
}

void loop(){
  float v=0, p=0;
  if (readBattery(v,p)) Serial.printf("Vbat=%.2f V  SoC=%.1f %%\n", v, p);
  else Serial.println("Battery read failed");
  delay(1000);
}
