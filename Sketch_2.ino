// Sketch_2_MAX17048_MPU6050.ino
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>

constexpr int     SDA_PIN  = 21, SCL_PIN = 22; // ADS1115 und MPU6050

// ADS1115
Adafruit_ADS1115 ads;
constexpr uint8_t ADS_ADDR = 0x48;
constexpr uint8_t ADS_CH   = 0;
constexpr float   R1       = 100000.0f;
constexpr float   R2       = 47000.0f;
constexpr float   DIVIDER  = (R1 + R2) / R2;
constexpr float   VBAT_CAL = 7.14 / 7.08;

// MPU6050
constexpr uint8_t MPU_ADDR = 0x68;
constexpr float ACC_LSB   = 16384.0f; // ±2g
constexpr float GY_LSB    = 131.0f;
constexpr float A_DELTA_G = 0.030f;   // 
constexpr float G_DPS_THR = 6.5f;

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
bool mpuInit(){
  if(!mpuWrite(0x6B,0x00)) return false; // wake up
  mpuWrite(0x1A,0x06);                   
  mpuWrite(0x1C,0b00000000);             
  return (mpuRead8(0x75)==0x68);         
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

// Setup
void setup(){
  Serial.begin(115200); delay(120);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  
  if (!ads.begin(ADS_ADDR)) {
    Serial.println("ADS1115 nicht gefunden!");
  } else {
    ads.setGain(GAIN_ONE);
    Serial.println("ADS1115 Start");
  }

  if(!mpuInit()) Serial.println("MPU init failed");
  float v=0,p=0;
  if(readBattery(v,p)) Serial.printf("Battery: %.2f V  %.0f %%\n", v, p);
}

// Loop
void loop(){
  float v=0, p=0;
  if(readBattery(v,p)) Serial.printf("Vbat=%.2f V  SoC=%.1f %%\n", v, p);
  else Serial.println("Battery read failed");
  delay(1000);

  float a,g; readAccelGyro(a,g);
  bool moving = (fabsf(a-1.0f) > A_DELTA_G) || (g > G_DPS_THR);
  Serial.printf("a|g: %.3f g | %.1f dps  -> %s\n", a, g, moving?"MOVE":"idle");
  delay(500);
}
