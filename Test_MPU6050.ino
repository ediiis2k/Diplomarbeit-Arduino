// Testsketch MPU 6050

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const uint8_t REG_WHO_AM_I     = 0x75;

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2, true);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22); 
  write8(REG_PWR_MGMT_1, 0x00);
  delay(100);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1, true);
  uint8_t who = Wire.read();
  Serial.print("WHO_AM_I: 0x"); Serial.println(who, HEX);
  if (who != 0x68) {
    Serial.println("Warnung: Unerwartete WHO_AM_I Antwort. Verdrahtung und Adresse pruefen.");
  }
  Serial.println("Start Live Readout (Accel in raw LSB, Gyro in raw LSB) ...");
}

void loop() {
  int16_t ax = read16(REG_ACCEL_XOUT_H + 0);
  int16_t ay = read16(REG_ACCEL_XOUT_H + 2);
  int16_t az = read16(REG_ACCEL_XOUT_H + 4);
  int16_t gx = read16(REG_GYRO_XOUT_H + 0);
  int16_t gy = read16(REG_GYRO_XOUT_H + 2);
  int16_t gz = read16(REG_GYRO_XOUT_H + 4);
  Serial.print("ACC [LSB] ax: "); Serial.print(ax);
  Serial.print("  ay: "); Serial.print(ay);
  Serial.print("  az: "); Serial.print(az);
  Serial.print("   |   GYR [LSB] gx: "); Serial.print(gx);
  Serial.print("  gy: "); Serial.print(gy);
  Serial.print("  gz: "); Serial.println(gz);
  delay(150);
}
