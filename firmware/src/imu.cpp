#include "imu.h"
#include <Wire.h>

namespace {
constexpr uint8_t ADDR         = 0x68;
constexpr uint8_t REG_PWR      = 0x6B;
constexpr uint8_t REG_GYRO_CFG = 0x1B;
constexpr uint8_t REG_ACC_CFG  = 0x1C;
constexpr uint8_t REG_DATA     = 0x3B;


constexpr uint8_t ACC_CFG_8G    = 0x10;
constexpr uint8_t GYRO_CFG_500  = 0x08;
constexpr float   ACCEL_LSB     = 4096.0f;
constexpr float   GYRO_LSB      = 65.5f;
}  // namespace

void Imu::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool Imu::begin(int sda, int scl) {
  Wire.begin(sda, scl);
  Wire.setClock(400000);

  Wire.beginTransmission(ADDR);
  if (Wire.endTransmission() != 0) return false;

  writeReg(REG_PWR, 0x00);  
  delay(100);
  writeReg(REG_ACC_CFG, ACC_CFG_8G);
  writeReg(REG_GYRO_CFG, GYRO_CFG_500);
  delay(10);
  return true;
}

bool Imu::readRaw(ImuData &d) {
  Wire.beginTransmission(ADDR);
  Wire.write(REG_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADDR, (uint8_t)14) != 14) return false;

  auto next16 = []() -> int16_t {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (hi << 8) | lo;
  };

  d.ax = next16() / ACCEL_LSB;
  d.ay = next16() / ACCEL_LSB;
  d.az = next16() / ACCEL_LSB;
  next16();  // temp
  d.gx = next16() / GYRO_LSB;
  d.gy = next16() / GYRO_LSB;
  d.gz = next16() / GYRO_LSB;
  return true;
}

bool Imu::read(ImuData &d) {
  if (!readRaw(d)) return false;
  d.gx -= biasX_;
  d.gy -= biasY_;
  d.gz -= biasZ_;
  return true;
}

bool Imu::calibrate(int samples) {
  float sx = 0, sy = 0, sz = 0;
  int ok = 0;
  ImuData d;
  for (int i = 0; i < samples; i++) {
    if (readRaw(d)) { sx += d.gx; sy += d.gy; sz += d.gz; ok++; }
    delay(2);
  }
  if (ok < samples / 2) return false;
  biasX_ = sx / ok;
  biasY_ = sy / ok;
  biasZ_ = sz / ok;
  return true;
}
