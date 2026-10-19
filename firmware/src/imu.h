#pragma once
#include <Arduino.h>

struct ImuData {
  float ax, ay, az;  // g
  float gx, gy, gz;  // deg/s, bias removed
  float accelMag() const { return sqrtf(ax * ax + ay * ay + az * az); }
  float gyroMag() const { return sqrtf(gx * gx + gy * gy + gz * gz); }
};

class Imu {
 public:
  bool begin(int sda, int scl);
  bool read(ImuData &d);
  bool calibrate(int samples = 500);  // keep the board still

 private:
  void writeReg(uint8_t reg, uint8_t val);
  bool readRaw(ImuData &d);
  float biasX_ = 0, biasY_ = 0, biasZ_ = 0;
};
