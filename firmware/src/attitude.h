#pragma once
#include "imu.h"

// Complementary filter: trust gyro short-term, accel corrects drift long-term
class Attitude {
 public:
  explicit Attitude(float alpha) : alpha_(alpha) {}

  void reset(const ImuData &d) {
    accelAngles(d, pitch_, roll_);
  }

  // freezeAccel = true while accelerating: accel no longer points at gravity
  void update(const ImuData &d, float dt, bool freezeAccel = false) {
    pitch_ += d.gy * dt;
    roll_  += d.gx * dt;
    if (freezeAccel) return;

    float accPitch, accRoll;
    accelAngles(d, accPitch, accRoll);
    pitch_ = alpha_ * pitch_ + (1.0f - alpha_) * accPitch;
    roll_  = alpha_ * roll_  + (1.0f - alpha_) * accRoll;
  }

  float pitch() const { return pitch_; }
  float roll() const { return roll_; }

 private:
  static void accelAngles(const ImuData &d, float &p, float &r) {
    p = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * RAD_TO_DEG;
    r = atan2f(d.ay, d.az) * RAD_TO_DEG;
  }

  float alpha_;
  float pitch_ = 0, roll_ = 0;
};
