#pragma once

class Pid {
 public:
  Pid(float kp, float ki, float kd, float outLimit)
      : kp_(kp), ki_(ki), kd_(kd), limit_(outLimit) {}
  
float update(float setpoint, float measurement, float dt) {
    if (dt <= 0) return 0;
    float error = setpoint - measurement;

    float deriv = first_ ? 0 : -(measurement - prevMeas_) / dt;
    prevMeas_ = measurement;
    first_ = false;

    float out = kp_ * error + integral_ + kd_ * deriv;

    if (out < limit_ && out > -limit_) {
      integral_ += ki_ * error * dt;
    }

    if (out > limit_) out = limit_;
    if (out < -limit_) out = -limit_;
    return out;
  }

  void reset() {
    integral_ = 0;
    first_ = true;
  }

 private:
  float kp_, ki_, kd_, limit_;
  float integral_ = 0;
  float prevMeas_ = 0;
  bool first_ = true;
};
