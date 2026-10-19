// servo vector control
#pragma once
#include <ESP32Servo.h>
#include "config.h"

class Tvc {
 public:
  void begin() {
    pitch_.attach(PIN_SERVO_PITCH);
    yaw_.attach(PIN_SERVO_YAW);
    center();
  }

 
  void set(float pitchDeg, float yawDeg) {
    pitchDeg = constrain(pitchDeg, -GIMBAL_LIMIT, GIMBAL_LIMIT);
    yawDeg   = constrain(yawDeg,   -GIMBAL_LIMIT, GIMBAL_LIMIT);
    if (INVERT_PITCH) pitchDeg = -pitchDeg;
    if (INVERT_YAW)   yawDeg   = -yawDeg;
    cmdPitch_ = pitchDeg;
    cmdYaw_   = yawDeg;
    pitch_.write(SERVO_CENTER_PITCH + pitchDeg);
    yaw_.write(SERVO_CENTER_YAW + yawDeg);
  }

  void center() { set(0, 0); }

  float cmdPitch() const { return cmdPitch_; }
  float cmdYaw() const { return cmdYaw_; }

 private:
  Servo pitch_, yaw_;
  float cmdPitch_ = 0, cmdYaw_ = 0;
};
