#pragma once
#include <Arduino.h>
#include "imu.h"

enum class State : uint8_t {
  IDLE,      // powered on, waiting for arm command
  ARMED,     // ready, waiting for motor ignition
  POWERED,   // motor burning -> TVC active
  COAST,     // motor out, coasting up
  DESCENT,   // past apogee -> deploy parachute
  LANDED,
  TEST       // bench test: TVC always active
};

const char *stateName(State s);

class FlightState {
 public:
  void update(const ImuData &d, uint32_t nowMs);
  void set(State s, uint32_t nowMs);

  State get() const { return state_; }
  bool tvcActive() const { return state_ == State::POWERED || state_ == State::TEST; }
  bool accelReliable() const { return state_ != State::POWERED && state_ != State::COAST; }

 private:
  // true if cond has been true continuously for holdMs
  bool held(bool cond, uint32_t nowMs, uint32_t holdMs);

  State state_ = State::IDLE;
  uint32_t enteredMs_ = 0;
  uint32_t condSinceMs_ = 0;
  bool condActive_ = false;
};
