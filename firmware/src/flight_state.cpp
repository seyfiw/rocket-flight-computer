#include "flight_state.h"
#include "config.h"

const char *stateName(State s) {
  switch (s) {
    case State::IDLE:    return "IDLE";
    case State::ARMED:   return "ARMED";
    case State::POWERED: return "POWERED";
    case State::COAST:   return "COAST";
    case State::DESCENT: return "DESCENT";
    case State::LANDED:  return "LANDED";
    case State::TEST:    return "TEST";
  }
  return "?";
}

void FlightState::set(State s, uint32_t nowMs) {
  if (s == state_) return;
  state_ = s;
  enteredMs_ = nowMs;
  condActive_ = false;
  Serial.printf("# STATE -> %s\n", stateName(s));
}

bool FlightState::held(bool cond, uint32_t nowMs, uint32_t holdMs) {
  if (!cond) {
    condActive_ = false;
    return false;
  }
  if (!condActive_) {
    condActive_ = true;
    condSinceMs_ = nowMs;
  }
  return nowMs - condSinceMs_ >= holdMs;
}

void FlightState::update(const ImuData &d, uint32_t nowMs) {
  const float g = d.accelMag();
  const uint32_t inState = nowMs - enteredMs_;

  switch (state_) {
    case State::ARMED:
      if (held(g > LAUNCH_ACCEL_G, nowMs, LAUNCH_HOLD_MS)) set(State::POWERED, nowMs);
      break;

    case State::POWERED:
      if (held(g < BURNOUT_ACCEL_G, nowMs, BURNOUT_HOLD_MS) || inState > MAX_BURN_MS)
        set(State::COAST, nowMs);
      break;

    case State::COAST:
      if (inState > APOGEE_DELAY_MS) set(State::DESCENT, nowMs);
      break;

    case State::DESCENT:
      if (held(d.gyroMag() < LANDED_GYRO_DPS && fabsf(g - 1.0f) < 0.1f, nowMs, LANDED_HOLD_MS))
        set(State::LANDED, nowMs);
      break;

    default:
      break;  // IDLE, LANDED, TEST change only via Serial commands
  }
}
