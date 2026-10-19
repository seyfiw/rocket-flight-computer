#pragma once
#include <Arduino.h>

// ---------- Pins ----------
constexpr int PIN_SDA         = 21;
constexpr int PIN_SCL         = 22;
constexpr int PIN_SERVO_PITCH = 18;
constexpr int PIN_SERVO_YAW   = 19;
constexpr int PIN_LED         = 2;

// ---------- Loop ----------
constexpr float LOOP_HZ  = 200.0f;
constexpr float LOG_HZ   = 20.0f;

// ---------- Attitude filter ----------
constexpr float FILTER_ALPHA = 0.98f;

// ---------- TVC ----------
constexpr float SERVO_CENTER_PITCH = 90.0f;
constexpr float SERVO_CENTER_YAW   = 90.0f;
constexpr float GIMBAL_LIMIT       = 8.0f;   // degrees
constexpr bool  INVERT_PITCH       = false;  // flip if a servo moves the wrong way
constexpr bool  INVERT_YAW         = false;

// ---------- PID (not tuned yet) ----------
constexpr float PID_KP = 0.6f;
constexpr float PID_KI = 0.05f;
constexpr float PID_KD = 0.12f;

// ---------- Flight detection ----------
constexpr float LAUNCH_ACCEL_G     = 2.0f;    // above this = motor ignition
constexpr uint32_t LAUNCH_HOLD_MS  = 50;
constexpr float BURNOUT_ACCEL_G    = 0.5f;    // below this = motor burnout
constexpr uint32_t BURNOUT_HOLD_MS = 100;
constexpr uint32_t MAX_BURN_MS     = 4000;
constexpr uint32_t APOGEE_DELAY_MS = 5000;    // TODO: use barometer instead
constexpr float LANDED_GYRO_DPS    = 5.0f;
constexpr uint32_t LANDED_HOLD_MS  = 2000;
