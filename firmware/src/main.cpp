// rocket-flight-computer v0.1 — ESP32 + MPU6050 + 2 servo (TVC)
// MPU6050: SDA 21, SCL 22 | Servo pitch: 18, Servo yaw: 19

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

constexpr int PIN_SDA         = 21;
constexpr int PIN_SCL         = 22;
constexpr int PIN_SERVO_PITCH = 18;
constexpr int PIN_SERVO_YAW   = 19;

constexpr uint8_t MPU_ADDR      = 0x68;
constexpr uint8_t REG_PWR_MGMT1 = 0x6B;
constexpr uint8_t REG_DATA      = 0x3B;
constexpr float   ACCEL_LSB     = 16384.0f;  // ±2 g
constexpr float   GYRO_LSB      = 131.0f;    // ±250 °/s

constexpr float LOOP_HZ      = 200.0f;
constexpr float ALPHA        = 0.98f;  // น้ำหนัก gyro ใน complementary filter
constexpr float SERVO_CENTER = 90.0f;
constexpr float GIMBAL_LIMIT = 15.0f;
constexpr float GAIN         = 1.0f;   // = Kp ในอนาคต

Servo servoPitch, servoYaw;

struct Imu {
  float ax, ay, az;  // g
  float gx, gy, gz;  // °/s
};

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float pitch = 0, roll = 0;
uint32_t lastMicros = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readImu(Imu &d) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

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

// ต้องวางบอร์ดนิ่งตอนคาลิเบรต
void calibrateGyro(int samples = 500) {
  Serial.println("Calibrating gyro... keep still");
  float sx = 0, sy = 0, sz = 0;
  int ok = 0;
  Imu d;
  for (int i = 0; i < samples; i++) {
    if (readImu(d)) { sx += d.gx; sy += d.gy; sz += d.gz; ok++; }
    delay(2);
  }
  if (ok == 0) {
    Serial.println("ERROR: IMU not responding");
    return;
  }
  gyroBiasX = sx / ok;
  gyroBiasY = sy / ok;
  gyroBiasZ = sz / ok;
  Serial.printf("Gyro bias: %.3f %.3f %.3f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void accelAngles(const Imu &d, float &accPitch, float &accRoll) {
  accPitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * RAD_TO_DEG;
  accRoll  = atan2f(d.ay, d.az) * RAD_TO_DEG;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  writeReg(REG_PWR_MGMT1, 0x00);  // ปลุก MPU6050
  delay(100);

  servoPitch.attach(PIN_SERVO_PITCH);
  servoYaw.attach(PIN_SERVO_YAW);
  servoPitch.write(SERVO_CENTER);
  servoYaw.write(SERVO_CENTER);

  calibrateGyro();

  Imu d;
  if (readImu(d)) accelAngles(d, pitch, roll);  // เริ่มจากมุมจริง

  lastMicros = micros();
  Serial.println("pitch,roll,servoPitch,servoYaw");
}

void loop() {
  const uint32_t periodUs = 1000000.0f / LOOP_HZ;
  uint32_t now = micros();
  if (now - lastMicros < periodUs) return;
  float dt = (now - lastMicros) / 1e6f;
  lastMicros = now;

  Imu d;
  if (!readImu(d)) {
    Serial.println("IMU read failed");
    return;
  }

  // complementary filter: ระยะสั้นเชื่อ gyro, ระยะยาวดึงกลับด้วย accel
  float accPitch, accRoll;
  accelAngles(d, accPitch, accRoll);
  pitch = ALPHA * (pitch + (d.gy - gyroBiasY) * dt) + (1.0f - ALPHA) * accPitch;
  roll  = ALPHA * (roll  + (d.gx - gyroBiasX) * dt) + (1.0f - ALPHA) * accRoll;

  // หมุนเครื่องยนต์สวนทางกับมุมเอียง
  float cmdPitch = constrain(-GAIN * pitch, -GIMBAL_LIMIT, GIMBAL_LIMIT);
  float cmdYaw   = constrain(-GAIN * roll,  -GIMBAL_LIMIT, GIMBAL_LIMIT);
  servoPitch.write(SERVO_CENTER + cmdPitch);
  servoYaw.write(SERVO_CENTER + cmdYaw);

  static uint8_t printDiv = 0;  // พิมพ์ 20 Hz
  if (++printDiv >= 10) {
    printDiv = 0;
    Serial.printf("%.2f,%.2f,%.1f,%.1f\n", pitch, roll,
                  SERVO_CENTER + cmdPitch, SERVO_CENTER + cmdYaw);
  }
}
