#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

/* ================= MPU ================= */
Adafruit_MPU6050 mpu;
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float rollRate = 0.0, pitchRate = 0.0;  // Added pitchRate
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accRollOffset = 0, accPitchOffset = 0;
const float alpha = 0.98;
unsigned long lastMicros;

/* ================= PCA9685 ================= */
Adafruit_PWMServoDriver pca9685(0x40);

#define SERVO_FRONT_CH 0
#define SERVO_BACK_CH  1
#define SERVO_FREQ 50

#define SERVO_NEUTRAL 1500
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

/* ================= RC ================= */
#define RC_ROLL_PIN 34
#define RC_PITCH_PIN 35
#define RC_THROTTLE_PIN 25
#define RC_YAW_PIN 27
volatile uint32_t rollStart, pitchStart, thrStart, yawStart;
volatile int rcRoll = 1500, rcPitch = 1500, rcThrottle = 1000, rcYaw = 1500;

/* ================= GAINS ================= */
float KP_ROLL = 14.0;   // Roll responsiveness
float KD_ROLL = 2.0;    // Roll damping
float KP_PITCH = 12.0;  // Pitch
float KD_PITCH = 3.0;   // Pitch

/* ================= UTILS ================= */
uint16_t usToTicks(int us) {
  return map(us, 0, 20000, 0, 4096);
}

/* ================= RC ISR ================= */
void IRAM_ATTR rollISR() {
  if (digitalRead(RC_ROLL_PIN))
    rollStart = micros();
  else
    rcRoll = micros() - rollStart;
}

void IRAM_ATTR pitchISR() {
  if (digitalRead(RC_PITCH_PIN))
    pitchStart = micros();
  else
    rcPitch = micros() - pitchStart;
}

void IRAM_ATTR throttleISR() {
  if (digitalRead(RC_THROTTLE_PIN))
    thrStart = micros();
  else
    rcThrottle = micros() - thrStart;
}

void IRAM_ATTR yawISR() {
  if (digitalRead(RC_YAW_PIN))
    yawStart = micros();
  else
    rcYaw = micros() - yawStart;
}

/* ================= CALIBRATION ================= */
void calibrateGyro() {
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x;
    sy += g.gyro.y;
    sz += g.gyro.z;
    delay(4);
  }
  gyroOffsetX = sx / 500;
  gyroOffsetY = sy / 500;
  gyroOffsetZ = sz / 500;
}

void calibrateAccelLevel() {
  float sr = 0, sp = 0;
  for (int i = 0; i < 300; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sr += atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
    sp += atan2(-a.acceleration.x,
                sqrt(a.acceleration.y*a.acceleration.y +
                     a.acceleration.z*a.acceleration.z)) * RAD_TO_DEG;
    delay(5);
  }
  accRollOffset = sr / 300;
  accPitchOffset = sp / 300;
}

/* ================= SERIAL COMMANDS ================= */
void readSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("KP_ROLL,")) KP_ROLL = cmd.substring(8).toFloat();
  if (cmd.startsWith("KD_ROLL,")) KD_ROLL = cmd.substring(8).toFloat();
  if (cmd.startsWith("KP_PITCH,")) KP_PITCH = cmd.substring(9).toFloat();
  if (cmd.startsWith("KD_PITCH,")) KD_PITCH = cmd.substring(9).toFloat();
}

/* ================= IMU UPDATE ================= */
void updateIMU() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  lastMicros = now;

  float gx = (gyro.gyro.x - gyroOffsetX) * RAD_TO_DEG;
  float gy = (gyro.gyro.y - gyroOffsetY) * RAD_TO_DEG;
  float gz = (gyro.gyro.z - gyroOffsetZ) * RAD_TO_DEG;

  rollRate = gx;  // deg/s
  pitchRate = gy; // deg/s

  roll += gx * dt;
  pitch += gy * dt;
  yaw += gz * dt;

  float accRoll = atan2(accel.acceleration.y, accel.acceleration.z) * RAD_TO_DEG - accRollOffset;
  float accPitch = atan2(-accel.acceleration.x,
                         sqrt(accel.acceleration.y * accel.acceleration.y +
                              accel.acceleration.z * accel.acceleration.z)) * RAD_TO_DEG - accPitchOffset;

  roll = alpha * roll + (1 - alpha) * accRoll;
  pitch = alpha * pitch + (1 - alpha) * accPitch;
}

/* ================= PD CONTROL ================= */
void updateControl() {
  // RC targets (deg, -20 to 20)
  int rcR = constrain(rcRoll, 1000, 2000);
  int rcP = constrain(rcPitch, 1000, 2000);
  float targetRoll = map(rcR, 1000, 2000, -20, 20);
  float targetPitch = map(rcP, 1000, 2000, -20, 20);

  // Roll PD
  float errorRoll = targetRoll - roll;
  float correctionRoll = KP_ROLL * errorRoll - KD_ROLL * rollRate;

  // Pitch PD (placeholder)
  // float errorPitch = targetPitch - pitch;
  // float correctionPitch = KP_PITCH * errorPitch - KD_PITCH * pitchRate;

  // Servo outputs (roll differential)
  int front = SERVO_NEUTRAL + correctionRoll;
  int back  = SERVO_NEUTRAL - correctionRoll;

  front = constrain(front, SERVO_MIN_US, SERVO_MAX_US);
  back  = constrain(back, SERVO_MIN_US, SERVO_MAX_US);

  pca9685.setPWM(SERVO_FRONT_CH, 0, usToTicks(front));
  pca9685.setPWM(SERVO_BACK_CH,  0, usToTicks(back));

  // Send servo telemetry here
  Serial.print("SERVO_FB,");
  Serial.print(front); Serial.print(",");
  Serial.println(back);
}

/* ================= TELEMETRY ================= */
void sendTelemetry() {
  // Attitude + rates
  Serial.print("ATT,");
  Serial.print(roll, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.print(yaw, 2); Serial.print(",");
  Serial.println(rollRate, 2);

  // RC
  Serial.print("RC,");
  Serial.print(constrain(rcRoll, 1000, 2000)); Serial.print(",");
  Serial.print(constrain(rcPitch, 1000, 2000)); Serial.print(",");
  Serial.print(constrain(rcThrottle, 1000, 2000)); Serial.print(",");
  Serial.println(constrain(rcYaw, 1000, 2000));
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(RC_ROLL_PIN, INPUT);
  pinMode(RC_PITCH_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_YAW_PIN, INPUT);
  attachInterrupt(RC_ROLL_PIN, rollISR, CHANGE);
  attachInterrupt(RC_PITCH_PIN, pitchISR, CHANGE);
  attachInterrupt(RC_THROTTLE_PIN, throttleISR, CHANGE);
  attachInterrupt(RC_YAW_PIN, yawISR, CHANGE);

  if (!mpu.begin()) {
    Serial.println("MPU FAIL");
    while (1);
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pca9685.begin();
  pca9685.setPWMFreq(SERVO_FREQ);

  delay(2000);
  calibrateGyro();
  calibrateAccelLevel();

  Serial.println("Flight Controller Ready");
  lastMicros = micros();
}

/* ================= LOOP ================= */
void loop() {
  readSerialCommands();

  updateIMU();
  updateControl();
  sendTelemetry();

  delay(10);  // ~100 Hz
}