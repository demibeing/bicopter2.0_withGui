#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_PWMServoDriver.h>

// --- TRANSMITTER CALIBRATION ---
const int RC_MIN = 920;   // Your stick at the bottom
const int RC_MAX = 1850;  // Your stick at the top
const int RC_MID = 1385;  // Calculated center (for Pitch)
//RAW changed to RC

// --- PINS AND CHANNELS ---
#define RC_THROTTLE_PIN 25
#define RC_PITCH_PIN 34
#define FRONT_MOTOR_ESC_CHANNEL 14
#define BACK_MOTOR_ESC_CHANNEL 15

// PCA9685 Standard Limits (1.0ms to 2.0ms)
//values 205 and 410 got from math corresponds to 1000ms and 2000ms
#define ESC_MIN 205 
#define ESC_MAX 410 

//The Values (1000 and 1385): These are "Safe Defaults."
//1000 is the standard "Zero" for throttle.
//1385 is the center point for your pitch (based on the 920–1850 range you found).
volatile int throttle_raw = 1000, pitch_raw = 1385;
volatile unsigned long t_start = 0, p_start = 0;

// PID Constants
float KP_PITCH = 2.5, KI_PITCH = 0.02, KD_PITCH = 12.0;
float ALPHA = 0.98, PITCH_THRUST_GAIN = 0.3;
float smoothedPitch = 0, pitchErrorSum = 0, lastPitchError = 0;
unsigned long lastTime = 0;

Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- INTERRUPTS ---
void IRAM_ATTR readThrottle() {
  // Step 1: The signal just turned ON (HIGH)
  if (digitalRead(RC_THROTTLE_PIN) == HIGH) t_start = micros();// Capture the "Start" timestamp
  else {
     int p = micros() - t_start; // THE MATH: Current Time minus Start Time
     // Step 3: Validation Filter
    if (p > 800 && p < 2200) throttle_raw = p; }// Update the variable used by the main loop
}

void IRAM_ATTR readPitch() {
  if (digitalRead(RC_PITCH_PIN) == HIGH) p_start = micros();
  else { int p = micros() - p_start; if (p > 800 && p < 2200) pitch_raw = p; }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mpu.begin()) while (1); // Stop if MPU fails
  
  pwm.begin();
  pwm.setPWMFreq(60);

  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_PITCH_PIN, INPUT);
  attachInterrupt(RC_THROTTLE_PIN, readThrottle, CHANGE);
  attachInterrupt(RC_PITCH_PIN, readPitch, CHANGE);

  // Arming/IDLE state
  pwm.setPWM(FRONT_MOTOR_ESC_CHANNEL, 0, ESC_MIN);
  pwm.setPWM(BACK_MOTOR_ESC_CHANNEL, 0, ESC_MIN);
  delay(2000); 
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.01;
  lastTime = currentTime;

  // --- 1. LINEAR MAPPING ---
  // We use 940 to 1830 to ensure the motors start/stop reliably
  //throttle_pct means throttle percentage
  float throttle_pct = constrain(map(throttle_raw, 940, 1830, 0, 100), 0, 100);
  
  // Pitch mapping (Desired angle -20 to +20 degrees)
  float desired_pitch = map(pitch_raw, RAW_MIN, RAW_MAX, -20, 20);

  // --- 2. SENSING ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float rawPitch = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;//pitch
  //complementary filter for pitch
  smoothedPitch = ALPHA * (smoothedPitch + g.gyro.y * dt * (180 / PI)) + (1 - ALPHA) * rawPitch;

  float rawRoll = atan2(-a.acceleration.y, a.acceleration.z) * 180 / PI;//roll
  //complementary filter for roll
  smoothedRoll = ALPHA * (smoothedRoll + g.gyro.x * dt * (180 / PI)) + (1 - ALPHA) * rawRoll;


  // --- 3. PID ---
  float Pitch_error = desired_pitch - smoothedPitch;
  float Roll_error = desired_roll - smoothedRoll

  pitchErrorSum = constrain(pitchErrorSum + error * dt, -15, 15);
  float pid = (KP_PITCH * error) + (KI_PITCH * pitchErrorSum) + (KD_PITCH * (error - lastPitchError) / dt);
  lastPitchError = error;

  // --- 4. MOTOR MIXING ---
  int base_pwm = ESC_MIN + (throttle_pct / 100.0) * (ESC_MAX - ESC_MIN);
  float pitch_adj = pid * PITCH_THRUST_GAIN * (ESC_MAX - ESC_MIN) / 100.0;

  if (throttle_pct > 2) { // Only spin if stick is moved up
    pwm.setPWM(FRONT_MOTOR_ESC_CHANNEL, 0, constrain(base_pwm - pitch_adj, ESC_MIN, ESC_MAX));
    pwm.setPWM(BACK_MOTOR_ESC_CHANNEL, 0, constrain(base_pwm + pitch_adj, ESC_MIN, ESC_MAX));
  } else {
    pwm.setPWM(FRONT_MOTOR_ESC_CHANNEL, 0, ESC_MIN);
    pwm.setPWM(BACK_MOTOR_ESC_CHANNEL, 0, ESC_MIN);
    pitchErrorSum = 0; // Prevent motor "jump" on takeoff
  }

  // --- DEBUG ---
  Serial.printf("RawT:%d | Throt%%:%.0f | Pitch:%.1f\n", throttle_raw, throttle_pct, smoothedPitch);
}
