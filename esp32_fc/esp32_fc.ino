#include <Wire.h>

// --- CẤU HÌNH CHÂN PIN ---
#define MPU_ADDR 0x68
#define LEFT_SERVO_PIN 13
#define RIGHT_SERVO_PIN 12
#define PWM_PITCH_IN 14
#define PWM_ROLL_IN  27

// --- THÔNG SỐ PWM MỚI (ESP32 Core v3.x) ---
#define PWM_FREQ 50
#define PWM_RES 16 

// --- CẤU TRÚC PID ---
struct PID {
  float kp, ki, kd;
  float integral, lastError;
};

// --- KHU VỰC CÂN CHỈNH (TUNING) ---
PID pPID = {15.0, 0.05, 2.0}; 
PID rPID = {15.0, 0.05, 2.0}; 
PID yPID = {12.0, 0.02, 1.0}; 

float max_angle = 40.0;       
int flightMode = 0;           

// --- BIẾN TOÀN CỤC ---
float ax, ay, az, gx, gy, gz;
float offsetGX, offsetGY, offsetGZ;
float roll = 0, pitch = 0;
unsigned long lastTime;

void writeReg(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

float runPID(float target, float actual, PID &p, float dt) {
  float error = target - actual;
  p.integral += error * dt;
  p.integral = constrain(p.integral, -50, 50); 
  float derivative = (error - p.lastError) / dt;
  p.lastError = error;
  return (error * p.kp) + (p.integral * p.ki) + (derivative * p.kd);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  writeReg(0x6B, 0x00); 

  // 1. AUTO-CALIBRATION
  Serial.println(">>> CALIBRATING...");
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    sumX += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    sumY += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    sumZ += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    delay(2);
  }
  offsetGX = sumX / 500.0;
  offsetGY = sumY / 500.0;
  offsetGZ = sumZ / 500.0;

  // 2. CẤU HÌNH PWM KIỂU MỚI (LEDC V3.x)
  // Không dùng ledcSetup nữa, dùng ledcAttach trực tiếp
  ledcAttach(LEFT_SERVO_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_SERVO_PIN, PWM_FREQ, PWM_RES);

  pinMode(PWM_PITCH_IN, INPUT);
  pinMode(PWM_ROLL_IN, INPUT);

  lastTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  if (dt <= 0) dt = 0.005;
  lastTime = currentTime;

  // Đọc cảm biến
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  ax = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  ay = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  az = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.read(); Wire.read(); 
  gx = ((int16_t)(Wire.read() << 8 | Wire.read()) / 131.0) - offsetGX;
  gy = ((int16_t)(Wire.read() << 8 | Wire.read()) / 131.0) - offsetGY;
  gz = ((int16_t)(Wire.read() << 8 | Wire.read()) / 131.0) - offsetGZ;

  // Tính toán góc
  float pAcc = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float rAcc = atan2(ay, az) * 180.0 / PI;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * pAcc;
  roll  = 0.98 * (roll + gx * dt) + 0.02 * rAcc;

  // Đọc RX & Failsafe
  long pIn = pulseIn(PWM_PITCH_IN, HIGH, 25000);
  long rIn = pulseIn(PWM_ROLL_IN, HIGH, 25000);
  float targetP = 0, targetR = 0;
  if (pIn == 0 || rIn == 0) {
    targetP = 0; targetR = 0;
  } else {
    targetP = map(pIn, 1000, 2000, -max_angle, max_angle);
    targetR = map(rIn, 1000, 2000, -max_angle, max_angle);
  }

  // PID 3 Trục
  float outP = runPID(targetP, pitch, pPID, dt);
  float outR = runPID(targetR, roll, rPID, dt);
  float outY = runPID(0, gz, yPID, dt);

  // Mixer Elevon
  float leftUs  = 1500 + outP + outR + outY;
  float rightUs = 1500 - outP + outR + outY;

  leftUs = constrain(leftUs, 1000, 2000);
  rightUs = constrain(rightUs, 1000, 2000);

  // XUẤT PWM KIỂU MỚI (Sử dụng ledcWrite trực tiếp vào chân Pin)
  uint32_t dutyL = (leftUs / 20000.0) * ((1 << PWM_RES) - 1);
  uint32_t dutyR = (rightUs / 20000.0) * ((1 << PWM_RES) - 1);
  
  ledcWrite(LEFT_SERVO_PIN, dutyL);
  ledcWrite(RIGHT_SERVO_PIN, dutyR);

  delay(5);
}
