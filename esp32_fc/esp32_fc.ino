#include <Wire.h>

// --- CẤU HÌNH CHÂN PIN ---
#define MPU_ADDR 0x68
#define LEFT_SERVO_PIN 13
#define RIGHT_SERVO_PIN 12
#define PWM_PITCH_IN 14
#define PWM_ROLL_IN  27

// --- THÔNG SỐ PWM SERVO (50Hz, 16-bit) ---
#define PWM_FREQ 50
#define PWM_RES 16 
#define CH_L 0
#define CH_R 1

// --- CẤU TRÚC PID ---
struct PID {
  float kp, ki, kd;
  float integral, lastError;
};

// --- KHU VỰC CÂN CHỈNH (TUNING) ---
// Bạn có thể thay đổi các số này để máy bay bay êm hơn
PID pPID = {15.0, 0.05, 2.0}; // PID cho trục Pitch
PID rPID = {15.0, 0.05, 2.0}; // PID cho trục Roll
PID yPID = {12.0, 0.02, 1.0}; // PID cho trục Yaw (Kháng gió ngang)

float max_angle = 40.0;       // Góc nghiêng tối đa (độ)
int flightMode = 0;           // 0: Angle Mode (Tự cân bằng), 1: Rate Mode (Acro)

// --- BIẾN TOÀN CỤC ---
float ax, ay, az, gx, gy, gz;
float offsetGX, offsetGY, offsetGZ;
float roll = 0, pitch = 0;
unsigned long lastTime;

// Hàm ghi vào thanh ghi MPU
void writeReg(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// Hàm tính toán PID
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
  writeReg(0x6B, 0x00); // Wake up MPU

  // 1. AUTO-CALIBRATION: Giữ máy bay thật đứng yên khi bật nguồn
  Serial.println(">>> ĐANG CÂN BẰNG... GIỮ MÁY BAY PHẲNG VÀ ĐỨNG YÊN!");
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
  Serial.println(">>> CÂN BẰNG XONG! SẴN SÀNG BAY.");

  // 2. Cấu hình PWM cho Servo
  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_SERVO_PIN, CH_L);
  ledcAttachPin(RIGHT_SERVO_PIN, CH_R);

  pinMode(PWM_PITCH_IN, INPUT);
  pinMode(PWM_ROLL_IN, INPUT);

  lastTime = micros();
}

void loop() {
  // 1. Quản lý thời gian dt
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  if (dt <= 0) dt = 0.005; // Tránh chia cho 0
  lastTime = currentTime;

  // 2. Đọc dữ liệu từ cảm biến
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  
  ax = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  ay = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  az = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.read(); Wire.read(); // Bỏ qua nhiệt độ
  
  // Đọc Gyro và trừ đi Offset đã calibrate
  gx = ((int16_t)(Wire.read() << 8 | Wire.read()) / 131.0) - offsetGX;
  gy = ((int16_t)(Wire.read() << 8 | Wire.read()) / 131.0) - offsetGY;
  gz = ((int16_t)(Wire.read() << 8 | Wire.read()) / 131.0) - offsetGZ;

  // 3. Tính toán góc hiện tại (Bộ lọc bù)
  float pAcc = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float rAcc = atan2(ay, az) * 180.0 / PI;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * pAcc;
  roll  = 0.98 * (roll + gx * dt) + 0.02 * rAcc;

  // 4. Đọc tín hiệu RX & Failsafe
  long pIn = pulseIn(PWM_PITCH_IN, HIGH, 25000);
  long rIn = pulseIn(PWM_ROLL_IN, HIGH, 25000);
  
  float targetP = 0, targetR = 0;
  if (pIn == 0 || rIn == 0) {
    // FAILSAFE: Mất sóng thì giữ thăng bằng
    targetP = 0; targetR = 0;
  } else {
    // Chuyển xung 1000-2000us sang góc mục tiêu
    targetP = map(pIn, 1000, 2000, -max_angle, max_angle);
    targetR = map(rIn, 1000, 2000, -max_angle, max_angle);
  }

  // 5. Tính toán Output PID cho 3 trục
  float outP, outR, outY;
  
  if (flightMode == 0) { // Angle Mode: Giữ góc ổn định
    outP = runPID(targetP, pitch, pPID, dt);
    outR = runPID(targetR, roll, rPID, dt);
  } else { // Rate Mode: Giữ tốc độ xoay ổn định
    outP = runPID(targetP * 2, gy, pPID, dt); 
    outR = runPID(targetR * 2, gx, rPID, dt);
  }
  
  // Kháng gió Yaw (Target xoay ngang luôn là 0 độ/s)
  outY = runPID(0, gz, yPID, dt);

  // 6. MIXER: Trộn Elevon (Cánh trái & Cánh phải)
  // Pitch (+/-), Roll (+/+), Yaw (+/+)
  float leftUs  = 1500 + outP + outR + outY;
  float rightUs = 1500 - outP + outR + outY;

  // Giới hạn xung an toàn cho Servo
  leftUs = constrain(leftUs, 1000, 2000);
  rightUs = constrain(rightUs, 1000, 2000);

  // Xuất PWM ra Servo
  ledcWrite(CH_L, (leftUs / 20000.0) * 65535);
  ledcWrite(CH_R, (rightUs / 20000.0) * 65535);

  // Debug (Tùy chọn - Mở Serial Monitor để xem)
  // Serial.printf("P: %.2f | R: %.2f | YawOut: %.2f\n", pitch, roll, outY);

  delay(5); // Duy trì vòng lặp 200Hz cực mượt
}
