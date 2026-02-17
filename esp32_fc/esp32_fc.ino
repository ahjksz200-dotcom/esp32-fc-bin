#include <Arduino.h>

// --- 1. CẤU HÌNH PIN ---
#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- 2. ĐỒNG BỘ SERVO (SERVO SYNC) ---
// Nếu servo bên trái hơi cao, hãy giảm số này (ví dụ 1480)
// Nếu servo bên phải hơi thấp, hãy tăng số này (ví dụ 1520)
int TRIM_L = 1500; 
int TRIM_R = 1500; 

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;

// Biến lưu điểm giữa học được từ tay điều khiển
long midP = 1500, midR = 1500;
float EXPO = 0.5; 
float RATE = 0.9;

float applyExpo(float input, float expo) {
  float in = constrain(input / 500.0, -1.0, 1.0);
  float out = (1 - expo) * in + expo * (in * in * in);
  return out * 500.0;
}

void setup() {
  Serial.begin(115200);
  
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLDOWN);
  pinMode(R_IN, INPUT_PULLDOWN);

  // HỌC ĐIỂM GIỮA (Giúp Auto-Center hoạt động chuẩn)
  Serial.println(">>> THA CAN VE GIUA DE DONG BO...");
  delay(2000); 
  
  long sumP = 0, sumR = 0;
  for(int i=0; i<10; i++) {
    sumP += pulseIn(P_IN, HIGH, 30000);
    sumR += pulseIn(R_IN, HIGH, 30000);
    delay(20);
  }
  midP = (sumP / 10 > 900) ? sumP / 10 : 1500;
  midR = (sumR / 10 > 900) ? sumR / 10 : 1500;

  Serial.printf("Center Learned: P=%ld, R=%ld\n", midP, midR);
}

void loop() {
  // Đọc xung RX
  long pI = pulseIn(P_IN, HIGH, 25000); 
  long rI = pulseIn(R_IN, HIGH, 25000);

  // FAILSAFE: Nếu mất tín hiệu (rút dây hoặc tắt TX)
  if (pI == 0 || pI < 900 || pI > 2100) pI = midP;
  if (rI == 0 || rI < 900 || rI > 2100) rI = midR;

  // Tính toán độ lệch có kèm EXPO
  float pDiff = applyExpo(pI - midP, EXPO) * RATE;
  float rDiff = applyExpo(rI - midR, EXPO) * RATE;

  // MIXER ELEVON + ĐỒNG BỘ TRIM
  // Thay vì cộng vào 1500, mình cộng vào giá trị TRIM riêng của mỗi bên
  float vL = TRIM_L + pDiff + rDiff;
  float vR = TRIM_R - pDiff + rDiff;

  // Giới hạn an toàn và xuất xung
  ledcWrite(L_PIN, (constrain(vL, 1100, 1900) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1100, 1900) / 20000.0) * MAX_DUTY);

  // Debug để kiểm tra Expo và Auto-center
  if (millis() % 500 == 0) {
    Serial.print("Diff_P: "); Serial.print(pI - midP);
    Serial.print(" | Final_L: "); Serial.println(vL);
  }
  delay(10);
}
