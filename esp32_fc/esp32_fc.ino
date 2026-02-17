#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;

// --- TÙY CHỈNH RIÊNG CHO MC6C ---
float EXPO = 0.5;       
float LIMIT_ANGLE = 25; 
int DEADBAND = 20;      // Tăng vùng chết lên 20 vì MC6C hay bị nhiễu kim (jitter)
int TRIM_L = 1500;      
int TRIM_R = 1500;      

// Biến lưu trạng thái thực tế của MC6C
long midP = 1500, midR = 1500;

float applyExpo(float input, float expo) {
  float in = constrain(input / 450.0, -1.0, 1.0); // MC6C dải xung hẹp hơn nên chia 450
  float out = (1 - expo) * in + expo * (in * in * in);
  return out * 450.0;
}

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLUP); // Dùng PULLUP cho MC7RE ổn định hơn
  pinMode(R_IN, INPUT_PULLUP);

  // QUAN TRỌNG: MC6C cần thời gian khởi động
  Serial.println(">>> DANG DOC TIN HIEU MICROZONE... THA CAN VE GIUA!");
  delay(3000); 
  
  long sumP = 0, sumR = 0;
  int count = 0;
  while(count < 20) {
    long p = pulseIn(P_IN, HIGH, 30000);
    long r = pulseIn(R_IN, HIGH, 30000);
    if(p > 900 && r > 900) { // Chỉ lấy mẫu khi có sóng
      sumP += p; sumR += r;
      count++;
    }
    delay(10);
  }
  midP = sumP / 20;
  midR = sumR / 20;
  
  Serial.printf("MC6C Center Learned: P=%ld, R=%ld\n", midP, midR);
}

void loop() {
  // Đọc xung từ MC7RE
  long pI = pulseIn(P_IN, HIGH, 25000); 
  long rI = pulseIn(R_IN, HIGH, 25000);

  // FAILSAFE: MC7RE v2 khi mất sóng thường trả về 0 hoặc giữ xung cuối
  // Nếu rút dây hoặc tắt TX mà pulseIn trả về 0:
  if (pI == 0 || pI < 800 || pI > 2200) pI = midP;
  if (rI == 0 || rI < 800 || rI > 2200) rI = midR;

  // TÍNH ĐỘ LỆCH THỰC TẾ
  float pRaw = (float)pI - midP;
  float rRaw = (float)rI - midR;

  // BUỘC AUTO CENTER: Loại bỏ jitter của Microzone
  if (abs(pRaw) < DEADBAND) pRaw = 0;
  if (abs(rRaw) < DEADBAND) rRaw = 0;

  // EXPO & LIMIT
  float pAfter = applyExpo(pRaw, EXPO);
  float rAfter = applyExpo(rRaw, EXPO);

  float maxDiff = LIMIT_ANGLE * 10.0; 
  float pFinal = constrain(pAfter, -maxDiff, maxDiff);
  float rFinal = constrain(rAfter, -maxDiff, maxDiff);

  // MIXER & SYNC
  float vL = (float)TRIM_L + pFinal + rFinal;
  float vR = (float)TRIM_R - pFinal + rFinal;

  ledcWrite(L_PIN, (constrain(vL, 1100, 1900) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1100, 1900) / 20000.0) * MAX_DUTY);

  if (millis() % 200 == 0) {
    Serial.print("MC6C Diff_P: "); Serial.println(pRaw);
  }
}
