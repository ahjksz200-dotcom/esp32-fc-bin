#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;

// --- CÀI ĐẶT ÉP PHÊ ---
float EXPO = 0.6;       
float LIMIT_ANGLE = 25; 
int DEADBAND = 40;      // Tăng mạnh lên 40 để ép Center tuyệt đối
int TRIM_L = 1500;      
int TRIM_R = 1500;      

long midP = 1500, midR = 1500;

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLUP);
  pinMode(R_IN, INPUT_PULLUP);

  Serial.println(">>> DANG CALIB... GIU CAN O GIUA!!!");
  delay(3000); 
  
  long sumP = 0, sumR = 0;
  for(int i=0; i<50; i++) { // Đọc 50 lần để lấy con số trung bình siêu chuẩn
    sumP += pulseIn(P_IN, HIGH, 30000);
    sumR += pulseIn(R_IN, HIGH, 30000);
    delay(5);
  }
  midP = sumP / 50;
  midR = sumR / 50;
  Serial.printf("DONE! Center: P=%ld, R=%ld\n", midP, midR);
}

void loop() {
  long pI = pulseIn(P_IN, HIGH, 25000); 
  long rI = pulseIn(R_IN, HIGH, 25000);

  // --- 1. FAILSAFE CHO MC7RE ---
  // Nếu tắt TX mà RX vẫn gửi xung cũ, code sẽ kiểm tra độ rung (jitter)
  // Thường khi mất sóng, xung từ RX sẽ đứng im tuyệt đối hoặc nhảy loạn
  static long lastPI = 0;
  bool isFrozen = (pI == lastPI && pI != midP); // Kiểm tra xem tín hiệu có bị "đứng hình" không
  lastPI = pI;

  if (pI == 0 || pI < 900 || pI > 2100 || isFrozen) {
    pI = midP;
    rI = midR;
  }

  // --- 2. ÉP AUTO CENTER BẰNG LỆCH ĐỐI CHIẾU ---
  float pRaw = (float)pI - midP;
  float rRaw = (float)rI - midR;

  // Nếu lệch dưới 40 đơn vị (Deadband), ép về 0 ngay lập tức
  if (abs(pRaw) < DEADBAND) pRaw = 0;
  if (abs(rRaw) < DEADBAND) rRaw = 0;

  // --- 3. EXPO & LIMIT ---
  float inP = constrain(pRaw / 450.0, -1.0, 1.0);
  float pAfter = ((1 - EXPO) * inP + EXPO * (inP * inP * inP)) * 450.0;
  
  float inR = constrain(rRaw / 450.0, -1.0, 1.0);
  float rAfter = ((1 - EXPO) * inR + EXPO * (inR * inR * inR)) * 450.0;

  float maxDiff = LIMIT_ANGLE * 10.0; 
  float pFinal = constrain(pAfter, -maxDiff, maxDiff);
  float rFinal = constrain(rAfter, -maxDiff, maxDiff);

  // --- 4. XUẤT RA SERVO ---
  float vL = (float)TRIM_L + pFinal + rFinal;
  float vR = (float)TRIM_R - pFinal + rFinal;

  ledcWrite(L_PIN, (constrain(vL, 1100, 1900) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1100, 1900) / 20000.0) * MAX_DUTY);

  if (millis() % 200 == 0) {
    Serial.print("P_Diff: "); Serial.print(pRaw);
    Serial.print(" | L_Servo: "); Serial.println(vL);
  }
}
