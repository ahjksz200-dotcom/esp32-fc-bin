#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- CHỈNH SỬA TẠI ĐÂY ---
int GIOI_HAN = 250; 
int VUNG_CHET_AI = 85;  // Tăng lên 85 để bắt bằng được điểm giữa
int TRIM_L = 1500; 
int TRIM_R = 1500;
float EXPO = 0.7;

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;
long midP = 1500, midR = 1500;

// Bộ nhớ AI để lọc nhiễu
float smoothedP = 0;
unsigned long lastSignalTime = 0;
long lastRawP = 0;

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLUP);
  pinMode(R_IN, INPUT_PULLUP);

  Serial.println(">>> AI IS LEARNING NOISE PATTERNS...");
  delay(3000); 
  
  // Calib siêu kỹ: 100 mẫu để tìm điểm không (Zero point)
  long sP = 0;
  int validSamples = 0;
  while(validSamples < 100) {
    long p = pulseIn(P_IN, HIGH, 30000);
    if(p > 900 && p < 2100) {
      sP += p;
      validSamples++;
    }
    delay(2);
  }
  midP = sP / 100;
  midR = midP;
  Serial.printf("AI LEARNED CENTER: %ld\n", midP);
}

void loop() {
  long pI = pulseIn(P_IN, HIGH, 25000);
  long rI = pulseIn(R_IN, HIGH, 25000);

  // --- 1. AI FAILSAFE DETECTOR ---
  // Điều kiện Failsafe: Không có xung (0) HOẶC xung bị "đứng hình" tuyệt đối (lỗi RX)
  bool signalJumped = (abs(pI - lastRawP) > 1); // Tín hiệu thật phải có rung động nhỏ
  if (pI > 950 && pI < 2050 && (signalJumped || pI != lastRawP)) {
    lastSignalTime = millis();
    lastRawP = pI;
  }

  bool isFailsafe = (millis() - lastSignalTime > 120 || pI == 0);

  float pOut = 0, rOut = 0;

  if (isFailsafe) {
    pOut = 0; rOut = 0;
    if(millis() % 500 == 0) Serial.println("[AI] !!! FAILSAFE ACTIVE !!!");
  } 
  else {
    // --- 2. AI NOISE FILTER & CENTER LOCK ---
    // Lọc mượt tín hiệu đầu vào để chống rung
    smoothedP = (pI * 0.3) + (smoothedP * 0.7); 
    
    long diffP = (long)smoothedP - midP;
    long diffR = rI - midR;

    // ÉP AUTO CENTER: Nếu nằm trong vùng chết rộng, khóa ngay về 0
    if (abs(diffP) < VUNG_CHET_AI) {
      pOut = 0;
    } else {
      // Tính Expo nếu thoát vùng chết
      float inP = constrain(diffP / 500.0, -1.0, 1.0);
      pOut = ((1 - EXPO) * inP + EXPO * (inP * inP * inP)) * 500.0;
    }

    if (abs(diffR) < VUNG_CHET_AI) rOut = 0;
    else {
      float inR = constrain(diffR / 500.0, -1.0, 1.0);
      rOut = ((1 - EXPO) * inR + EXPO * (inR * inR * inR)) * 500.0;
    }
  }

  // --- 3. GIỚI HẠN GÓC CƯỠNG BỨC ---
  pOut = constrain(pOut, -GIOI_HAN, GIOI_HAN);
  rOut = constrain(rOut, -GIOI_HAN, GIOI_HAN);

  // MIXER & XUẤT XUNG
  float vL = TRIM_L + pOut + rOut;
  float vR = TRIM_R - pOut + rOut;

  ledcWrite(L_PIN, (constrain(vL, 1000, 2000) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1000, 2000) / 20000.0) * MAX_DUTY);

  // DEBUG ĐỂ TÌM LỖI
  if (millis() % 200 == 0) {
    Serial.print("Mode: ");
    if(isFailsafe) Serial.print("FAILSAFE");
    else if(pOut == 0) Serial.print("LOCKED");
    else Serial.print("MANUAL");
    
    Serial.print(" | P_Diff: "); Serial.println(pI - midP);
  }
}
