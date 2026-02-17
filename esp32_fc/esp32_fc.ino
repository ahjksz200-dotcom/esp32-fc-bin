#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;

// --- TÙY CHỈNH TẠI ĐÂY ---
float EXPO = 0.6;       
float LIMIT_ANGLE = 25; // GIỚI HẠN GÓC (Độ): Thử để thấp (ví dụ 15-20) để thấy nó bị chặn
int TRIM_L = 1500;      
int TRIM_R = 1500;      

long midP = 1500, midR = 1500;

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

  Serial.println(">>> DANG HOC LENH (THA CAN VE GIUA)...");
  delay(2000); 
  
  long sumP = 0, sumR = 0;
  for(int i=0; i<10; i++) {
    sumP += pulseIn(P_IN, HIGH, 30000);
    sumR += pulseIn(R_IN, HIGH, 30000);
    delay(20);
  }
  midP = (sumP / 10 > 900) ? sumP / 10 : 1500;
  midR = (sumR / 10 > 900) ? sumR / 10 : 1500;
}

void loop() {
  long pI = pulseIn(P_IN, HIGH, 25000); 
  long rI = pulseIn(R_IN, HIGH, 25000);

  // --- 1. KIỂM TRA FAILSAFE TRƯỚC (BẮT BUỘC) ---
  bool isLost = (pI == 0 || pI < 900 || pI > 2100 || rI == 0 || rI < 900 || rI > 2100);
  
  if (isLost) {
    pI = midP; // Ép giá trị đọc được về điểm giữa đã học
    rI = midR;
  }

  // --- 2. TÍNH TOÁN DỰA TRÊN GIÁ TRỊ ĐÃ XỬ LÝ FAILSAFE ---
  float pRaw = (float)pI - midP;
  float rRaw = (float)rI - midR;

  // Áp dụng EXPO
  float pAfter = applyExpo(pRaw, EXPO);
  float rAfter = applyExpo(rRaw, EXPO);

  // --- 3. GIỚI HẠN GÓC (LIMIT ANGLE) - Ép biên cực độ ---
  // Mỗi 10us tương đương khoảng 1 độ lệch. 
  float maxDiff = LIMIT_ANGLE * 10.0; 
  float pFinal = constrain(pAfter, -maxDiff, maxDiff);
  float rFinal = constrain(rAfter, -maxDiff, maxDiff);

  // --- 4. MIXER ELEVON + SYNC ---
  float vL = (float)TRIM_L + pFinal + rFinal;
  float vR = (float)TRIM_R - pFinal + rFinal;

  // --- 5. XUẤT XUNG AN TOÀN ---
  ledcWrite(L_PIN, (constrain(vL, 1100, 1900) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1100, 1900) / 20000.0) * MAX_DUTY);

  if (millis() % 500 == 0) {
    if(isLost) Serial.println("!!! FAILSAFE COMMANDING SERVOS TO CENTER !!!");
    else {
      Serial.print("P_Final:"); Serial.print(pFinal);
      Serial.print(" | L_Out:"); Serial.println(vL);
    }
  }
  delay(10);
}
