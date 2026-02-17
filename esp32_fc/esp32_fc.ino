#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;

// --- TÙY CHỈNH TẠI ĐÂY ---
float EXPO = 0.6;       // Tăng lên 0.6 để thấy rõ (0.0 là tắt)
float LIMIT_ANGLE = 30; // GIỚI HẠN GÓC (Độ): Máy bay sẽ không bao giờ vểnh quá mức này
int TRIM_L = 1500;      // Tinh chỉnh phẳng cánh trái
int TRIM_R = 1500;      // Tinh chỉnh phẳng cánh phải

long midP = 1500, midR = 1500;

// Hàm Expo mới: Chạy dựa trên tỷ lệ %, đảm bảo luôn có tác dụng
float applyExpo(float input, float expo) {
  float in = input / 500.0; // Đưa về -1.0 đến 1.0
  in = constrain(in, -1.0, 1.0);
  // Công thức Expo chuẩn quốc tế
  float out = (1 - expo) * in + expo * (in * in * in);
  return out * 500.0;
}

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLDOWN);
  pinMode(R_IN, INPUT_PULLDOWN);

  Serial.println(">>> ĐANG HỌC LỆNH TAY ĐIỀU KHIỂN...");
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
  // Đọc xung với Timeout ngắn để Failsafe nhạy
  long pI = pulseIn(P_IN, HIGH, 25000); 
  long rI = pulseIn(R_IN, HIGH, 25000);

  // --- CƠ CHẾ FAILSAFE MỚI ---
  // Nếu rút dây hoặc tắt TX, pulseIn trả về 0 hoặc giá trị rác
  bool isLostSignal = (pI == 0 || rI == 0 || pI < 800 || pI > 2200);
  
  if (isLostSignal) {
    pI = midP; // Ép về giữa
    rI = midR; // Ép về giữa
  }

  // Tính độ lệch từ điểm giữa
  float pRaw = pI - midP;
  float rRaw = rI - midR;

  // Áp dụng EXPO
  float pAfter = applyExpo(pRaw, EXPO);
  float rAfter = applyExpo(rRaw, EXPO);

  // --- GIỚI HẠN GÓC (LIMIT ANGLE) ---
  // Chuyển đổi từ xung (500us) sang góc lệch tối đa (LIMIT_ANGLE)
  // Xung 1us tương đương khoảng 0.1 độ
  float pFinal = constrain(pAfter, -LIMIT_ANGLE * 10, LIMIT_ANGLE * 10);
  float rFinal = constrain(rAfter, -LIMIT_ANGLE * 10, LIMIT_ANGLE * 10);

  // Mixer Elevon + Sync
  float vL = TRIM_L + pFinal + rFinal;
  float vR = TRIM_R - pFinal + rFinal;

  // Xuất xung an toàn
  ledcWrite(L_PIN, (constrain(vL, 1100, 1900) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1100, 1900) / 20000.0) * MAX_DUTY);

  if (millis() % 500 == 0) {
    if(isLostSignal) Serial.println("!!! FAILSAFE ACTIVE !!!");
    else {
      Serial.print("Raw:"); Serial.print(pRaw);
      Serial.print(" -> Expo:"); Serial.println(pAfter);
    }
  }
  delay(10);
}
