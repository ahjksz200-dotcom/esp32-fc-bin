#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- BIẾN FIX LỖI ---
volatile long rawP = 1500, rawR = 1500;
volatile unsigned long lastTimeP = 0, lastTimeR = 0;
long midP = 1500, midR = 1500; // Sẽ được tự động học lại
int VUNG_CHET = 60; // Nới rộng vùng chết để bắt bằng được điểm giữa

// Ngắt bắt xung chính xác 99%
void IRAM_ATTR readP() {
  if (digitalRead(P_IN) == HIGH) lastTimeP = micros();
  else { long d = micros() - lastTimeP; if (d > 800 && d < 2200) rawP = d; }
}
void IRAM_ATTR readR() {
  if (digitalRead(R_IN) == HIGH) lastTimeR = micros();
  else { long d = micros() - lastTimeR; if (d > 800 && d < 2200) rawR = d; }
}

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, 50, 16);
  ledcAttach(R_PIN, 50, 16);
  pinMode(P_IN, INPUT_PULLUP);
  pinMode(R_IN, INPUT_PULLUP);
  attachInterrupt(P_IN, readP, CHANGE);
  attachInterrupt(R_IN, readR, CHANGE);

  // --- BƯỚC FIX LỖI QUAN TRỌNG NHẤT ---
  Serial.println(">>> ĐANG HỌC VỊ TRÍ TAY MC6C... ĐỂ YÊN CẦN GẠT!");
  delay(3000); // Chờ 3 giây để bạn buông tay hoàn toàn
  midP = rawP; // Lấy giá trị thực tế của MC6C làm điểm giữa mới
  midR = rawR;
  Serial.printf("Học xong! Điểm giữa thực tế: P=%ld, R=%ld\n", midP, midR);
}

void loop() {
  // 1. Tính độ lệch dựa trên "Điểm giữa đã học"
  long diffP = rawP - midP; 
  long diffR = rawR - midR;

  float pOut = 0, rOut = 0;

  // 2. ÉP AUTO CENTER (Bản fix lỗi)
  // Nếu lệch ít hơn VUNG_CHET, ép chết về 0 ngay lập tức
  if (abs(diffP) < VUNG_CHET) pOut = 0; 
  else pOut = diffP;

  if (abs(diffR) < VUNG_CHET) rOut = 0;
  else rOut = diffR;

  // 3. MIXER & XUẤT XUNG
  float vL = 1500 + pOut + rOut;
  float vR = 1500 - pOut + rOut;

  ledcWrite(L_PIN, (constrain(vL, 1100, 1900) * 65535) / 20000);
  ledcWrite(R_PIN, (constrain(vR, 1100, 1900) * 65535) / 20000);

  if (millis() % 200 == 0) {
    Serial.printf("RawP:%ld | MidP:%ld | Diff:%ld | %s\n", 
                  rawP, midP, diffP, (pOut == 0 ? "LOCKED" : "MOVING"));
  }
}
