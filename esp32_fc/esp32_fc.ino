#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- BIỆN PHÁP MẠNH: COUNTER-PULSE LOGIC ---
volatile long rawP = 1500, rawR = 1500;
volatile unsigned long lastTimeP = 0, lastTimeR = 0;
unsigned long lastSignalTime = 0;

int GIOI_HAN = 200; 
int VUNG_CHET = 80; 
int TRIM_L = 1500, TRIM_R = 1500;

// Hàm ngắt bắt xung cực nhanh (Interrupt)
void IRAM_ATTR readP() {
  if (digitalRead(P_IN) == HIGH) lastTimeP = micros();
  else {
    long duration = micros() - lastTimeP;
    if (duration > 900 && duration < 2100) rawP = duration;
  }
}

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, 50, 16);
  ledcAttach(R_PIN, 50, 16);
  
  pinMode(P_IN, INPUT_PULLUP);
  attachInterrupt(P_IN, readP, CHANGE); // Bắt xung bằng phần cứng

  Serial.println(">>> DANG KHOI DONG HE THONG NGAT...");
  delay(2000);
}

void loop() {
  // --- 1. KIỂM TRA FAILSAFE (NGẮT THỜI GIAN THỰC) ---
  static long prevRawP = 0;
  if (rawP != prevRawP) {
    lastSignalTime = millis();
    prevRawP = rawP;
  }

  bool isFailsafe = (millis() - lastSignalTime > 150);

  // --- 2. COUNTER-PULSE LOGIC (TÍNH TOÁN XUNG NGƯỢC) ---
  float pOffset = 0;
  
  if (isFailsafe) {
    pOffset = 0; // Ép về tâm tuyệt đối
    if (millis() % 500 == 0) Serial.println("!!! FAILSAFE !!!");
  } else {
    long diff = rawP - 1500; // Khoảng cách so với điểm cân bằng

    // Nếu nằm trong vùng chết, AI sẽ gửi "Xung bù" để triệt tiêu độ lệch
    if (abs(diff) < VUNG_CHET) {
      pOffset = 0; // Đây chính là lúc gửi tín hiệu ngược để ép servo về 0
    } else {
      // Nếu ngoài vùng chết, cho phép di chuyển có giới hạn
      pOffset = constrain(diff, -GIOI_HAN, GIOI_HAN);
    }
  }

  // --- 3. XUẤT LỆNH ĐIỀU KHIỂN ---
  float vL = TRIM_L + pOffset;
  float vR = TRIM_R - pOffset;

  ledcWrite(L_PIN, (constrain(vL, 1100, 1900) / 20000.0) * 65535);
  ledcWrite(R_PIN, (constrain(vR, 1100, 1900) / 20000.0) * 65535);

  if (millis() % 200 == 0) {
    Serial.printf("Raw:%ld | Offset:%.1f | Status:%s\n", rawP, pOffset, (pOffset==0?"LOCKED":"MOVE"));
  }
}
