#include <Arduino.h>

// --- 1. CẤU HÌNH CHÂN CẮM (Pinout) ---
#define L_PIN 13    // Servo cánh trái
#define R_PIN 12    // Servo cánh phải
#define P_IN 14     // Tín hiệu Pitch từ RX (Lên/Xuống)
#define R_IN 27     // Tín hiệu Roll từ RX (Trái/Phải)

// --- 2. TÙY CHỈNH CẢM GIÁC BAY ---
float EXPO = 0.4;   // Độ mượt: 0.0 (thẳng), 0.4 (bay mượt), 0.7 (rất mượt ở giữa cần)
float RATE = 0.8;   // Độ hỗn: 0.8 (an toàn cho máy bay nhanh), 1.0 (hết công suất)

// Thông số kỹ thuật PWM
const int FREQ = 50, RES = 16, MAX_DUTY = 65535;

// Hàm xử lý EXPO (Giúp tay lái không bị quá "hỗn" ở vị trí giữa)
float applyExpo(float input, float expo) {
  float in = constrain(input / 500.0, -1.0, 1.0); 
  float out = (1 - expo) * in + expo * (in * in * in);
  return out * 500.0;
}

void setup() {
  Serial.begin(115200);

  // Khởi động mềm: Đợi 1.5s cho nguồn ổn định trước khi cấp xung Servo
  pinMode(L_PIN, OUTPUT); digitalWrite(L_PIN, LOW);
  pinMode(R_PIN, OUTPUT); digitalWrite(R_PIN, LOW);
  delay(1500);

  // Thiết lập PWM cho ESP32
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);

  // Thiết lập chân đọc RX (Dùng PULLDOWN để chống nhiễu khi rút dây)
  pinMode(P_IN, INPUT_PULLDOWN);
  pinMode(R_IN, INPUT_PULLDOWN);

  Serial.println(">>> AIRPLANE READY: MANUAL PRO MODE ENABLED!");
  Serial.println(">>> FAILSAFE: ACTIVE | EXPO: " + String(EXPO));
}

void loop() {
  // 1. Đọc xung từ Bộ thu (RX) với Timeout 25ms
  long pI = pulseIn(P_IN, HIGH, 25000);
  long rI = pulseIn(R_IN, HIGH, 25000);

  // 2. TẦNG FAILSAFE: Nếu mất sóng hoặc dây lỏng, tự động ép về 1500 (giữa)
  if (pI < 950 || pI > 2050) pI = 1500;
  if (rI < 950 || rI > 2050) rI = 1500;

  // 3. TÍNH TOÁN ĐỘ LỆCH (Auto-Center)
  // Khi bạn buông cần, pDiff và rDiff sẽ về 0.
  float pDiff = applyExpo(pI - 1500, EXPO) * RATE;
  float rDiff = applyExpo(rI - 1500, EXPO) * RATE;

  // 4. MIXER ELEVON: Trộn lệnh cho cánh bằng
  // Nếu gạt Pitch lên mà máy bay đi xuống (ngược hướng), hãy đổi dấu + thành - 
  float vL = 1500 + pDiff + rDiff;
  float vR = 1500 - pDiff + rDiff;

  // 5. GIỚI HẠN AN TOÀN (Constrain): Bảo vệ Servo không bị kẹt cơ khí
  vL = constrain(vL, 1150, 1850);
  vR = constrain(vR, 1150, 1850);

  // 6. XUẤT XUNG ĐIỀU KHIỂN
  ledcWrite(L_PIN, (vL / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (vR / 20000.0) * MAX_DUTY);

  // Log dữ liệu để chẩn đoán qua Terminal (tốc độ chậm để không lag)
  if (millis() % 500 == 0) {
    Serial.print("P: "); Serial.print(pI);
    Serial.print(" | R: "); Serial.println(rI);
  }

  delay(10); // Tần số điều khiển 100Hz
}
