#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- CÀI ĐẶT BIỆN PHÁP MẠNH ---
int GIOI_HAN_HANH_TRINH = 200; // Xung từ giữa (1500) chỉ được đi tối đa +/- 200 (Cực kỳ an toàn)
int VUNG_CHET = 60;            // Thả tay ra là CHẾT CỨNG ở giữa
int TRIM_L = 1500;             
int TRIM_R = 1500;             
float DO_MUOT_EXPO = 0.7;      // Làm mượt cực mạnh

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;
long midP = 1500, midR = 1500;
unsigned long lastSignalTime = 0;

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLUP);
  pinMode(R_IN, INPUT_PULLUP);

  Serial.println(">>> DANG EP CALIB... KHONG DUOC DONG CAN!");
  delay(3000); 
  
  // Đọc 100 lần lấy trung bình cho siêu chuẩn
  long sP = 0, sR = 0;
  for(int i=0; i<100; i++) {
    long p = pulseIn(P_IN, HIGH, 30000);
    long r = pulseIn(R_IN, HIGH, 30000);
    sP += (p > 0) ? p : 1500;
    sR += (r > 0) ? r : 1500;
    delay(5);
  }
  midP = sP / 100; midR = sR / 100;
  Serial.printf("TAM CHUAN: P=%ld R=%ld\n", midP, midR);
}

void loop() {
  // Đọc xung
  long pI = pulseIn(P_IN, HIGH, 25000); 
  long rI = pulseIn(R_IN, HIGH, 25000);

  // --- BIỆN PHÁP MẠNH 1: FAILSAFE THỜI GIAN THỰC ---
  if (pI > 900 && pI < 2100) {
    lastSignalTime = millis(); // Cập nhật thời điểm có sóng
  }

  // Nếu quá 100ms không có sóng mới hoặc pulseIn trả về 0 -> FAILSAFE
  bool forcedFailsafe = (millis() - lastSignalTime > 100 || pI == 0);

  float pFinal = 0, rFinal = 0;

  if (!forcedFailsafe) {
    // --- BIỆN PHÁP MẠNH 2: AUTO CENTER "BÀN TAY SẮT" ---
    long pRaw = pI - midP;
    long rRaw = rI - midR;

    // Nếu nằm trong vùng chết -> Xóa sổ mọi cử động
    if (abs(pRaw) < VUNG_CHET) pRaw = 0;
    if (abs(rRaw) < VUNG_CHET) rRaw = 0;

    // Tính Expo
    float inP = constrain(pRaw / 500.0, -1.0, 1.0);
    float expoP = ((1 - DO_MUOT_EXPO) * inP + DO_MUOT_EXPO * (inP * inP * inP)) * 500.0;
    
    float inR = constrain(rRaw / 500.0, -1.0, 1.0);
    float expoR = ((1 - DO_MUOT_EXPO) * inR + DO_MUOT_EXPO * (inR * inR * inR)) * 500.0;

    // --- BIỆN PHÁP MẠNH 3: GIỚI HẠN GÓC CƯỠNG BỨC ---
    pFinal = constrain(expoP, -GIOI_HAN_HANH_TRINH, GIOI_HAN_HANH_TRINH);
    rFinal = constrain(expoR, -GIOI_HAN_HANH_TRINH, GIOI_HAN_HANH_TRINH);
  } else {
    // KHI FAILSAFE: Đưa mọi giá trị về 0 tuyệt đối
    pFinal = 0;
    rFinal = 0;
    if (millis() % 500 == 0) Serial.println("!!! CƯỠNG BỨC FAILSAFE !!!");
  }

  // MIXER
  float vL = (float)TRIM_L + pFinal + rFinal;
  float vR = (float)TRIM_R - pFinal + rFinal;

  // XUẤT XUNG
  ledcWrite(L_PIN, (constrain(vL, 1000, 2000) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1000, 2000) / 20000.0) * MAX_DUTY);

  if (millis() % 200 == 0 && !forcedFailsafe) {
    Serial.printf("P_Final: %.2f | AutoCenter: %s\n", pFinal, (pFinal == 0 ? "LOCKED" : "MOVING"));
  }
}
