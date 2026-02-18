#include <Arduino.h>

#define L_PIN 13
#define R_PIN 12
#define P_IN 14
#define R_IN 27

// --- QUẢN LÝ THÔNG MINH ---
int GIOI_HAN = 250;      // Giới hạn hành trình tuyệt đối
int VUNG_CHET_AI = 70;   // AI sẽ tự khóa Center nếu tín hiệu nằm trong vùng này
int TRIM_L = 1500, TRIM_R = 1500;

const int FREQ = 50, RES = 16, MAX_DUTY = 65535;
long midP = 1500, midR = 1500;

// Biến quản lý trạng thái AI
enum SystemState {NORMAL, CENTER_LOCK, FAILSAFE};
SystemState currentState = NORMAL;

unsigned long lastValidPulse = 0;
long prevP = 0;

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN, FREQ, RES);
  ledcAttach(R_PIN, FREQ, RES);
  pinMode(P_IN, INPUT_PULLUP);
  pinMode(R_IN, INPUT_PULLUP);

  Serial.println(">>> AI CALIBRATING...");
  delay(3000); 
  
  // Thu thập dữ liệu mẫu để AI học "điểm tĩnh"
  long sP = 0;
  for(int i=0; i<100; i++) {
    long p = pulseIn(P_IN, HIGH, 30000);
    sP += (p > 0) ? p : 1500;
    delay(2);
  }
  midP = sP / 100;
  midR = midP; // Giả định kênh Roll tương đương hoặc bạn có thể calib riêng
}

void loop() {
  long pI = pulseIn(P_IN, HIGH, 25000);
  long rI = pulseIn(R_IN, HIGH, 25000);

  // --- 1. AI FAILSAFE MANAGER (Quản lý mất sóng) ---
  // AI nhận biết dựa trên: Thời gian mất xung HOẶC Xung bị đứng im (Frozen)
  if (pI > 900 && pI < 2100 && pI != prevP) {
    lastValidPulse = millis();
    currentState = NORMAL;
  } else if (millis() - lastValidPulse > 150) {
    currentState = FAILSAFE;
  }
  prevP = pI;

  float pOut = 0, rOut = 0;

  if (currentState == FAILSAFE) {
    // Ép về 0 ngay lập tức khi AI nhận diện mất sóng
    pOut = 0; rOut = 0;
    if(millis() % 500 == 0) Serial.println("[AI] STATUS: FAILSAFE!");
  } 
  else {
    // --- 2. AI CENTER MANAGER (Quản lý vị trí giữa) ---
    long diffP = pI - midP;
    long diffR = rI - midR;

    // AI phân tích: Nếu độ lệch nhỏ, kích hoạt CENTER_LOCK
    if (abs(diffP) < VUNG_CHET_AI && abs(diffR) < VUNG_CHET_AI) {
      currentState = CENTER_LOCK;
      pOut = 0; rOut = 0;
    } else {
      // Nếu thoát vùng chết, áp dụng EXPO làm mượt
      currentState = NORMAL;
      float inP = constrain(diffP / 500.0, -1.0, 1.0);
      pOut = (0.3 * inP + 0.7 * (inP * inP * inP)) * 500.0; // Expo 0.7
      
      float inR = constrain(diffR / 500.0, -1.0, 1.0);
      rOut = (0.3 * inR + 0.7 * (inR * inR * inR)) * 500.0;
    }
  }

  // --- 3. GIỚI HẠN GÓC CƯỠNG BỨC ---
  pOut = constrain(pOut, -GIOI_HAN, GIOI_HAN);
  rOut = constrain(rOut, -GIOI_HAN, GIOI_HAN);

  // MIXER
  float vL = TRIM_L + pOut + rOut;
  float vR = TRIM_R - pOut + rOut;

  ledcWrite(L_PIN, (constrain(vL, 1000, 2000) / 20000.0) * MAX_DUTY);
  ledcWrite(R_PIN, (constrain(vR, 1000, 2000) / 20000.0) * MAX_DUTY);

  // Theo dõi AI làm việc
  if (millis() % 200 == 0) {
    Serial.print("MODE: "); 
    if(currentState == CENTER_LOCK) Serial.println("CENTER_LOCKED");
    else if(currentState == NORMAL) Serial.println("MANUAL");
    else Serial.println("FAILSAFE");
  }
}
