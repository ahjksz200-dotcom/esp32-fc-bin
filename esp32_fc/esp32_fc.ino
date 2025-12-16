#include <Arduino.h>
#include <Wire.h>

/* ================== PIN MAP ================== */
#define PIN_ESC      25
#define PIN_SERVO_L  26
#define PIN_SERVO_R  27
#define PIN_RUDDER   14

#define MBUS_RX_PIN  16

/* ================== PWM ================== */
#define ESC_FREQ   400
#define SERVO_FREQ 50
#define PWM_RES    16

/* ================== FAILSAFE ================== */
#define FAILSAFE_TIMEOUT 100
#define THROTTLE_CUT_US 1000

/* ================== MPU ================== */
#define MPU_ADDR 0x68

/* ================== SERIAL ================== */
HardwareSerial MBUS(2);

/* ================== GLOBAL ================== */
uint16_t rc[16];
unsigned long lastMbus = 0;
bool failsafe = true;
bool armed = false;

/* ================== MPU DATA ================== */
float gyroX, gyroY;
float accX, accY, accZ;
float anglePitch = 0, angleRoll = 0;

/* ================== PWM UTILS ================== */
uint32_t usToDuty(uint32_t us, uint32_t freq) {
  uint32_t maxDuty = (1UL << PWM_RES) - 1;
  uint32_t period = 1000000UL / freq;
  return (uint32_t)((float)us / period * maxDuty);
}

void writePWM(uint8_t pin, uint32_t us, uint32_t freq) {
  ledcWrite(pin, usToDuty(us, freq));
}

/* ================== RC ================== */
uint16_t rcToUs(uint16_t v) {
  return map(v, 172, 1811, 1000, 2000);
}

/* ================== M.BUS ================== */
bool readMBUS() {
  static uint8_t buf[32];
  static uint8_t idx = 0;

  while (MBUS.available()) {
    buf[idx++] = MBUS.read();
    if (idx >= 25) {
      idx = 0;
      if (buf[0] != 0x0F) return false;

      for (int i = 0; i < 16; i++)
        rc[i] = (buf[1+i*2] | (buf[2+i*2]<<8)) & 0x07FF;

      lastMbus = millis();
      failsafe = false;
      return true;
    }
  }
  return false;
}

/* ================== MPU ================== */
void mpuInit() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readMPU(float dt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX = (Wire.read()<<8 | Wire.read()) / 16384.0;
  accY = (Wire.read()<<8 | Wire.read()) / 16384.0;
  accZ = (Wire.read()<<8 | Wire.read()) / 16384.0;
  Wire.read(); Wire.read();

  gyroX = (Wire.read()<<8 | Wire.read()) / 131.0;
  gyroY = (Wire.read()<<8 | Wire.read()) / 131.0;
  Wire.read(); Wire.read();

  anglePitch += gyroY * dt;
  angleRoll  += gyroX * dt;

  float accPitch = atan2(accY, accZ) * 57.3;
  float accRoll  = atan2(accX, accZ) * 57.3;

  anglePitch = anglePitch * 0.98 + accPitch * 0.02;
  angleRoll  = angleRoll  * 0.98 + accRoll  * 0.02;
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);
  MBUS.begin(115200, SERIAL_8E1, MBUS_RX_PIN, -1);

  ledcAttach(PIN_ESC, ESC_FREQ, PWM_RES);
  ledcAttach(PIN_SERVO_L, SERVO_FREQ, PWM_RES);
  ledcAttach(PIN_SERVO_R, SERVO_FREQ, PWM_RES);
  ledcAttach(PIN_RUDDER, SERVO_FREQ, PWM_RES);

  writePWM(PIN_ESC, THROTTLE_CUT_US, ESC_FREQ);
  writePWM(PIN_SERVO_L, 1500, SERVO_FREQ);
  writePWM(PIN_SERVO_R, 1500, SERVO_FREQ);
  writePWM(PIN_RUDDER, 1500, SERVO_FREQ);

  mpuInit();
  Serial.println("ESP32 FC READY");
}

/* ================== LOOP ================== */
void loop() {
  static unsigned long lastLoop = micros();
  unsigned long now = micros();
  float dt = (now - lastLoop) * 1e-6;
  lastLoop = now;

  readMBUS();

  if (millis() - lastMbus > FAILSAFE_TIMEOUT) {
    failsafe = true;
    armed = false;
  }

  if (!failsafe) {
    armed = rc[6] > 1000;   // CH7 ARM
    readMPU(dt);
  }

  uint16_t throttle = armed ? rcToUs(rc[2]) : THROTTLE_CUT_US;
  uint16_t pitch = rcToUs(rc[1]) - anglePitch * 10;
  uint16_t roll  = rcToUs(rc[0]) - angleRoll  * 10;
  uint16_t yaw   = rcToUs(rc[3]);

  writePWM(PIN_ESC, throttle, ESC_FREQ);
  writePWM(PIN_SERVO_L, constrain(pitch + roll, 1000, 2000), SERVO_FREQ);
  writePWM(PIN_SERVO_R, constrain(pitch - roll, 1000, 2000), SERVO_FREQ);
  writePWM(PIN_RUDDER, yaw, SERVO_FREQ);

  delay(2);
}
