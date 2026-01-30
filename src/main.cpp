#include <Arduino.h>

// Keep WiFi headers out unless you enable them
#define ENABLE_WIFI_UDP   0
#define ENABLE_TELEMETRY  0

#if ENABLE_WIFI_UDP || ENABLE_TELEMETRY
  #include <WiFi.h>
  #include <WiFiUdp.h>
#endif

#include "config.h"
#include "ultrasonic.h"
#include "encoders.h"
#include "Motor.h"
#include "Odometry.h"
#include "Navigation.h"
#include "RadarLink.h"

// =====================
// Settings
// =====================
#define DEBUG_SERIAL_PRINTS  1       // 1 = print US values for tuning
#define USE_RADAR_SAFETY     1       // 1 = radar can hard-stop if danger

static const uint32_t RADAR_TIMEOUT_MS = 300;

// If you do enable WiFi later, fix smart quotes:
static const char* ssid   = "Irving's iPhone";
static const char* pass   = "romero22";
static const char* hostIP = "172.20.10.4";
static const uint16_t hostPort = 5005;

#if ENABLE_WIFI_UDP || ENABLE_TELEMETRY
WiFiUDP Udp;
#endif

// =====================
// Globals
// =====================
RadarLink radar;

// Ultrasonics
Ultrasonic frontUS(US1_TRIG, US1_ECHO);
Ultrasonic leftUS(US2_TRIG, US2_ECHO);
Ultrasonic rightUS(US3_TRIG, US3_ECHO);
Ultrasonic frontRightUS(US4_TRIG, US4_ECHO);
Ultrasonic frontLeftUS(US5_TRIG, US5_ECHO);

// Shared ultrasonic snapshot (cm)
volatile float usFront  = 300;
volatile float usLeft   = 300;
volatile float usRight  = 300;
volatile float usFrontR = 300;
volatile float usFrontL = 300;
portMUX_TYPE usMux = portMUX_INITIALIZER_UNLOCKED;

Motor leftMotor(M1_PWM, M1_INA, M1_INB, M1_EN, CH_M1);
Motor rightMotor(M2_PWM, M2_INA, M2_INB, M2_EN, CH_M2);

Encoder leftEnc(L_ENCA, L_ENCB);
Encoder rightEnc(R_ENCA, R_ENCB);

Odometry odo(&leftEnc, &rightEnc);

// Tuning (yours)
float Kside     = 20.0f;
float Kfront    = 5.0f;
int   baseSpeed = 200;
int   maxSpeed  = 500;
float Kopen     = 0.5f;

// =====================
// Helpers
// =====================
static inline float sanitizeUS(float d) {
  if (d <= 0 || d > 300) return 300;
  return d;
}

static float median3(float a, float b, float c) {
  if (a > b) { float t=a; a=b; b=t; }
  if (b > c) { float t=b; b=c; c=t; }
  if (a > b) { float t=a; a=b; b=t; }
  return b;
}

static float limitRate(float target, float prev, float maxDeltaPerStep) {
  float d = target - prev;
  if (d >  maxDeltaPerStep) d =  maxDeltaPerStep;
  if (d < -maxDeltaPerStep) d = -maxDeltaPerStep;
  return prev + d;
}

static float gateJump(float newV, float prevV, float maxJumpCm) {
  if (fabs(newV - prevV) > maxJumpCm) return prevV;
  return newV;
}

static float readMedianCM(Ultrasonic &us) {
  float a = sanitizeUS(us.getDistanceCM()); delay(8);
  float b = sanitizeUS(us.getDistanceCM()); delay(8);
  float c = sanitizeUS(us.getDistanceCM());
  return median3(a,b,c);
}

static inline void setUS(float f, float l, float r, float fr, float fl) {
  portENTER_CRITICAL(&usMux);
  usFront  = f;
  usLeft   = l;
  usRight  = r;
  usFrontR = fr;
  usFrontL = fl;
  portEXIT_CRITICAL(&usMux);
}

static inline void getUS(float &f, float &l, float &r, float &fr, float &fl) {
  portENTER_CRITICAL(&usMux);
  f  = usFront;
  l  = usLeft;
  r  = usRight;
  fr = usFrontR;
  fl = usFrontL;
  portEXIT_CRITICAL(&usMux);
}

static inline void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}

// =====================
// Ultrasonic sampling (task)
// =====================
void readAllUltrasonicsOnce() {
  static float prevF  = 300, prevL  = 300, prevR  = 300, prevFR = 300, prevFL = 300;

  // Order + delays to reduce crosstalk
  float f  = gateJump(readMedianCM(frontUS),      prevF,  60);  delay(35);
  float fr = gateJump(readMedianCM(frontRightUS), prevFR, 60);  delay(35);
  float r  = gateJump(readMedianCM(rightUS),      prevR,  60);  delay(35);
  float fl = gateJump(readMedianCM(frontLeftUS),  prevFL, 60);  delay(35);
  float l  = gateJump(readMedianCM(leftUS),       prevL,  60);  delay(35);

  prevF = f; prevFR = fr; prevR = r; prevFL = fl; prevL = l;
  setUS(f, l, r, fr, fl);
}

void ultrasonicTask(void* pvParameters) {
  vTaskDelay(200 / portTICK_PERIOD_MS); // settle

  while (true) {
    readAllUltrasonicsOnce();
    // Update rate ~8–12 Hz is enough and reduces crosstalk
    vTaskDelay(80 / portTICK_PERIOD_MS);
  }
}

// =====================
// Navigation using ultrasonics
// =====================
void smoothNavigate(float frontDist, float leftDist, float rightDist) {
  frontDist = sanitizeUS(frontDist);
  leftDist  = sanitizeUS(leftDist);
  rightDist = sanitizeUS(rightDist);

  float L = min(leftDist, 20.0f);
  float R = min(rightDist, 20.0f);

  float speedFactor = 1.0f;
  if (frontDist < 30) speedFactor = frontDist / 30.0f;

  int forwardSpeedTarget = (int)(baseSpeed * speedFactor);
  if (forwardSpeedTarget < 200) forwardSpeedTarget = 200;

  static float forwardPrev = 0;
  float forwardSpeedF = limitRate((float)forwardSpeedTarget, forwardPrev, 30.0f);
  forwardPrev = forwardSpeedF;
  int forwardSpeed = (int)forwardSpeedF;

  float diff = L - R;
  if (fabs(diff) < 5) diff = 0;

  float steer = diff * (Kside * 1.2f);

  float openSteer = (rightDist - leftDist) * Kopen;
  steer += openSteer;

  if (frontDist < 100)
    steer += (100 - frontDist) * (Kfront * 0.8f);

  static float steerPrev = 0;
  steer = limitRate(steer, steerPrev, 30.0f);
  steerPrev = steer;

  int leftSpeed  = (int)(forwardSpeed + steer);
  int rightSpeed = (int)(forwardSpeed - steer);

  leftSpeed  = constrain(leftSpeed,  -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  if (leftSpeed >= 0) leftMotor.forward(leftSpeed);
  else                leftMotor.reverse(-leftSpeed);

  if (rightSpeed >= 0) rightMotor.forward(rightSpeed);
  else                 rightMotor.reverse(-rightSpeed);
}

// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);
  radar.begin(Serial);

#if ENABLE_WIFI_UDP
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    yield();
  }
  delay(300);
  Udp.begin(hostPort);
#endif

  leftEnc.begin();
  rightEnc.begin();

  frontUS.begin();
  leftUS.begin();
  rightUS.begin();
  frontRightUS.begin();
  frontLeftUS.begin();

  leftMotor.begin();
  rightMotor.begin();

  // Ultrasonic sampling task owns pulseIn()
  xTaskCreatePinnedToCore(ultrasonicTask, "US", 4096, NULL, 1, NULL, 0);
}

// =====================
// Loop: ultrasonics drive, radar optional safety
// =====================
void loop() {
  odo.update();

  // ---- Radar handling (optional) ----
  radar.poll();
  const bool radar_ok = radar.hasFresh(RADAR_TIMEOUT_MS);
  const auto& rd = radar.data();

  // Print ACK for your logger (5 Hz) if radar is feeding us data
  static uint32_t lastAck = 0;
  if (radar_ok && (millis() - lastAck) > 200) {
    lastAck = millis();
    Serial.printf("ACK,%lu,%.2f,%.2f,%.2f\n",
                  (unsigned long)rd.seq, rd.range_m, rd.vel_mps, rd.conf);
  }

#if USE_RADAR_SAFETY
  const bool radar_danger =
      (radar_ok && rd.conf > 0.6f && rd.range_m < 0.8f && rd.vel_mps < -0.10f);
  if (radar_danger) {
    stopMotors();
    delay(20);
    return;
  }
#endif

  // ---- Ultrasonics snapshot ----
  float f, l, r, fr, fl;
  getUS(f, l, r, fr, fl);

  // Hazards (cm)
  float frontHazard = median3(f, fl, fr);              // uses new angled sensors
  float leftHazard  = median3(fl, l, 300.0f);
  float rightHazard = median3(fr, r, 300.0f);

  // Basic stop/TTC (cm, cm/s)
  float v = max(odo.getSpeedCms(), 1.0f);
  float stopBuffer = 25.0f;
  float ttc = max(frontHazard - stopBuffer, 0.0f) / v;

  if (frontHazard < stopBuffer || ttc < 0.7f) {
    stopMotors();
    delay(20);
    return;
  }

  // Navigate based on fused ultrasonic hazards
  smoothNavigate(frontHazard, leftHazard, rightHazard);

#if DEBUG_SERIAL_PRINTS
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("US f=%.1f fl=%.1f l=%.1f fr=%.1f r=%.1f | Haz F=%.1f L=%.1f R=%.1f\n",
                  f, fl, l, fr, r, frontHazard, leftHazard, rightHazard);
  }
#endif

  delay(20); // ~50 Hz control loop
}
