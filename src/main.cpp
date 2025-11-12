#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "ultrasonic.h"
#include "encoders.h"
#include "Motor.h"
#include "Odometry.h"

const char* ssid = "Irving’s iPhone";
const char* pass = "romero22";
const char* hostIP = "172.20.10.4";  // <-- your PC or phone IP
const uint16_t hostPort = 5005;

// --- Objects ---
WiFiUDP Udp;
Ultrasonic frontUS(US1_TRIG, US1_ECHO);
Ultrasonic leftUS(US2_TRIG, US2_ECHO);
Ultrasonic rightUS(US3_TRIG, US3_ECHO);
Motor leftMotor(M1_PWM, M1_INA, M1_INB, M1_EN, CH_M1);
Motor rightMotor(M2_PWM, M2_INA, M2_INB, M2_EN, CH_M2);
Encoder leftEnc(L_ENCA, L_ENCB);
Encoder rightEnc(R_ENCA, R_ENCB);
Odometry odo(&leftEnc, &rightEnc);


// --- Telemetry task (runs on Core 0) ---
void telemetryTask(void* pvParameters) {
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      char buf[128];
      snprintf(buf, sizeof(buf), "%f,%f,%f,%f,%f,%f\n",
               odo.getX(), odo.getY(), odo.getTheta(),
               frontUS.getDistanceCM(), leftUS.getDistanceCM(), rightUS.getDistanceCM());
      Udp.beginPacket(hostIP, hostPort);
      Udp.write((uint8_t*)buf, strlen(buf));
      Udp.endPacket();
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);  // send every 0.5 s
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);


  // --- Wi-Fi ---
  WiFi.begin(ssid, pass);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 7000) {
 
    delay(250);
  }
  
  delay(500);  // let lwIP finish
  Udp.begin(hostPort);

  // --- Start other hardware ---
  leftEnc.begin();
  rightEnc.begin();
  frontUS.begin();
  leftUS.begin();
  rightUS.begin();
  leftMotor.begin();
  rightMotor.begin();
 

  // --- Launch telemetry in the background ---
  xTaskCreatePinnedToCore(telemetryTask, "Telemetry", 4096, NULL, 1, NULL, 0);

  Serial.println("Setup complete!");
}

// --- Main loop (Core 1) ---
void loop() {
  odo.update();

  float frontDist = frontUS.getDistanceCM();
  float leftDist  = leftUS.getDistanceCM();
  float rightDist = rightUS.getDistanceCM();

  // --- Obstacle avoidance ---
  if (frontDist > 0 && frontDist < 20) {
    leftMotor.stop();
    rightMotor.stop();
    delay(200);

    if (leftDist > rightDist) {
      leftMotor.reverse(400);
      rightMotor.forward(400);
      delay(400);
    } else {
      leftMotor.forward(400);
      rightMotor.reverse(400);
      delay(400);
    }
  } else {
    leftMotor.forward(200);
    rightMotor.forward(200);
  }

  // --- Debug print ---
  Serial.printf("X: %.2f Y: %.2f θ: %.2f\n", odo.getX(), odo.getY(), odo.getTheta());
  delay(20);  // fast control loop (~50 Hz)
}
