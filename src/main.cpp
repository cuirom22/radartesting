#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "ultrasonic.h"
#include "encoders.h"
#include "Motor.h"
#include "ultrasonic.h"
#include "Odometry.h"
WiFiUDP Udp;

//heyyy


const char* ssid = "Irving's iPhone";
const char* pass = "romero22";
const char* hostIP = "10.110.253.18";   // <-- your PC IP
const uint16_t hostPort = 5005;


Ultrasonic frontUS(US1_TRIG, US1_ECHO);
Ultrasonic leftUS(US2_TRIG, US2_ECHO);
Ultrasonic rightUS(US3_TRIG, US3_ECHO);

Motor leftMotor(M1_PWM, M1_INA, M1_INB, M1_EN, CH_M1);
Motor rightMotor(M2_PWM, M2_INA, M2_INB, M2_EN, CH_M2);

Encoder leftEnc(L_ENCA, L_ENCB);
Encoder rightEnc(R_ENCA, R_ENCB);

Odometry odo(&leftEnc, &rightEnc);

void setup() {
  Serial.begin(115200);
  Serial.println("Robot System Initializing...");
  //wifi stuff
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.println("WiFi connected");
  Udp.begin(hostPort); // optional for send-only
  // Encoder Setup
  leftEnc.begin();
  rightEnc.begin();
// Ultrasonic setup
  frontUS.begin();
  leftUS.begin();
  rightUS.begin();
  //Motor setup 
  leftMotor.begin();
  rightMotor.begin();
  delay(1000);
  Serial.println("Setup complete!");

}

void loop() {

  // update odo sensors
  odo.update();
  //wifi stuff
  char buf[128];
  snprintf(buf, sizeof(buf), "%f,%f,%f,%f,%f,%f\n",
           odo.getX(), odo.getY(), odo.getTheta(),
           frontUS.getDistanceCM(), leftUS.getDistanceCM(), rightUS.getDistanceCM());
  Udp.beginPacket(hostIP, hostPort);
  Udp.write((uint8_t*)buf, strlen(buf));
  Udp.endPacket();


  // --- Read sensors ---
  float frontDist = frontUS.getDistanceCM();
  float leftDist  = leftUS.getDistanceCM();
  float rightDist = rightUS.getDistanceCM();

  // --- Display ultrasonics
  Serial.print("Front: "); Serial.print(frontDist); Serial.print(" cm  ");
  Serial.print("Left: ");  Serial.print(leftDist);  Serial.print(" cm  ");
  Serial.print("Right: "); Serial.print(rightDist); Serial.print(" cm  ");
  //display encoder counts
  Serial.print(" | L Count: "); Serial.print(leftEnc.getCount());
  Serial.print(" | R Count: "); Serial.println(rightEnc.getCount());
  //display odo calculations
  Serial.print("X: "); Serial.print(odo.getX());
  Serial.print(" cm, Y: "); Serial.print(odo.getY());
  Serial.print(" cm, θ: "); Serial.println(odo.getTheta() * 180.0 / PI);
  

  // --- Basic obstacle avoidance ---
  if (frontDist > 0 && frontDist < 20) {  // obstacle in front
    leftMotor.stop();
    rightMotor.stop();
    Serial.println("Obstacle detected! Stopping...");
    delay(500);

    // Turn toward clearer side
    if (leftDist > rightDist) {
      Serial.println("Turning left...");
      leftMotor.reverse(400);
      rightMotor.forward(400);
      delay(400);
    } else {
      Serial.println("Turning right...");
      leftMotor.forward(400);
      rightMotor.reverse(400);
      delay(400);
    }
  } 
  else {
    // Move forward
    leftMotor.forward(512);
    rightMotor.forward(512);
  }

  delay(200);
}