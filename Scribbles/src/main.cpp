#include <Arduino.h>
#include "HBridgeMotor.hpp"
#include <ESP32Encoder.h>
#include "PID.hpp"
#include "WheelSpeed.hpp"
#include "ESP32_PWM.h"

HBridgeMotor motor0(5, 18, 19);
HBridgeMotor motor1(32, 33, 25);
HBridgeMotor motor2(26, 27, 14);

PID pid0(1.0, 0.0, 0.0);
PID pid1(1.0, 0.0, 0.0);
PID pid2(1.0, 0.0, 0.0);


WheelSpeed wheelSpeed0;
WheelSpeed wheelSpeed1;
WheelSpeed wheelSpeed2;

float targetSpeed = 0.2f;


void setup() {

  wheelSpeed0 = WheelSpeed(2, 4);
  wheelSpeed1 = WheelSpeed(13, 17);
  wheelSpeed2 = WheelSpeed(22, 23);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
  // Encoder - PID test

  motor0.set(pid0.update(targetSpeed, wheelSpeed0.calculateSpeed()));
  motor1.set(pid1.update(targetSpeed, wheelSpeed1.calculateSpeed()));
  motor2.set(pid2.update(targetSpeed, wheelSpeed2.calculateSpeed()));

  Serial.print(wheelSpeed0.calculateSpeed());
  Serial.print(", ");
  Serial.print(wheelSpeed1.calculateSpeed());
  Serial.print(", ");
  Serial.println(wheelSpeed2.calculateSpeed());
}
