#include <Arduino.h>
#include "HBridgeMotor.hpp"
#include <ESP32Encoder.h>
#include "PID_FF.hpp"
#include "WheelSpeed.hpp"
#include "ESP32_PWM.h"

int cnt = 0;

HBridgeMotor motor0(27, 25, 26);
HBridgeMotor motor1(13, 12, 23);
HBridgeMotor motor2(32, 33, 18);

PID_FF pid0(0.08929, 0.0188, 0.02, 0.1, 0.002);
PID_FF pid1(0.08105, 0.02559, 0.02, 0.1, 0.002);
PID_FF pid2(0.08084, 0.02601, 0.02, 0.1, 0.002);


WheelSpeed wheelSpeed0;
WheelSpeed wheelSpeed1;
WheelSpeed wheelSpeed2;

float targetSpeed = 0.5f;


void setup() {

  wheelSpeed0 = WheelSpeed(4, 5);
  wheelSpeed1 = WheelSpeed(21, 22);
  wheelSpeed2 = WheelSpeed(14, 19);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
  // Encoder - PID_FF test

  if (cnt > 25) {
    motor0.stop();
    motor1.stop();
    motor2.stop();
    return;
  }
  cnt++;

  float wheel0Speed = wheelSpeed0.calculateSpeed();
  float wheel1Speed = wheelSpeed1.calculateSpeed();
  float wheel2Speed = wheelSpeed2.calculateSpeed();

  float motor0Output = pid0.update(targetSpeed, wheel0Speed);
  float motor1Output = pid1.update(targetSpeed, wheel1Speed);
  float motor2Output = pid2.update(targetSpeed, wheel2Speed);

  if (abs(motor0Output) < 0.05f) motor0.stop();
  else motor0.set(abs(motor0Output), motor0Output >= 0.0f);

  if (abs(motor1Output) < 0.05f) motor1.stop();
  else motor1.set(abs(motor1Output), motor1Output >= 0.0f);

  if (abs(motor2Output) < 0.05f) motor2.stop();
  else motor2.set(abs(motor2Output), motor2Output >= 0.0f);

  delay(200);
}
