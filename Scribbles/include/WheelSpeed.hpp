#pragma once
#include <ESP32Encoder.h>
#include <Arduino.h>


class WheelSpeed {
public:

    WheelSpeed() = default;
    WheelSpeed(int encoderPinA, int encoderPinB);

    float calculateSpeed();

private:
    ESP32Encoder encoder;
    float last_time = 0.0f;
};