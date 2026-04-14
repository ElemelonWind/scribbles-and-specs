#pragma once
#include <Arduino.h>
#include <ESP32_PWM.hpp>

constexpr bool FORWARD = true;
constexpr bool REVERSE = false;

class HBridgeMotor {

public:
    HBridgeMotor() = delete;
    // Initializes the motor with the given pins for direction and speed control
    HBridgeMotor(int pin1, int pin2, int pinSpeed);

    // Sets the motor speed and direction.
    // @arg speed: A value between 0.0 (stopped) and 1.0 (full speed).
    // @arg direction: true for forward, false for reverse.
    void set(float speed, bool direction);

    // Sets the motor speed and direction.
    // @arg speed: A value between -1.0 (full speed reverse) and 1.0 (full speed forward).
    void set(float speed);

    // Stops the motor by turning it off.
    void stop();

    
private:
    int pin1, pin2, pinSpeed;
    int timerNo;

    
};