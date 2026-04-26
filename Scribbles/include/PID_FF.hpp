#pragma once
#include <Arduino.h>

class PID_FF {
public:
    PID_FF(float m, float b, float kp, float ki, float kd);

    // Computes the controller output for the given setpoint and measurement.
    // dt is the elapsed time since the previous update in seconds.

    // Calculates feedforward term as output = m * setpoint + b, where m is the slope and b is the intercept.
    // Adds an error PID term.

    float update(float setpoint, float measurement);

    // Resets the integral and derivative history.
    void reset();

private:
    float m;
    float b;
    float kp;
    float ki;
    float kd;

    float last_time = 0.0f;

    float integral = 0.0f;
    float previousError = 0.0f;
};