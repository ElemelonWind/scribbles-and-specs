#pragma once
#include <Arduino.h>

class PID {
public:
    PID(float kp, float ki, float kd);

    // Computes the controller output for the given setpoint and measurement.
    // dt is the elapsed time since the previous update in seconds.
    float update(float setpoint, float measurement);

    // Resets the integral and derivative history.
    void reset();

private:
    float kp;
    float ki;
    float kd;

    float last_time = 0.0f;

    float integral = 0.0f;
    float previousError = 0.0f;
};