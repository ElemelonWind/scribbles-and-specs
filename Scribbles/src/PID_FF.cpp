#include "PID_FF.hpp"

PID_FF::PID_FF(float m, float b, float kp, float ki, float kd) : m(m), b(b), kp(kp), ki(ki), kd(kd) {

}

float PID_FF::update(float setpoint, float measurement) {

    float current_time = millis() / 1000.0f; // Convert to seconds
    float dt = current_time - last_time;
    last_time = current_time;

    float error = setpoint - measurement;
    integral += error * dt;

    float derivative = 0.0f;
    derivative = (error - previousError) / dt;
    
    previousError = error;

    // Serial.println("Error: " + String(error) + ", Integral: " + String(integral) + ", Derivative: " + String(derivative));

    float ff_term = m * abs(setpoint) + b;
    if (setpoint < 0) ff_term = -ff_term; // Preserve direction for feedforward term
    float u = ff_term + kp * error + ki * integral + kd * derivative;
    // Serial.println("Control Output (u): " + String(u));
    return u;
}

void PID_FF::reset() {
    integral = 0.0f;
    previousError = 0.0f;
}