#include "PID.hpp"

PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {

}

float PID::update(float setpoint, float measurement) {

    float current_time = millis() / 1000.0f; // Convert to seconds
    float dt = current_time - last_time;
    last_time = current_time;

    float error = setpoint - measurement;
    integral += error * dt;

    float derivative = 0.0f;
    derivative = (error - previousError) / dt;
    
    previousError = error;

    return kp * error + ki * integral + kd * derivative;
}

void PID::reset() {
    integral = 0.0f;
    previousError = 0.0f;
}