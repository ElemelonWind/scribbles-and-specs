#include "WheelSpeed.hpp"

constexpr float GEAR_RATIO = 78.0;
constexpr float ENCODER_RES = 48.0;


WheelSpeed::WheelSpeed(int encoderPinA, int encoderPinB) {
    encoder.attachFullQuad(encoderPinA, encoderPinB);
    encoder.clearCount();
}

float WheelSpeed::calculateSpeed() {
    float current_time = millis() / 1000.0f; // Convert to seconds
    float dt = current_time - last_time;
    last_time = current_time;

    int64_t count = encoder.getCount();

    float rev = count / (ENCODER_RES * GEAR_RATIO); // Number of wheel revolutions
    encoder.clearCount();

    return rev / dt;
}