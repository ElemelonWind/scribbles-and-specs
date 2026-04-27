#pragma once
#include <cmath>
#include <Arduino.h>

struct WheelCommand {
    float u0, u1, u2;
};

class LookaheadController {

    // For the current path
    float start_x, start_y;
    float end_x, end_y;

    // Precomputed for lookahead point calculation
    float a;
    float dx, dy;

    // Parameters
    const float lookahead_distance = 0.2; // meters
    const float speed = 0.5f;
    const float erase_speed = 0.8f;
    const float r = 0.02f; // wheel radius

    const float kp_w = 0.03;
    const float d = 0.1f; // Distance from center to wheel

    bool eraseMode = false;

    void calcLookaheadPoint(float curr_x, float curr_y, float& lookahead_x, float& lookahead_y);

public:
    LookaheadController();

    void setEraseMode(bool erase);

    void updatePath(float start_x, float start_y, float end_x, float end_y);

    // Generates desired velocity for Scibbles to follow path.
    WheelCommand command(float curr_x, float curr_y, float curr_theta);
};