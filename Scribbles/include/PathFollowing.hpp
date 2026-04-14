#pragma once
#include <cmath>

struct WheelCommand {
    float u0, u1, u2;
};

class PurePursuitController {

    // For the current path
    float start_x, start_y;
    float end_x, end_y;

    // Precomputed for lookahead point calculation
    float a;
    float dx, dy;

    // Parameters
    float lookahead_distance = 0.1; // meters

    void calcLookaheadPoint(float curr_x, float curr_y, float& lookahead_x, float& lookahead_y);

public:
    PurePursuitController();

    void updatePath(float start_x, float start_y, float end_x, float end_y);

    // Generates desired velocity for Scibbles to follow path.
    WheelCommand command(float curr_x, float curr_y, float curr_theta);
};