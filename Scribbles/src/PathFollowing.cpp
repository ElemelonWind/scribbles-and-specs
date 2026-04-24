#include "PathFollowing.hpp"

constexpr float d = 0.1f; // Distance from center to wheel

LookaheadController::LookaheadController() {

}

void LookaheadController::updatePath(float _start_x, float _start_y, float _end_x, float _end_y) {
    start_x = _start_x;
    start_y = _start_y;
    end_x = _end_x;
    end_y = _end_y;
    Serial.printf("UpdatePath with start_x = %f start_y = %f end_x = %f end_y = %f\n", start_x, start_y, end_x, end_y);

    a = (end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y);
    dx = end_x - start_x;
    dy = end_y - start_y;
}   


WheelCommand LookaheadController::command(float curr_x, float curr_y, float curr_theta) {
    float lookahead_x, lookahead_y;
    calcLookaheadPoint(curr_x, curr_y, lookahead_x, lookahead_y);

    float desired_theta = 0;
    float error_theta = desired_theta - curr_theta;

    // Normalize error to [-pi, pi]
    while (error_theta > M_PI) error_theta -= 2 * M_PI;
    while (error_theta < -M_PI) error_theta += 2 * M_PI;

    float vel_x, vel_y, w;
    vel_x = lookahead_x - curr_x;
    vel_y = lookahead_y - curr_y;
    w = error_theta;


    float mod_vel = sqrt(vel_x * vel_x + vel_y * vel_y);
    vel_x = (vel_x / mod_vel) * speed;
    vel_y = (vel_y / mod_vel) * speed;
    w = 0.1 * w;

    float u0 = (1/r) * d * w + vel_x;
    float u1 = (1/r) * d * w - 0.5f * vel_x - (sqrt(3) / 2) * vel_y;
    float u2 = (1/r) * d * w - 0.5f * vel_x + (sqrt(3) / 2) * vel_y;
    return {u0, u1, u2};

}
void LookaheadController::calcLookaheadPoint(float curr_x, float curr_y, float& lookahead_x, float& lookahead_y) {

    float b = 2 * (start_x * (dx) + start_y * (dy) - curr_x * (dx) - curr_y * (dy));
    float c = (start_x - curr_x) * (start_x - curr_x) + (start_y - curr_y) * (start_y - curr_y) - lookahead_distance * lookahead_distance;
    Serial.println("X goal:" + String(end_x) + " Y goal: " + String(end_y) + " X Current" + String(curr_x) + " Y Current" + String(curr_y));
    if (b * b - 4 * a * c < 0) {
        // No intersection, lookahead point is end
        lookahead_x = end_x;
        lookahead_y = end_y;
        return;
    }

    float sqrt_disc = sqrt(b * b - 4 * a * c);
    float mu = (-b + sqrt_disc) / (2 * a);
    lookahead_x = start_x + mu * dx;
    lookahead_y = start_y + mu * dy;

    Serial.println("Lookahead Point: (" + String(lookahead_x) + ", " + String(lookahead_y) + ")");
}