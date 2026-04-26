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

    while (curr_theta < -M_PI) curr_theta += 2 * M_PI;
    while (curr_theta > M_PI) curr_theta -= 2 * M_PI;

    float desired_theta = 0;
    float error_theta = desired_theta - curr_theta;


    float vel_x_global, vel_y_global, w;
    vel_x_global = lookahead_x - curr_x;
    vel_y_global = lookahead_y - curr_y;
    w = error_theta;


    float mod_vel = sqrt(vel_x_global * vel_x_global + vel_y_global * vel_y_global);

    float adj_speed = speed;
    float dist_to_end = sqrt((end_x - curr_x) * (end_x - curr_x) + (end_y - curr_y) * (end_y - curr_y));
    if (dist_to_end < 0.05) {
        adj_speed = (dist_to_end / 0.05) * speed;
    }

    vel_x_global = (vel_x_global / mod_vel) * adj_speed;
    vel_y_global = (vel_y_global / mod_vel) * adj_speed;

    // Convert global velocities to robot frame
    float vel_x = cos(curr_theta) * vel_x_global - sin(curr_theta) * vel_y_global;
    float vel_y = sin(curr_theta) * vel_x_global + cos(curr_theta) * vel_y_global;
    w = kp_w * w;

    float u0 = (1/r) * d * w + vel_x;
    float u1 = (1/r) * d * w - 0.5f * vel_x + (sqrt(3) / 2) * vel_y;
    float u2 = (1/r) * d * w - 0.5f * vel_x - (sqrt(3) / 2) * vel_y;
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