#include "PathFollowing.hpp"

PurePursuitController::PurePursuitController() {

}

void PurePursuitController::updatePath(float start_x, float start_y, float end_x, float end_y) {
    start_x = start_x;
    start_y = start_y;
    end_x = end_x;
    end_y = end_y;

    a = (end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y);
    dx = end_x - start_x;
    dy = end_y - start_y;
}   


WheelCommand PurePursuitController::command(float curr_x, float curr_y, float curr_theta) {

}
void PurePursuitController::calcLookaheadPoint(float curr_x, float curr_y, float& lookahead_x, float& lookahead_y) {

    float b = 2 * (start_x * (dx) + start_y * (dy) - curr_x * (dx) - curr_y * (dy));
    float c = (start_x - curr_x) * (start_x - curr_x) + (start_y - curr_y) * (start_y - curr_y) - lookahead_distance * lookahead_distance;
    
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
}