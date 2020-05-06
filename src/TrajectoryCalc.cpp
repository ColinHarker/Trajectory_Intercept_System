#include "TrajectoryCalc.hpp"
#include "constants.hpp"

int calculateTrajectory(int p[]) {


    int p_position = p[0], p_velocity = 0, p_s_velocity = 0, p_acceleration = 0, p_s_acceleration;
    int c_position = p[1], c_velocity = 0, c_s_velocity = 0, c_acceleration = 0, c_s_acceleration;
    int f_position[constants::kNumPredictedFrames];


    for (int i = 0; i < constants::kNumCalcFrames - 1; i++) {


        // +Velocity
        if (p[i + 1] >= p[i]) {
            c_velocity = p[i + 1] - p[i];
            c_s_velocity = 1;
        }
        // -Velocity
        if (p[i + 1] < p[i]) {
            c_velocity = p[i] - p[i + 1];
            c_s_velocity = 0;
        }
        // Accelerating
        if (c_velocity >= p_velocity) {
            c_acceleration = c_velocity - p_velocity;
            c_s_acceleration = c_s_velocity;
        }
        // Decelerating
        if (c_velocity < p_velocity) {
            c_acceleration = p_velocity - c_velocity;
            c_s_acceleration = c_s_velocity ^ 1;
        }
        // Inflection Point
        if ((c_s_velocity ^ p_s_velocity) == 1) {
            c_acceleration = c_position + p_position;
            c_s_acceleration = c_s_velocity;
        }

        p_velocity = c_velocity;
        p_acceleration = c_acceleration;
        p_s_velocity = c_s_velocity;
    }
    //Next, the path is predicted_x by incrementally adding the acceleration to the velocity
    //then this sum to the position

    f_position[0] = p[constants::kNumCalcFrames - 2]; //previous
    f_position[1] = p[constants::kNumCalcFrames - 1]; //current

    for (int i = 1; i < constants::kNumPredictedFrames; i++) {
        if (c_s_velocity != 0) {
            if (c_s_acceleration != 0) {
                f_position[i] = f_position[i - 1] + c_velocity + c_acceleration;
            }
            if (c_s_acceleration == 0) {
                f_position[i] = f_position[i - 1] + c_velocity - c_acceleration;
            }
        }
        if (c_s_velocity == 0) {
            if (c_s_acceleration != 0) {
                f_position[i] = f_position[i - 1] - c_velocity + c_acceleration;
            }
            if (c_s_acceleration == 0) {
                f_position[i] = f_position[i - 1] - c_velocity - c_acceleration;
            }
        }
    }

    return f_position[constants::kNumPredictedFrames - 1];
}


