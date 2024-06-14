#ifndef __GAIT_GENERATOR_H
#define __GAIT_GENERATOR_H

// #include "leg_controller.h"
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#define swing_leg 0
#define stance_leg 1
#define Lose_Contact 2
#define Early_Contact 3

#define t_stance 0.25

class gait_generator{
public:
    gait_generator();
    void update(float current_time);
    Eigen::Vector2d stance_duration;
    Eigen::Vector2d normalized_phase;
    std::vector<int> leg_state = {0,1};
    std::vector<int> desired_leg_state = {0,0};
private:
    Eigen::Vector2d swing_duration;
    Eigen::Vector2d stance_dutyrate;
    float contact_detection_phase_threshold;
    std::vector<int> initial_leg_state = {0,0};
    std::vector<int> next_leg_state = {0,0};
    Eigen::Vector2d initial_leg_phase;
    Eigen::Vector2d initial_state_ratio_in_cycle;

    std::vector<int> contact_state = {0,0};
    
};

#endif