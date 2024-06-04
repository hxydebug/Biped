/*! @file PosVelEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef __POSVELESTIMATOR_H
#define __POSVELESTIMATOR_H

#include "gait_generator.h"
#include "control.h"
#include "bikebot_control.h"

class PosVelEstimator{
public:
    PosVelEstimator(Leg_state *robot, gait_generator *gait_generator, float timestep);
    void run();
private:
    Eigen::Matrix<float, 12, 1> _xhat;
    Eigen::Matrix<float, 6, 1> _ps;
    Eigen::Matrix<float, 6, 1> _vs;
    Eigen::Matrix<float, 12, 12> _A;
    Eigen::Matrix<float, 12, 12> _Q0;
    Eigen::Matrix<float, 12, 12> _P;
    Eigen::Matrix<float, 14, 14> _R0;
    Eigen::Matrix<float, 12, 3> _B;
    Eigen::Matrix<float, 14, 12> _C;

    gait_generator *_gait_generator;
    Leg_state *_robot;

};

#endif  // __POSVELESTIMATOR_H
