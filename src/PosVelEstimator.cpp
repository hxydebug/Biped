/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "PosVelEstimator.h"

// vicon global variables
Eigen::Vector3d COMposition_vicon;
Eigen::Vector3d COMvelocity_vicon;
Eigen::Vector3d position_vicon;
Eigen::Vector3d last_position_vicon;

PosVelEstimator::PosVelEstimator(Leg_state *robot, gait_generator *gait_generator, float timestep){
    _robot = robot;
    _gait_generator = gait_generator;
    float dt = timestep;

    _xhat.setZero();
    _ps.setZero();
    _vs.setZero();
    _A.setZero();
    _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(6, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
    _B.setZero();
    _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
    _C.setZero();
    _C.block(0, 0, 3, 6) = C1;
    _C.block(3, 0, 3, 6) = C1;
    _C.block(0, 6, 6, 6) = double(-1) * Eigen::Matrix<double, 6, 6>::Identity();
    _C.block(6, 0, 3, 6) = C2;
    _C.block(9, 0, 3, 6) = C2;
    _C(12, 8) = double(1);
    _C(13, 11) = double(1);
    _P.setIdentity();
    _P = double(100) * _P;
    _Q0.setIdentity();
    _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(3, 3, 3, 3) =
        (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(6, 6, 6, 6) = dt * Eigen::Matrix<double, 6, 6>::Identity();
    _R0.setIdentity();

    position_offset[0] = -(0.02845+0.007-detx);
    position_offset[1] = 0.0077;
    position_offset[2] = -(0.08866+0.016-detz);
}

void PosVelEstimator::run(){
    double process_noise_pimu = 0.005;
    double process_noise_vimu = 0.015;
    double process_noise_pfoot = 0.002;
    double sensor_noise_pimu_rel_foot = 0.001;
    double sensor_noise_vimu_rel_foot = 0.02;//this can be smaller
    double sensor_noise_zfoot = 0.001;

    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
    Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
    Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
    Q.block(6, 6, 6, 6) = _Q0.block(6, 6, 6, 6) * process_noise_pfoot;

    Eigen::Matrix<double, 14, 14> R = Eigen::Matrix<double, 14, 14>::Identity();
    R.block(0, 0, 6, 6) = _R0.block(0, 0, 6, 6) * sensor_noise_pimu_rel_foot;
    R.block(6, 6, 6, 6) =
        _R0.block(6, 6, 6, 6) * sensor_noise_vimu_rel_foot;
    R.block(12, 12, 2, 2) = _R0.block(12, 12, 2, 2) * sensor_noise_zfoot;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;
    // rotation matrix from body to world
    Eigen::Matrix3d Rbod = rpy2romatrix(_robot->rpy[0],_robot->rpy[1],_robot->rpy[2]);
    // angular velocity in imu frame
    Eigen::Vector3d omegaBody, omegaWorld;
    omegaBody[0] = _robot->omega[0];
    omegaBody[1] = _robot->omega[1];
    omegaBody[2] = _robot->omega[2];
    omegaWorld = Rbod*omegaBody;

    // Rbod * acc + g 
    Eigen::VectorXd Body_acc;
    Body_acc.resize(3);
    Body_acc << _robot->acc[0],_robot->acc[1],_robot->acc[2];
    Eigen::Vector3d g;
    g << 0, 0, -9.81;
    Eigen::VectorXd World_acc = Rbod*Body_acc + g;
    // std::cout << "A WORLD\n" << World_acc << "\n";

    Eigen::Matrix<double, 2, 1> pzs = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 2, 1> trusts = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 3, 1> p0, v0;
    p0 << _xhat[0], _xhat[1], _xhat[2];
    v0 << _xhat[3], _xhat[4], _xhat[5];

    // get leg joint position and velocity
    Eigen::VectorXd nowang(6);
    Eigen::VectorXd nowangV(6);
    for(int i(0);i<6;i++){
        nowang[i] = _robot->cbdata[i].p;
        nowangV[i] = _robot->cbdata[i].v;
    }
    Eigen::Vector3d legpos[2];
    legpos[0] = nowang.head(3);
    legpos[1] = nowang.tail(3);
    Eigen::Vector3d legvel[2];
    legvel[0] = nowangV.head(3);
    legvel[1] = nowangV.tail(3);

    // two legs
    for (int i = 0; i < 2; i++) {
        int i1 = 3 * i;  

        // get foot position in world frame
        Position posxyz = getFootPositionInBaswFrame(nowang,i);
        Eigen::Vector3d p_rel;
        p_rel[0] = posxyz.x;
        p_rel[1] = posxyz.y;
        p_rel[2] = posxyz.z;
        Eigen::VectorXd p_f = Rbod * p_rel;

        // get foot velocity in world frame
        Eigen::VectorXd dp_rel(3);
        Eigen::Matrix3d Jac = calcu_Jaco(legpos[i],i);
        dp_rel = Jac*legvel[i];
        // cross must define specifically
        Eigen::VectorXd cross_result = omegaBody.cross(p_rel);
        Eigen::VectorXd dp_f = Rbod * (cross_result + dp_rel);

        qindex = 6 + i1;
        rindex1 = i1;
        rindex2 = 6 + i1;
        rindex3 = 12 + i;

        double trust = double(1);
        double phase = 0;
        if (_gait_generator->leg_state[i] == 1)// only stance foot, phase > 0
        {
            phase = fmin(_gait_generator->normalized_phase[i], double(1));
        }
        //double trust_window = double(0.25);
        double trust_window = double(0.2);

        if (phase < trust_window) {
        trust = phase / trust_window;
        } else if (phase > (double(1) - trust_window)) {
        trust = (double(1) - phase) / trust_window;
        }
        //double high_suspect_number(1000);
        double high_suspect_number(100);

        //printf("Trust %d: %.3f\n", i, trust);
        // it is not confident to predict swing foot position
        Q.block(qindex, qindex, 3, 3) =
            (double(1) + (double(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
        // it is not confident to measure swing foot velocity and z position
        R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
        R.block(rindex2, rindex2, 3, 3) =
            (double(1) + (double(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
        R(rindex3, rindex3) =
            (double(1) + (double(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

        trusts(i) = trust;

        // add bias to transfer imu frame to com frame
        _ps.segment(i1, 3) = -p_f - Rbod * position_offset;
        _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f) - Rbod * omegaBody.cross(position_offset);
        pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
    }

    Eigen::Matrix<double, 14, 1> y;
    y << _ps, _vs, pzs;
    _xhat = _A * _xhat + _B * World_acc;
    Eigen::Matrix<double, 12, 12> At = _A.transpose();
    Eigen::Matrix<double, 12, 12> Pm = _A * _P * At + Q;
    Eigen::Matrix<double, 12, 14> Ct = _C.transpose();
    Eigen::Matrix<double, 14, 1> yModel = _C * _xhat;
    Eigen::Matrix<double, 14, 1> ey = y - yModel;
    Eigen::Matrix<double, 14, 14> S = _C * Pm * Ct + R;

    // todo compute LU only once
    Eigen::Matrix<double, 14, 1> S_ey = S.lu().solve(ey);
    _xhat += Pm * Ct * S_ey;

    Eigen::Matrix<double, 14, 12> S_C = S.lu().solve(_C);
    _P = (Eigen::Matrix<double, 12, 12>::Identity() - Pm * Ct * S_C) * Pm;

    Eigen::Matrix<double, 12, 12> Pt = _P.transpose();
    _P = (_P + Pt) / double(2);

    if (_P.block(0, 0, 2, 2).determinant() > double(0.000001)) {
        _P.block(0, 2, 2, 10).setZero();
        _P.block(2, 0, 10, 2).setZero();
        _P.block(0, 0, 2, 2) /= double(10);
    }

    // transform imu state to com state in world frame
    Eigen::VectorXd com_pos = _xhat.block(0, 0, 3, 1) + Rbod * position_offset;
    Eigen::VectorXd com_vel = _xhat.block(3, 0, 3, 1) + Rbod * omegaBody.cross(position_offset);
    _robot->com_height = com_pos[2];
    _robot->com_velocity[0] = com_vel[0];
    _robot->com_velocity[1] = com_vel[1];
    _robot->com_velocity[2] = com_vel[2];
    _robot->com_position[0] = com_pos[0];
    _robot->com_position[1] = com_pos[1];
    _robot->com_position[2] = com_pos[2];
    _robot->omega_world[0] = omegaWorld[0];
    _robot->omega_world[1] = omegaWorld[1];
    _robot->omega_world[2] = omegaWorld[2];
    // foot position in world frame
    _robot->foot_p[0] = _xhat.block(6, 0, 3, 1);
    _robot->foot_p[1] = _xhat.block(9, 0, 3, 1);

    // vicon
    position_vicon[0] = _robot->vicon_pos[0];
    position_vicon[1] = _robot->vicon_pos[1];
    position_vicon[2] = _robot->vicon_pos[2];
    double offset_z = 0.23;//initial guess
    Eigen::Vector3d position_vicon_body = Rbod.transpose()*position_vicon;
    position_vicon_body[2] -= offset_z;
    COMposition_vicon = Rbod*position_vicon_body;
    Eigen::Vector3d last_position_vicon_body = Rbod.transpose()*last_position_vicon;
    last_position_vicon_body[2] -= offset_z;
    COMvelocity_vicon = Rbod*(position_vicon_body - last_position_vicon_body)*480.0;

    _robot->vicon_COMpos[0] = COMposition_vicon[0];
    _robot->vicon_COMpos[1] = COMposition_vicon[1];
    _robot->vicon_COMpos[2] = COMposition_vicon[2];

    _robot->vicon_COMvel[0] = COMvelocity_vicon[0];
    _robot->vicon_COMvel[1] = COMvelocity_vicon[1];
    _robot->vicon_COMvel[2] = COMvelocity_vicon[2];

    last_position_vicon = position_vicon;
}
