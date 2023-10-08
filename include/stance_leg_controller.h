#ifndef __STANCE_LEG_CONTROL_H
#define __STANCE_LEG_CONTROL_H

#include "gait_generator.h"
#include "control.h"
#include "bikebot_control.h"

class ConvexMpc{

public:

    ConvexMpc();
    std::vector<double> ComputeContactForces(
        float com_velocity,
        float com_roll,
        float com_droll,
        std::vector<int> foot_contact_states,
        Eigen::MatrixXd foot_positions_body_frame,
        float desired_com_velocity,
        float desired_com_roll,
        float desired_com_droll);

private:

    // 4 * horizon diagonal matrix.
    const Eigen::MatrixXd qp_weights_;
    // 4 x 4 diagonal matrix.
    const Eigen::MatrixXd qp_weights_single_;
    // num_legs * 3 * horizon diagonal matrix.
    const Eigen::MatrixXd alpha_;
    const Eigen::MatrixXd alpha_single_;
    // The following matrices will be updated for every call. However, their sizes
    // can be determined at class initialization time.
    Eigen::VectorXd state_;               // 4
    Eigen::VectorXd desired_states_;      // 4 * horizon
    Eigen::MatrixXd contact_states_;      // horizon x num_legs
    Eigen::MatrixXd foot_positions_base_; // 3 x num_legs
    Eigen::MatrixXd foot_positions_world_;// 3 x num_legs

    Eigen::MatrixXd a_mat_;               // 4 x 4
    Eigen::MatrixXd b_mat_;               // 4 x (num_legs * 3)
    Eigen::MatrixXd ab_concatenated_;     // 4 + num_legs * 3 x 4 + num_legs * 3
    Eigen::MatrixXd a_exp_;               // same dimension as a_mat_
    Eigen::MatrixXd b_exp_;               // same dimension as b_mat_

    // Contains all the power mats of a_exp_. Consider Eigen::SparseMatrix.
    Eigen::MatrixXd a_qp_;                // 4 * horizon x 4
    Eigen::MatrixXd b_qp_;                // 4 * horizon x num_legs * 3 * horizon    sparse
    Eigen::MatrixXd b_qp_transpose_;
    Eigen::MatrixXd p_mat_;               // num_legs * 3 * horizon x num_legs * 3 * horizon
    Eigen::VectorXd q_vec_;               // num_legs * 3 * horizon vector

    // Auxiliary containing A^n*B, with n in [0, num_legs * 3)
    Eigen::MatrixXd anb_aux_;             // 4 * horizon x (num_legs * 3)

    // Contains the constraint matrix and bounds.
    Eigen::MatrixXd constraint_;          // 5 * num_legs * horizon x 3 * num_legs * horizon
    Eigen::VectorXd constraint_lb_;       // 5 * num_legs * horizon
    Eigen::VectorXd constraint_ub_;       // 5 * num_legs * horizon

    std::vector<double> qp_solution_;     // 3 * num_legs
};

class stance_leg_controller{
public:
    stance_leg_controller(Leg_state *bike,gait_generator *gait_generator,float desired_speed);
    Eigen::VectorXd get_action(void);
    float desired_xspeed;
    float desired_roll;
    
private:
    gait_generator *_gait_generator;
    Leg_state *licycle;
    ConvexMpc Cmpc;
    Eigen::VectorXd _desired_height;
    int num_leg;

};

// useful function
Eigen::MatrixXd AsBlockDiagonalMat(const std::vector<double>& qp_weights,
    int planning_horizon);
void CalculateAMat(Eigen::MatrixXd* a_mat_ptr);
void CalculateBMat(const Eigen::MatrixXd& foot_positions,
    Eigen::MatrixXd* b_mat_ptr);
void CalculateExponentials(const Eigen::MatrixXd& a_mat,
    const Eigen::MatrixXd& b_mat, float timestep,
    Eigen::MatrixXd* ab_mat_ptr,
    Eigen::MatrixXd* a_exp_ptr,
    Eigen::MatrixXd* b_exp_ptr);
void CalculateQpMats(const Eigen::MatrixXd& a_exp, const Eigen::MatrixXd& b_exp,
    const Eigen::MatrixXd& qp_weights_single,
    const Eigen::MatrixXd& alpha_single, int horizon,
    Eigen::MatrixXd* a_qp_ptr, Eigen::MatrixXd* anb_aux_ptr,
    Eigen::MatrixXd* b_qp_ptr, Eigen::MatrixXd* p_mat_ptr);
void UpdateConstraintsMatrix(std::vector<float>& friction_coeff,
    int horizon, int num_legs,
    Eigen::MatrixXd* constraint_ptr);
void CalculateConstraintBounds(const Eigen::MatrixXd& contact_state, float fz_max,
    float fz_min, float friction_coeff, int horizon,
    Eigen::VectorXd* constraint_lb_ptr,
    Eigen::VectorXd* constraint_ub_ptr);
#endif