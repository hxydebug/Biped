#include "stance_leg_controller.h"
#include "qpOASES.hpp"
#include "qpOASES/Types.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "Eigen/SparseCore"

using qpOASES::QProblem;

typedef Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic,
                      Eigen::RowMajor>
    RowMajorMatrixXd;

// Auxiliary function for copying data to qpOASES data structure.
void CopyToVec(const Eigen::VectorXd& vec,
               const std::vector<int> foot_contact_states, int num_legs,
               int planning_horizon, int blk_size,
               std::vector<qpOASES::real_t>* out) {
  int buffer_index = 0;
  for (int i = 0; i < num_legs * planning_horizon; ++i) {
    int leg_id = (i % num_legs);
    if (foot_contact_states[leg_id] == 0) {
      // skip the block.
      continue;
    }
    // otherwise copy this block.
    assert(buffer_index < out->size());
    for (int j = 0; j < blk_size; ++j) {
      int index = i * blk_size + j;
      (*out)[buffer_index] = vec[index];
      ++buffer_index;
    }
  }
}

void CopyToMatrix(const Eigen::MatrixXd& input,
                  const std::vector<int> foot_contact_states, int num_legs,
                  int planning_horizon, int row_blk_size, int col_blk_size,
                  bool is_block_diagonal, Eigen::Map<RowMajorMatrixXd>* out) {
  // the block index in the destination matrix.
  int row_blk = 0;
  for (int i = 0; i < planning_horizon * num_legs; ++i) {
    int leg_id = (i % num_legs);
    if (foot_contact_states[leg_id] == 0) {
      // skip the row block.
      continue;
    }
    if (is_block_diagonal) {
      // just copy the block
      int col_blk = row_blk;
      out->block(row_blk * row_blk_size, col_blk * col_blk_size, row_blk_size,
                 col_blk_size) = input.block(i * row_blk_size, i * col_blk_size,
                                             row_blk_size, col_blk_size);
    } else {
      int col_blk = 0;
      // Non-diagonal, need to copy all elements.
      for (int j = 0; j < planning_horizon * num_legs; ++j) {
        int leg_id = (j % num_legs);
        if (foot_contact_states[leg_id] == 0) {
          // skip the col block.
          continue;
        }
        out->block(row_blk * row_blk_size, col_blk * col_blk_size, row_blk_size,
                   col_blk_size) =
            input.block(i * row_blk_size, j * col_blk_size, row_blk_size,
                        col_blk_size);
        ++col_blk;
      }
    }
    ++row_blk;
  }
}

/***********************            class            *********************/
stance_leg_controller::stance_leg_controller(Leg_state *bike,gait_generator *gait_generator,float desired_speed){
  licycle = bike;
  _gait_generator = gait_generator;

  desired_xspeed = desired_speed;
  desired_roll = 0;
  num_leg = 2;
  _desired_height.resize(3);
  _desired_height << 0,0,0.32;
  GRF.resize(6);
  GRF << 0,0,0,0,0,0;
}

Eigen::VectorXd stance_leg_controller::get_action(Eigen::VectorXd user_cmd){
    // std::vector<int> footcontact = licycle->GetFootContact();
    std::vector<int> footcontact(2);
    footcontact[0] = _gait_generator->leg_state[0];
    footcontact[1] = _gait_generator->leg_state[1];

    Eigen::Vector3d p_com_des,w_com_des,dp_com_des,dw_com_des;
    p_com_des<<0,0,user_cmd[2];//0.41~0.42
    dp_com_des<<user_cmd[0],user_cmd[1],0;
    w_com_des<<0,0,0;
    dw_com_des<<0,0,user_cmd[3];

    Eigen::VectorXd p_com(3);
    Eigen::VectorXd dp_com(3);
    Eigen::VectorXd dw_com(3);
    p_com << 0,0,licycle->com_height;
    Eigen::Matrix3d com_rotm = rpy2romatrix(licycle->rpy[0],licycle->rpy[1],licycle->rpy[2]);
    dp_com << licycle->com_velocity[0],licycle->com_velocity[1],licycle->com_velocity[2];
    dw_com << licycle->omega_world[0],licycle->omega_world[1],licycle->omega_world[2];

    Eigen::Matrix3d com_rotm_des = rpy2romatrix(w_com_des[0],w_com_des[1],w_com_des[2]);
    Eigen::Vector3d kp_p(0,0,20);
    Eigen::Vector3d kd_p(10,10,10);
    Eigen::Vector3d kp_w(30,50,10);
    Eigen::Vector3d kd_w(10,20,10);

    Eigen::Matrix3d M_kp_p = kp_p.asDiagonal();
    Eigen::Matrix3d M_kd_p = kd_p.asDiagonal();
    Eigen::Matrix3d M_kp_w = kp_w.asDiagonal();
    Eigen::Matrix3d M_kd_w = kd_w.asDiagonal();

    Eigen::Matrix3d R_error = com_rotm_des * com_rotm.transpose();
    Eigen::AngleAxisd axis_angle = romatrix2AngleAxis(R_error);
    Eigen::Vector3d w_error = axis_angle.angle()*axis_angle.axis();
    // std::cout<<"angle_axis:"<<w_error<<std::endl;
    Eigen::Vector3d f_pd = M_kp_p * (p_com_des-p_com) + M_kd_p * (dp_com_des-dp_com);
    Eigen::Vector3d tau_pd = M_kp_w * w_error + M_kd_w * (dw_com_des - dw_com);

    Eigen::MatrixXd foot_positions(3,2);
    Eigen::MatrixXd foot_positions_w(3,2);
    Angle l_angle;
    Angle r_angle;
    Position l_position;
    Position r_position;
    for(int i(0);i<3;i++){
        l_angle.q[i] = licycle->cbdata[i].p;
        r_angle.q[i] = licycle->cbdata[i+3].p;
    }
    Kinematics(&l_angle,&l_position,0);
    Kinematics(&r_angle,&r_position,1);
    //foot positions in body coordinate
    foot_positions << l_position.x, r_position.x,
                      l_position.y, r_position.y,
                      l_position.z, r_position.z;

    Eigen::Matrix3d rot_matrix = com_rotm;
    foot_positions_w = rot_matrix*foot_positions;

    float m = 8.6;
    //I_b to I_w
    Eigen::Vector3d I_b(0.4,0.4,0.046);
    Eigen::Matrix3d I_bM = I_b.asDiagonal();
    Eigen::Matrix3d I_wM = rot_matrix*I_bM*rot_matrix.transpose();

    std::vector<double> force = Cmpc.ComputeContactForces(f_pd,tau_pd,m,I_wM,foot_positions_w,footcontact);
    Eigen::Map<Eigen::VectorXd> force_E(force.data(),force.size());
    GRF = force_E;
    // std::cout<<force_E<<std::endl;
    Eigen::VectorXd l_force,r_force;
    l_force = force_E.head(3);
    r_force = force_E.tail(3);
    Eigen::VectorXd ltau(3),rtau(3);
    if (footcontact[0] == 0){
        ltau << 0,0,0;
    }
    else{
        ltau = calcu_Jaco(l_angle,0).transpose() * rot_matrix.transpose() * l_force;
    }
    if (footcontact[1] == 0){
        rtau << 0,0,0;
    }
    else{
        rtau = calcu_Jaco(r_angle,1).transpose() * rot_matrix.transpose() * r_force;
    }
    Eigen::VectorXd tau(6);
    tau << ltau,rtau;
    return tau;
}


/***********************            useful function           *********************/

const int kConstraintDim = 5;
const int k3Dim = 3;
const int num_legs = 2;
const int action_dim_ = num_legs * k3Dim;
const int planning_horizon = 1;
const float kGravity = 9.802;
const float kMaxScale = 10;
const float kMinScale = 0.1;
float body_mass = 8.6;
std::vector<float> foot_friction_coeffs {0.45,0.45,0.45,0.45};

ConvexMpc::ConvexMpc()
    :
    contact_states_(planning_horizon, num_legs),
    constraint_(kConstraintDim* num_legs* planning_horizon,
        action_dim_* planning_horizon),
    constraint_lb_(kConstraintDim* num_legs* planning_horizon),
    constraint_ub_(kConstraintDim* num_legs* planning_horizon),
    qp_solution_(k3Dim* num_legs)

{
    contact_states_.setZero();
    constraint_.setZero();
    constraint_lb_.setZero();
    constraint_ub_.setZero();

}

std::vector<double> ConvexMpc::ComputeContactForces(
    Eigen::Vector3d f_pd,
    Eigen::Vector3d tau_pd,
    float m,
    Eigen::Matrix3d I_wM,
    Eigen::MatrixXd foot_positions_w,
    std::vector<int> foot_contact_states) {
    
    //antisymmetric matrix
    Eigen::VectorXd lf_position_w = foot_positions_w.col(0);
    Eigen::VectorXd rf_position_w = foot_positions_w.col(1);
    Eigen::Matrix3d lf_X = antisym_Matrix(lf_position_w);
    Eigen::Matrix3d rf_X = antisym_Matrix(rf_position_w);
    
    //calculate A & B matrix
    Eigen::MatrixXd A_mat(6,6),B_mat(6,1);
    A_mat.setZero();
    B_mat.setZero();
    A_mat.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3, 3);
    A_mat.block<3,3>(0,3)=Eigen::MatrixXd::Identity(3, 3);
    A_mat.block<3,3>(3,0)=lf_X;
    A_mat.block<3,3>(3,3)=rf_X;
    float g = 9.802;
    Eigen::Vector3d g_acc(0,0,g);
    B_mat << m*(f_pd+g_acc),I_wM*tau_pd;

    //QP
    Eigen::VectorXd L(6);
    L << 1,1,50,600,1700,1;
    Eigen::MatrixXd L_M = L.asDiagonal();
    Eigen::MatrixXd W = 0.7*Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd M = 1*Eigen::MatrixXd::Identity(6, 6);

    Eigen::MatrixXd Hd = 2*(A_mat.transpose()*L_M*A_mat + W);
    Eigen::VectorXd fd = -2*(A_mat.transpose()*L_M*B_mat);

    const Eigen::VectorXd one_vec = Eigen::VectorXd::Constant(planning_horizon, 1.0);
    const Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(planning_horizon);
    for (int j = 0; j < foot_contact_states.size(); ++j) {
        if (foot_contact_states[j]) {
            contact_states_.col(j) = one_vec;
        }
        else {
            contact_states_.col(j) = zero_vec;
        }
    }

    CalculateConstraintBounds(contact_states_, body_mass * kGravity * 1.5,
        body_mass * kGravity * 0,
        foot_friction_coeffs[0], planning_horizon,
        &constraint_lb_, &constraint_ub_);

    UpdateConstraintsMatrix(foot_friction_coeffs,planning_horizon,num_legs,&constraint_);

    // To use qpOASES, we need to eleminate the zero rows/cols from the
    // matrices when copy to qpOASES buffer
    int num_legs_in_contact = 0;
    for (int i = 0; i < foot_contact_states.size(); ++i) {
      if (foot_contact_states[i]) {
        num_legs_in_contact += 1;
      }
    }
    const int qp_dim = num_legs_in_contact * k3Dim * planning_horizon;
    const int constraint_dim = num_legs_in_contact * 5 * planning_horizon;
    std::vector<qpOASES::real_t> hessian(qp_dim * qp_dim, 0);
    Eigen::Map<RowMajorMatrixXd> hessian_mat_view(hessian.data(), qp_dim, qp_dim);
    // Copy to the hessian
    CopyToMatrix(Hd, foot_contact_states, num_legs, planning_horizon,
                 k3Dim, k3Dim, false, &hessian_mat_view);

    std::vector<qpOASES::real_t> g_vec(qp_dim, 0);
    // Copy the g_vec
    CopyToVec(fd, foot_contact_states, num_legs, planning_horizon, k3Dim,
              &g_vec);

    std::vector<qpOASES::real_t> a_mat(qp_dim * constraint_dim, 0);
    Eigen::Map<RowMajorMatrixXd> a_mat_view(a_mat.data(), constraint_dim, qp_dim);
    CopyToMatrix(constraint_, foot_contact_states, num_legs, planning_horizon,
                 5, k3Dim, true, &a_mat_view);

    std::vector<qpOASES::real_t> a_lb(constraint_dim, 0);
    CopyToVec(constraint_lb_, foot_contact_states, num_legs, planning_horizon,
              5, &a_lb);

    std::vector<qpOASES::real_t> a_ub(constraint_dim, 0);
    CopyToVec(constraint_ub_, foot_contact_states, num_legs, planning_horizon,
              5, &a_ub);

    auto qp_problem = QProblem(qp_dim, constraint_dim, qpOASES::HST_UNKNOWN,
                               qpOASES::BT_TRUE);

    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp_problem.setOptions(options);

    int max_solver_iter = 100;

    qp_problem.init(hessian.data(), g_vec.data(), a_mat.data(), nullptr,
                    nullptr, a_lb.data(), a_ub.data(), max_solver_iter,
                    nullptr);

    std::vector<qpOASES::real_t> qp_sol(qp_dim, 0);
    qp_problem.getPrimalSolution(qp_sol.data());
    for (auto& force : qp_sol) {
      force = -force;
    }

    int buffer_index = 0;
    for (int i = 0; i < num_legs; ++i) {
      int leg_id = i % num_legs;
      if (foot_contact_states[leg_id] == 0) {
        qp_solution_[i * k3Dim] = 0;
        qp_solution_[i * k3Dim + 1] = 0;
        qp_solution_[i * k3Dim + 2] = 0;
      } else {
        qp_solution_[i * k3Dim] = qp_sol[buffer_index * k3Dim];
        qp_solution_[i * k3Dim + 1] = qp_sol[buffer_index * k3Dim + 1];
        qp_solution_[i * k3Dim + 2] = qp_sol[buffer_index * k3Dim + 2];
        ++buffer_index;
      }
    }

    return qp_solution_;
}

Eigen::MatrixXd  antisym_Matrix(Eigen::Vector3d w_axis){
    Eigen::Matrix3d w_hat;
    w_hat<< 0,-w_axis(2),w_axis(1),
            w_axis(2),0,-w_axis(0),
            -w_axis(1),w_axis(0),0;
    return w_hat;
}

void UpdateConstraintsMatrix(std::vector<float>& friction_coeff,
    int horizon, int num_legs,
    Eigen::MatrixXd* constraint_ptr) {
    const int constraint_dim = kConstraintDim;
    Eigen::MatrixXd& constraint = *constraint_ptr;
    for (int i = 0; i < horizon * num_legs; ++i) {
        constraint.block<constraint_dim, k3Dim>(i * constraint_dim, i * k3Dim)
            << -1, 0, friction_coeff[0], 
            1, 0, friction_coeff[1],
            0, -1, friction_coeff[2],
            0, 1, friction_coeff[3], 
            0, 0, 1;
    }
}

void CalculateConstraintBounds(const Eigen::MatrixXd& contact_state, float fz_max,
    float fz_min, float friction_coeff,
    int horizon, Eigen::VectorXd* constraint_lb_ptr,
    Eigen::VectorXd* constraint_ub_ptr) {
    const int constraint_dim = kConstraintDim;

    const int num_legs = contact_state.cols();

    Eigen::VectorXd& constraint_lb = *constraint_lb_ptr;
    Eigen::VectorXd& constraint_ub = *constraint_ub_ptr;
    for (int i = 0; i < horizon; ++i) {
        for (int j = 0; j < num_legs; ++j) {
            const int row = (i * num_legs + j) * constraint_dim;
            constraint_lb(row) = 0;
            constraint_lb(row + 1) = 0;
            constraint_lb(row + 2) = 0;
            constraint_lb(row + 3) = 0;
            constraint_lb(row + 4) = fz_min * contact_state(i, j);

            const double friction_ub =
                (friction_coeff + 1) * fz_max * contact_state(i, j);
            constraint_ub(row) = friction_ub;
            constraint_ub(row + 1) = friction_ub;
            constraint_ub(row + 2) = friction_ub;
            constraint_ub(row + 3) = friction_ub;
            constraint_ub(row + 4) = fz_max * contact_state(i, j);
        }
    }
}