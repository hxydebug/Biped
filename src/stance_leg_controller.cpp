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
    // foot contact state
    std::vector<int> footcontact(2);
    footcontact[0] = _gait_generator->leg_state[0];
    footcontact[1] = _gait_generator->leg_state[1];

    // desired state
    Eigen::Vector3d p_com_des,w_com_des,dp_com_des,dw_com_des;
    p_com_des<<0,0,user_cmd[2];//0.41~0.42
    dp_com_des<<user_cmd[0],user_cmd[1],0;
    w_com_des<<0,-0.05,0;
    dw_com_des<<0,0,user_cmd[3];

    // current state
    Eigen::VectorXd p_com(3);
    Eigen::VectorXd w_com(3);
    Eigen::VectorXd dp_com(3);
    Eigen::VectorXd dw_com(3);
    p_com << licycle->com_position[0],licycle->com_position[1],licycle->com_height;
    w_com << licycle->rpy[0],licycle->rpy[1],licycle->rpy[2];
    Eigen::Matrix3d com_rotm = rpy2romatrix(licycle->rpy[0],licycle->rpy[1],licycle->rpy[2]);
    dp_com << licycle->com_velocity[0],licycle->com_velocity[1],licycle->com_velocity[2];
    dw_com << licycle->omega_world[0],licycle->omega_world[1],licycle->omega_world[2];

    // foot position in the world frame
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
    Eigen::Vector3d invI_b(1.0/0.46,1.0/0.5,1.0/0.146);//1.0/0.406,1.0/0.335,1.0/0.144
    Eigen::Matrix3d invI_bM = invI_b.asDiagonal();
    Eigen::Matrix3d invI_wM = rot_matrix*invI_bM*rot_matrix.transpose();

    std::vector<double> force = Cmpc.ComputeContactForces(p_com,w_com,dp_com,dw_com,p_com_des,w_com_des,dp_com_des,dw_com_des,m,invI_wM,foot_positions_w,footcontact);
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
const int kStateDim = 13; // 6 dof pose + 6 dof velocity + 1 gravity
const int planning_horizon = 10;
const float timestep = 0.025;
const float kGravity = 9.802;
const float kMaxScale = 10;
const float kMinScale = 0.1;
float body_mass = 8.5;
float alpha = 0.0002;
const std::vector<double> qp_weights {5,5,0.1, 0.1,0.1,50, 0.1,0.1,1, 1,1,1, 0}; // w,p,dw,dp,g
std::vector<float> foot_friction_coeffs {0.45,0.45,0.45,0.45};

ConvexMpc::ConvexMpc()
    : qp_weights_(AsBlockDiagonalMat(qp_weights, planning_horizon)),
    qp_weights_single_(AsBlockDiagonalMat(qp_weights, 1)),
    alpha_(alpha* Eigen::MatrixXd::Identity(num_legs* planning_horizon* k3Dim,
        num_legs* planning_horizon* k3Dim)),
    alpha_single_(alpha*
        Eigen::MatrixXd::Identity(num_legs* k3Dim, num_legs* k3Dim)),

    state_(kStateDim),
    desired_states_(kStateDim* planning_horizon),
    a_mat_(kStateDim, kStateDim),
    b_mat_(kStateDim, action_dim_),
    ab_concatenated_(kStateDim + action_dim_, kStateDim + action_dim_),
    a_exp_(kStateDim, kStateDim),
    b_exp_(kStateDim, action_dim_),
    a_qp_(kStateDim* planning_horizon, kStateDim),
    b_qp_(kStateDim* planning_horizon, action_dim_* planning_horizon),
    p_mat_(num_legs* planning_horizon* k3Dim,
        num_legs* planning_horizon* k3Dim),
    q_vec_(num_legs* planning_horizon* k3Dim),
    anb_aux_(kStateDim* planning_horizon, action_dim_),
    contact_states_(planning_horizon, num_legs),
    constraint_(kConstraintDim* num_legs* planning_horizon,
        action_dim_* planning_horizon),
    constraint_lb_(kConstraintDim* num_legs* planning_horizon),
    constraint_ub_(kConstraintDim* num_legs* planning_horizon),
    qp_solution_(k3Dim* num_legs)

{
    state_.setZero();
    desired_states_.setZero();
    a_mat_.setZero();
    b_mat_.setZero();
    ab_concatenated_.setZero();
    a_exp_.setZero();
    b_exp_.setZero();
    a_qp_.setZero();
    b_qp_.setZero();
    b_qp_transpose_.setZero();
    contact_states_.setZero();
    constraint_.setZero();
    constraint_lb_.setZero();
    constraint_ub_.setZero();

}

std::vector<double> ConvexMpc::ComputeContactForces(
    Eigen::Vector3d p_com,
    Eigen::Vector3d w_com,
    Eigen::Vector3d dp_com,
    Eigen::Vector3d dw_com,
    Eigen::Vector3d p_com_des,
    Eigen::Vector3d w_com_des,
    Eigen::Vector3d dp_com_des,
    Eigen::Vector3d dw_com_des,
    float m,
    Eigen::Matrix3d invI_wM,
    Eigen::MatrixXd foot_positions_w,
    std::vector<int> foot_contact_states) {

    // Prepare the current and desired state vectors of length kStateDim *
    // planning_horizon.
    state_ << w_com[0], w_com[1], w_com[2],
              p_com[0], p_com[1], p_com[2], 
              dw_com[0], dw_com[1], dw_com[2], 
              dp_com[0], dp_com[1], dp_com[2], -kGravity;

    for (int i = 0; i < planning_horizon; ++i) {
        desired_states_[i * kStateDim + 0] = w_com_des[0];
        desired_states_[i * kStateDim + 1] = w_com_des[1];
        desired_states_[i * kStateDim + 2] =
            w_com_des[2] +
            timestep * (i + 1) * dw_com_des[2]; // here only work when roll and pitch are set zero

        desired_states_[i * kStateDim + 3] =
            timestep * (i + 1) * dp_com_des[0];
        desired_states_[i * kStateDim + 4] =
            timestep * (i + 1) * dp_com_des[1];
        desired_states_[i * kStateDim + 5] = p_com_des[2];

        // Prefer to stablize roll and pitch.
        desired_states_[i * kStateDim + 6] = dw_com_des[0];
        desired_states_[i * kStateDim + 7] = dw_com_des[1];
        desired_states_[i * kStateDim + 8] = dw_com_des[2];

        desired_states_[i * kStateDim + 9] = dp_com_des[0];
        desired_states_[i * kStateDim + 10] = dp_com_des[1];
        // Prefer to stablize the body height.
        desired_states_[i * kStateDim + 11] = 0;

        desired_states_[i * kStateDim + 12] = -kGravity;
    }

    //calculate A & B matrix
    CalculateAMat(w_com, &a_mat_);
    // std::cout<<"a_mat_"<<std::endl;
    // std::cout<<a_mat_<<std::endl;
    CalculateBMat(m, invI_wM, foot_positions_w, &b_mat_);
    // std::cout<<"b_mat_"<<std::endl;
    // std::cout<<b_mat_<<std::endl;
    CalculateExponentials(a_mat_, b_mat_, timestep, &ab_concatenated_, &a_exp_,
        &b_exp_);
    CalculateQpMats(a_exp_, b_exp_, qp_weights_single_, alpha_single_,
        planning_horizon, &a_qp_, &anb_aux_, &b_qp_, &p_mat_);

    const Eigen::MatrixXd state_diff = a_qp_ * state_ - desired_states_;
    q_vec_ = 2 * b_qp_.transpose() * (qp_weights_ * state_diff);

    //QP
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
    CopyToMatrix(p_mat_, foot_contact_states, num_legs, planning_horizon,
                 k3Dim, k3Dim, false, &hessian_mat_view);

    std::vector<qpOASES::real_t> g_vec(qp_dim, 0);
    // Copy the g_vec
    CopyToVec(q_vec_, foot_contact_states, num_legs, planning_horizon, k3Dim,
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

Eigen::MatrixXd AsBlockDiagonalMat(const std::vector<double>& qp_weights,
    int planning_horizon) {
    const Eigen::Map<const Eigen::VectorXd> qp_weights_vec(qp_weights.data(),
        qp_weights.size());
    // Directly return the rhs will cause a TSAN failure, probably due to the
    // asDiagonal not reall copying the memory. Creates the temporary will ensure
    // copy on return.
    const Eigen::MatrixXd qp_weights_mat =
        qp_weights_vec.replicate(planning_horizon, 1).asDiagonal();
    return qp_weights_mat;
}

void CalculateAMat(const Eigen::Vector3d& rpy, Eigen::MatrixXd* a_mat_ptr) {
    // The transformation of angular velocity to roll pitch yaw rate. Caveat:
    // rpy rate is not a proper vector and does not follow the common vector
    // transformation dicted by the rotation matrix. Here we assume the input
    // rotation is in X->Y->Z order in the extrinsic/fixed frame, or z->y'->x''
    // order in the intrinsic frame.
    Eigen::Matrix3d Rx,Ry,Rz,Rb;
    double r,p,y;
    r = rpy[0];
    p = rpy[1];
    y = rpy[2];
    Rx << 1, 0, 0,
          0, cos(r), -sin(r),
          0, sin(r), cos(r);
    Ry << cos(p), 0, sin(p),
          0, 1, 0,
          -sin(p), 0, cos(p);
    Rz << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;
    Rb << cos(y)*cos(p), -sin(y), 0,
          sin(y)*cos(p), cos(y), 0,
          -sin(p), 0, 1;
    Eigen::Matrix3d R = Rb.inverse();
    Eigen::MatrixXd& a_mat = *a_mat_ptr;
    a_mat.block<3, 3>(0, 6) = R;
    a_mat(3, 9) = 1;
    a_mat(4, 10) = 1;
    a_mat(5, 11) = 1;
    a_mat(11, 12) = 1;

    // const double cos_yaw = cos(rpy[2]);
    // const double sin_yaw = sin(rpy[2]);
    // const double cos_pitch = cos(rpy[1]);
    // const double tan_pitch = tan(rpy[1]);
    // Eigen::Matrix3d angular_velocity_to_rpy_rate;
    // angular_velocity_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
    //     -sin_yaw, cos_yaw, 0, cos_yaw* tan_pitch, sin_yaw* tan_pitch, 1;

    // Eigen::MatrixXd& a_mat = *a_mat_ptr;
    // a_mat.block<3, 3>(0, 6) = angular_velocity_to_rpy_rate;
    // a_mat(3, 9) = 1;
    // a_mat(4, 10) = 1;
    // a_mat(5, 11) = 1;
    // a_mat(11, 12) = 1;
}

void CalculateBMat(double mass, const Eigen::Matrix3d& inv_inertia,
    const Eigen::MatrixXd& foot_positions, Eigen::MatrixXd* b_mat_ptr) {
    // b_mat contains non_zero elements only in row 6:12.
    Eigen::MatrixXd& b_mat = *b_mat_ptr;
    for (int i = 0; i < 2; ++i) {
        b_mat.block<k3Dim, k3Dim>(6, i * k3Dim) =
            inv_inertia * antisym_Matrix(foot_positions.col(i));
        b_mat(9, i * k3Dim) = 1.0/mass;
        b_mat(10, i * k3Dim + 1) = 1.0/mass;
        b_mat(11, i * k3Dim + 2) = 1.0/mass;
    }
}

void CalculateExponentials(const Eigen::MatrixXd& a_mat, const Eigen::MatrixXd& b_mat,
    double timestep, Eigen::MatrixXd* ab_mat_ptr,
    Eigen::MatrixXd* a_exp_ptr, Eigen::MatrixXd* b_exp_ptr) {
    const int state_dim = kStateDim;
    Eigen::MatrixXd& ab_mat = *ab_mat_ptr;
    ab_mat.block<state_dim, state_dim>(0, 0) = a_mat * timestep;
    const int action_dim = b_mat.cols();
    ab_mat.block(0, state_dim, state_dim, action_dim) = b_mat * timestep;

    // This temporary is inevitable.
    Eigen::MatrixXd ab_exp = ab_mat.exp();
    *a_exp_ptr = ab_exp.block<state_dim, state_dim>(0, 0);
    *b_exp_ptr = ab_exp.block(0, state_dim, state_dim, action_dim);
}

void CalculateQpMats(const Eigen::MatrixXd& a_exp, const Eigen::MatrixXd& b_exp,
    const Eigen::MatrixXd& qp_weights_single,
    const Eigen::MatrixXd& alpha_single, int horizon,
    Eigen::MatrixXd* a_qp_ptr, Eigen::MatrixXd* anb_aux_ptr,
    Eigen::MatrixXd* b_qp_ptr, Eigen::MatrixXd* p_mat_ptr) {
    const int state_dim = kStateDim;
    Eigen::MatrixXd& a_qp = *a_qp_ptr;
    a_qp.block(0, 0, state_dim, state_dim) = a_exp;
    for (int i = 1; i < horizon; ++i) {
        a_qp.block<state_dim, state_dim>(i * state_dim, 0) =
            a_exp * a_qp.block<state_dim, state_dim>((i - 1) * state_dim, 0);
    }

    const int action_dim = b_exp.cols();

    Eigen::MatrixXd& anb_aux = *anb_aux_ptr;
    // Compute auxiliary matrix: [B_exp, A_exp * B_exp, ..., A_exp^(h-1) * B_exp]
    anb_aux.block(0, 0, state_dim, action_dim) = b_exp;
    for (int i = 1; i < horizon; ++i) {
        anb_aux.block(i * state_dim, 0, state_dim, action_dim) =
            a_exp * anb_aux.block((i - 1) * state_dim, 0, state_dim, action_dim);
    }

    Eigen::MatrixXd& b_qp = *b_qp_ptr;
    for (int i = 0; i < horizon; ++i) {
        // Diagonal block.
        b_qp.block(i * state_dim, i * action_dim, state_dim, action_dim) = b_exp;
        // Off diagonal Diagonal block = A^(i - j - 1) * B_exp.
        for (int j = 0; j < i; ++j) {
            const int power = i - j;
            b_qp.block(i * state_dim, j * action_dim, state_dim, action_dim) =
                anb_aux.block(power * state_dim, 0, state_dim, action_dim);
        }
    }

    // We construct the P matrix by filling in h x h submatrices, each with size
    // action_dim x action_dim.
    // The r_th (r in [1, h]) diagonal submatrix of P is:
    // 2 * sum_{i=0:h-r}(B'A'^i L A^i B) + alpha, where h is the horizon.
    // The off-diagonal submatrix at row r and column c of P is:
    // 2 * sum_{i=0:h-c}(B'A'^{h-r-i} L A^{h-c-i} B)
    Eigen::MatrixXd& p_mat = *p_mat_ptr;
    // We first compute the submatrices at column h.
    for (int i = horizon - 1; i >= 0; --i) {
        p_mat.block(i * action_dim, (horizon - 1) * action_dim, action_dim,
            action_dim) =
            anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
            .transpose() *
            qp_weights_single * b_exp;
       // Fill the lower-triangle part by transposing the corresponding
       // upper-triangle part.
        if (i != horizon - 1) {
            p_mat.block((horizon - 1) * action_dim, i * action_dim, action_dim,
                action_dim) =
                p_mat
                .block(i * action_dim, (horizon - 1) * action_dim, action_dim,
                    action_dim)
                .transpose();
        }
    }

    // We then fill in the submatrices in the middle by propagating the values
    // from lower right to upper left.
    for (int i = horizon - 2; i >= 0; --i) {
        // Diagonal block.
        p_mat.block(i * action_dim, i * action_dim, action_dim, action_dim) =
            p_mat.block((i + 1) * action_dim, (i + 1) * action_dim, action_dim,
                action_dim) +
            anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
            .transpose() *
            qp_weights_single *
            anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim,
                action_dim);
        // Off diagonal block
        for (int j = i + 1; j < horizon - 1; ++j) {
            p_mat.block(i * action_dim, j * action_dim, action_dim, action_dim) =
                p_mat.block((i + 1) * action_dim, (j + 1) * action_dim, action_dim,
                    action_dim) +
                anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
                .transpose() *
                qp_weights_single *
                anb_aux.block((horizon - j - 1) * state_dim, 0, state_dim,
                    action_dim);
            // Fill the lower-triangle part by transposing the corresponding
            // upper-triangle part.
            p_mat.block(j * action_dim, i * action_dim, action_dim, action_dim) =
                p_mat.block(i * action_dim, j * action_dim, action_dim, action_dim)
                .transpose();
        }
    }

    // Multiply by 2 and add alpha.
    p_mat *= 2.0;
    for (int i = 0; i < horizon; ++i) {
        p_mat.block(i * action_dim, i * action_dim, action_dim, action_dim) +=
            alpha_single;
    }
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