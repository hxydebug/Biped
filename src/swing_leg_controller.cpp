#include "swing_leg_controller.h"

float _KP = 0.03;
float foot_clearance = 0.02;
float desired_height = 0.345;
float t_swing = t_stance;
Eigen::Vector3d dP;

swing_leg_controller::swing_leg_controller(Leg_state *bike,gait_generator *gait_generator,float desired_speed){
  licycle = bike;
  _gait_generator = gait_generator;

  last_leg_state = _gait_generator->desired_leg_state;

  Eigen::VectorXd nowang(6);
  for(int i(0);i<6;i++){
        nowang[i] = licycle->cbdata[i].p;
  }
  phase_switch_foot_local_position[0] = getFootPositionInBaswFrame(nowang,0);
  phase_switch_foot_local_position[1] = getFootPositionInBaswFrame(nowang,1);
  phase_switch_foot_local_position1[0] = getFootPositionInBaswFrame(nowang,0);
  phase_switch_foot_local_position1[1] = getFootPositionInBaswFrame(nowang,1);
  pos_com_last << licycle->com_position[0],licycle->com_position[1],0.33;
  rotMat_last << 1,0,0,
                0,1,0,
                0,0,1;
  foot_position_body[0] << phase_switch_foot_local_position[0].x, phase_switch_foot_local_position[0].y, phase_switch_foot_local_position[0].z;
  foot_position_begin[0] = rotMat_last * foot_position_body[0] + pos_com_last;
  foot_position_body[1] << phase_switch_foot_local_position[1].x, phase_switch_foot_local_position[1].y, phase_switch_foot_local_position[1].z;
  foot_position_begin[1] = rotMat_last * foot_position_body[1] + pos_com_last;
  desired_xspeed = desired_speed;
  angles.resize(6);
  _desired_height.resize(3);
  action.resize(6);
  hip_positions[0].resize(3);
  hip_positions[1].resize(3);
  _desired_height << 0,0,desired_height-foot_clearance;
  angles.setConstant(0);
  action.setConstant(0);
  hip_positions[0] << -0.007,wid/2+Len0,0;
  hip_positions[1] << -0.007,-wid/2-Len0,0;
  timer = 0;
  swing_leg_controller::set_PDGain();

}

void swing_leg_controller::update(float current_time){
  timer = current_time;
  Eigen::Vector3d com_p;
  if (timer < 0.1){
		com_p << licycle->com_position[0],licycle->com_position[1],0.33;
	}
  else {
    com_p << licycle->com_position[0],licycle->com_position[1],licycle->com_height;
  }
  std::vector<int> new_leg_state = _gait_generator->desired_leg_state;
  // can optimize better!
  Eigen::VectorXd nowang(6);
  for(int i(0);i<6;i++){
        nowang[i] = licycle->cbdata[i].p;
  }
  // Detects phase switch for each leg so we can remember the feet position at
  // the beginning of the swing phase.
  for(int i(0);i<2;i++){
    if(new_leg_state[i]==swing_leg && new_leg_state[i] != last_leg_state[i]){


      phase_switch_foot_local_position[i] = getFootPositionInBaswFrame(nowang,i);
      pos_com_last = com_p;
      rotMat_last = rpy2romatrix(licycle->rpy[0],licycle->rpy[1],licycle->rpy[2]);
      foot_position_body[i] << phase_switch_foot_local_position[i].x, phase_switch_foot_local_position[i].y, phase_switch_foot_local_position[i].z;
      foot_position_begin[i] = rotMat_last * foot_position_body[i] + pos_com_last;
    }
    if (new_leg_state[i]==stance_leg && new_leg_state[i] != last_leg_state[i]) {
      phase_switch_foot_local_position1[i] = getFootPositionInBaswFrame(nowang,i);
    }
  }

  last_leg_state = new_leg_state;

}

Eigen::VectorXd swing_leg_controller::get_action(Eigen::VectorXd user_cmd){

  Eigen::VectorXd com_velocity(3);
  com_velocity << licycle->com_velocity[0],licycle->com_velocity[1],licycle->com_velocity[2];
  Eigen::Matrix3d com_rotm = rpy2romatrix(licycle->rpy[0],licycle->rpy[1],licycle->rpy[2]);
  Eigen::Vector3d omegaBody;
  omegaBody[0] = licycle->omega[0];
  omegaBody[1] = licycle->omega[1];
  omegaBody[2] = licycle->omega[2];
  // com_velocity << 0,0,0;
  // Eigen::Matrix3d com_rotm = rpy2romatrix(0,0,0);

  // com in world frame
  // height (not used)


  /******************* test begin ******************/
  desired_height = user_cmd[2];
  _desired_height << 0,0,desired_height-foot_clearance;
  Eigen::Vector3d hip_horizontal_velocity;
  hip_horizontal_velocity << 0,0,0;
  Eigen::VectorXd target_hip_horizontal_velocity(3);
  target_hip_horizontal_velocity << user_cmd[0],user_cmd[1],0;
  /******************* test end ******************/


  Eigen::Vector3d p_com;
  if (timer < 0.1){
		p_com << licycle->com_position[0],licycle->com_position[1],0.33;
	}
  else {
    p_com << licycle->com_position[0],licycle->com_position[1],licycle->com_height;
  }

  // joint angle
  Eigen::VectorXd nowang(6);
  Eigen::VectorXd nowangV(6);
  for(int i(0);i<6;i++){
        nowang[i] = licycle->cbdata[i].p;
        nowangV[i] = licycle->cbdata[i].v;
  }
  Eigen::Vector3d legpos[2];
  legpos[0] = nowang.head(3);
  legpos[1] = nowang.tail(3);
  Eigen::Vector3d legvel[2];
  legvel[0] = nowangV.head(3);
  legvel[1] = nowangV.tail(3);
  // torque
  Eigen::VectorXd leg_motor_torque(6);
  leg_motor_torque.setConstant(0);
  Eigen::Vector3d motor_torque[2];
  // yaw
  double dyaw_com = licycle->omega_world[2];
  double yaw_com = licycle->rpy[2];

  // velocity cmd
  Eigen::VectorXd linear_velocity = com_velocity;
  linear_velocity[2] = 0;
  Eigen::VectorXd desired_linear_velocity(3);
  // command is in the body frame
  Eigen::Vector3d v_bd;
  v_bd << user_cmd[0],user_cmd[1],0;
  // make command in z is zero
  desired_linear_velocity = com_rotm * v_bd;
  desired_linear_velocity[2] = 0;
  // considering rotation for foot placement
  double desired_dw_com = user_cmd[3];
  double feetR[2], feetInitAngle[2];
  feetR[0] = wid/2 + Len0-0.03;
  feetR[1] = wid/2 + Len0-0.03;
  feetInitAngle[0] = 95.0*PII/180.0;
  feetInitAngle[1] = -95.0*PII/180.0;

  for(int i(0);i<2;i++){
    Eigen::VectorXd bias_pos = hip_positions[i];
    if(_gait_generator->leg_state[i]==stance_leg || _gait_generator->leg_state[i]==Early_Contact){

      Eigen::Vector3d angs;
      angs.setConstant(0);
      Eigen::Vector3d ansV;
      ansV.setConstant(0);

      /******************* test begin ******************/
      // Eigen::VectorXd foot_target_position1 = -(target_hip_horizontal_velocity * _gait_generator->stance_duration[i])/2
      //                             - _desired_height 
      //                             + bias_pos;
      // Position end_position1;
      // end_position1.x = foot_target_position1[0];
      // end_position1.y = foot_target_position1[1];
      // end_position1.z = foot_target_position1[2];

      // Position foot_position1 = get_swing_foot_trajectory1(_gait_generator->normalized_phase[i],phase_switch_foot_local_position1[i],end_position1);
      // postarget[i] = foot_position1;

      // //get joint[i] angles
      // Angle ans1;
      // Inv_kinematics(&ans1,&foot_position1,i);
      // for(int j(0);j<3;j++){
      //   angs[j] = ans1.q[j];
      // }

      // motor_torque[i] = pd_tau(legpos[i], legvel[i], angs, ansV, 80, 2.5);
      /******************* test end ******************/
      // stance phase
      motor_torque[i] = pd_tau(legpos[i], legvel[i], angs, ansV, 0.0, 5);
      

    }
  
    else{

      if (timer < t_stance) {
          Eigen::VectorXd foot_target_position = (target_hip_horizontal_velocity * _gait_generator->stance_duration[i])/2 - _desired_height 
                                      + bias_pos;
          Position end_position;
          end_position.x = foot_target_position[0];
          end_position.y = foot_target_position[1];
          end_position.z = foot_target_position[2];

          Position foot_position = get_swing_foot_trajectory(_gait_generator->normalized_phase[i],phase_switch_foot_local_position[i],end_position);
          postarget[i] = foot_position;
          Eigen::Vector3d Current_positionVector,Desired_positionVector;
          Desired_positionVector << foot_position.x,foot_position.y,foot_position.z;
          // get current position
          Position Current_positionxyz = getFootPositionInBaswFrame(nowang,i);
          Current_positionVector << Current_positionxyz.x,Current_positionxyz.y,Current_positionxyz.z;
          // get current velocity
          Eigen::Vector3d Current_velocityVector = calcu_Jaco(legpos[i],i)*legvel[i];
          // get virtual force
          Eigen::Vector3d pV;
          pV.setConstant(0);
          veltarget[i] = pV;
          Eigen::Vector3d virtualForce = swing_leg_controller::tau(Current_positionVector,Current_velocityVector,Desired_positionVector,pV);
          // pd_tau(Current_positionVector,Current_velocityVector,Desired_positionVector,dP,1000.0,15.0);
          motor_torque[i] = calcu_Jaco(legpos[i],i).transpose()*virtualForce;
      }

      else{
      
          // get desired foot placement in world frame
          double rotation_angle_bias = 0.5 * dyaw_com * _gait_generator->stance_duration[i]
                        + dyaw_com * (1-_gait_generator->normalized_phase[i]) * _gait_generator->swing_duration[i]
                        + 0.005 * (dyaw_com - desired_dw_com);
          double rotation_xpos_bias = feetR[i] * cos(yaw_com + feetInitAngle[i] + rotation_angle_bias);
          double rotation_ypos_bias = feetR[i] * sin(yaw_com + feetInitAngle[i] + rotation_angle_bias);
          Eigen::VectorXd foot_target_position = 0.5 * linear_velocity * _gait_generator->stance_duration[i]
                                              + linear_velocity * (1-_gait_generator->normalized_phase[i]) * _gait_generator->swing_duration[i]
                                              + _KP * (linear_velocity - desired_linear_velocity);
          foot_target_position[0] += rotation_xpos_bias + p_com[0];
          foot_target_position[1] += rotation_ypos_bias + p_com[1];
          foot_target_position[2] = 0;

          // get beginning foot position in world frame
          Eigen::Vector3d foot_position_now = get_swing_foot_trajectory(_gait_generator->normalized_phase[i],foot_position_begin[i],foot_target_position);
          postarget_global[i].x = foot_position_now[0];
          postarget_global[i].y = foot_position_now[1];
          postarget_global[i].z = foot_position_now[2];

          /**********change here begin */

          Eigen::Vector3d Current_positionVector,Desired_positionVector;
          // get desired position
          // from world to body frame
          Eigen::Vector3d foot_position = com_rotm.transpose() * (foot_position_now-p_com);
          postarget[i].x = foot_position[0];
          postarget[i].y = foot_position[1];
          postarget[i].z = foot_position[2];
          Desired_positionVector = foot_position;
          // get current position
          Position Current_positionxyz = getFootPositionInBaswFrame(nowang,i);
          Current_positionVector << Current_positionxyz.x,Current_positionxyz.y,Current_positionxyz.z;
          // get desired velocity
          // from world to body frame
          Eigen::VectorXd cross_result = omegaBody.cross(Current_positionVector);
          Eigen::Vector3d fV = com_rotm.transpose() * (dP-com_velocity) - cross_result;
          // get current velocity
          Eigen::Vector3d Current_velocityVector = calcu_Jaco(legpos[i],i)*legvel[i];
          // get virtual force
          veltarget[i] = fV;
          fV.setConstant(0);
          Eigen::Vector3d virtualForce = swing_leg_controller::tau(Current_positionVector,Current_velocityVector,Desired_positionVector,fV);
          // pd_tau(Current_positionVector,Current_velocityVector,Desired_positionVector,dP,1000.0,15.0);
          motor_torque[i] = calcu_Jaco(legpos[i],i).transpose()*virtualForce;

          /**********change here end */

          /******************* test begin ******************/
          // Eigen::VectorXd foot_target_position = (target_hip_horizontal_velocity * _gait_generator->stance_duration[i])/2 - _desired_height 
          //                             + bias_pos;
          // Position end_position;
          // end_position.x = foot_target_position[0];
          // end_position.y = foot_target_position[1];
          // end_position.z = foot_target_position[2];

          // Position foot_position = get_swing_foot_trajectory(_gait_generator->normalized_phase[i],phase_switch_foot_local_position[i],end_position);
          // postarget[i] = foot_position;
          // Eigen::Vector3d Current_positionVector,Desired_positionVector;
          // Desired_positionVector << foot_position.x,foot_position.y,foot_position.z;
          // // get current position
          // Position Current_positionxyz = getFootPositionInBaswFrame(nowang,i);
          // Current_positionVector << Current_positionxyz.x,Current_positionxyz.y,Current_positionxyz.z;
          // // get current velocity
          // Eigen::Vector3d Current_velocityVector = calcu_Jaco(legpos[i],i)*legvel[i];
          // // get virtual force
          // Eigen::Vector3d pV;
          // pV.setConstant(0);
          // veltarget[i] = pV;
          // Eigen::Vector3d virtualForce = swing_leg_controller::tau(Current_positionVector,Current_velocityVector,Desired_positionVector,pV);
          // // pd_tau(Current_positionVector,Current_velocityVector,Desired_positionVector,dP,1000.0,15.0);
          // motor_torque[i] = calcu_Jaco(legpos[i],i).transpose()*virtualForce;
          /******************* test end ******************/
      }
      // //get joint[i] angles
      Angle ans;
      Inv_kinematics(&ans,&postarget[i],i);
      angleTarget[i] = ans;
      // for(int j(0);j<3;j++){
      //   angs[j] = ans.q[j];
      // }

      //get joint[i] anglesV
      // angveltarget[i] = calcu_Jaco(legpos[i],i).inverse()*veltarget[i];

      // // swing phase
      // motor_torque[i] = pd_tau(legpos[i], legvel[i], angs, ansV, 80.0, 2.5);

    }
    

  }


  leg_motor_torque << motor_torque[0], motor_torque[1];
  return leg_motor_torque;


}

void swing_leg_controller::set_PDGain(){
	pGain.resize(3);
	dGain.resize(3);
	// pGain << 1300,1300,1300;
	// dGain << 30,  30,  30;
  pGain << 1000,1000,1200;
	dGain << 10,  10,  15;
  // pGain << 0,0,0;
	// dGain << 0,0,0;

}

Eigen::VectorXd swing_leg_controller::tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT){
  
  return dGain.cwiseProduct(vT-vA) + pGain.cwiseProduct(pT-pA);

}

Eigen::VectorXd pd_tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT, float p_num, float d_num){
  
  Eigen::VectorXd p_gain(3);
  Eigen::VectorXd d_gain(3);
  p_gain.setConstant(p_num);
  d_gain.setConstant(d_num);
  return d_gain.cwiseProduct(vT-vA) + p_gain.cwiseProduct(pT-pA);

}

float gen_parabola(float phase, float start, float mid, float end){
  /*** Gets a point on a parabola y = a x^2 + b x + c.

  The Parabola is determined by three points (0, start), (0.5, mid), (1, end) in
  the plane.

  Args:
    phase: Normalized to [0, 1]. A point on the x-axis of the parabola.
    start: The y value at x == 0.
    mid: The y value at x == 0.5.
    end: The y value at x == 1.

  Returns:
    The y value at x == phase.
  ***/
  float mid_phase = 0.5;
  float delta_1 = mid - start;
  float delta_2 = end - start;
  float delta_3 = mid_phase*mid_phase - mid_phase;
  float coef_a = (delta_1 - delta_2 * mid_phase) / delta_3;
  float coef_b = (delta_2 * mid_phase*mid_phase - delta_1) / delta_3;
  float coef_c = start;

  return coef_a * phase * phase + coef_b * phase + coef_c;
}

Position gen_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos){
  /*** Generates the swing trajectory using a parabola.

  Args:
    input_phase: the swing/stance phase value between [0, 1].
    start_pos: The foot's position at the beginning of swing cycle.
    end_pos: The foot's desired position at the end of swing cycle.

  Returns:
    The desired foot position at the current phase.
  ***/

  float phase = input_phase;
  if(input_phase <= 0.5) phase = 0.8 * sin(input_phase * PII);
  else phase = 0.8 + (input_phase - 0.5) * 0.4;

  Position pos;
  pos.x = (1 - phase) * start_pos.x + phase * end_pos.x;
  pos.y = (1 - phase) * start_pos.y + phase * end_pos.y;
  float max_clearance = 0.1;
  float mid = std::max(end_pos.z, start_pos.z) + max_clearance;
  pos.z = gen_parabola(phase, start_pos.z, mid, end_pos.z);

  return pos;
}

// add the cycloid
// Eigen::VectorXd simple_cal_p(float p_start, float p_end, float period, float t_whole, bool isZ){
//   float phase = period;
//   period = period * t_whole;
//   Eigen::Vector3d ans;
//   if (isZ) {
//     t_whole = t_whole * 0.5;
//     float p_des = p_start + (p_end - p_start) * (period / t_whole - sin(2 * PII * period / t_whole) / (2 * PII));
//     float v_des = (p_end - p_start) / t_whole * (1 - cos(2 * PII * period / t_whole));
//     float a_des = 2 * PII * sin(2 * PII * period / t_whole)*(p_end - p_start) / (t_whole * t_whole);
//     ans << p_des, v_des, a_des;
//   }
//   else{
//     if (phase <= 0.99){
//       t_whole = t_whole * 0.99;
//       float p_des = p_start + (p_end - p_start) * (period / t_whole - sin(2 * PII * period / t_whole) / (2 * PII));
//       float v_des = (p_end - p_start) / t_whole * (1 - cos(2 * PII * period / t_whole));
//       float a_des = 2 * PII * sin(2 * PII * period / t_whole)*(p_end - p_start) / (t_whole * t_whole);
//       ans << p_des, v_des, a_des;
//     }
//     else{
//       ans << p_end, 0, 0;
//     }
    
//   }
  

//   return ans;
// }

Eigen::VectorXd simple_cal_p(float p_start, float p_end, float period, float t_whole, bool isZ){

  period = period * t_whole;

  if (isZ) {
    t_whole = t_whole * 0.5;
  }
  float p_des = p_start + (p_end - p_start) * (period / t_whole - sin(2 * PII * period / t_whole) / (2 * PII));
  float v_des = (p_end - p_start) / t_whole * (1 - cos(2 * PII * period / t_whole));
  float a_des = 2 * PII * sin(2 * PII * period / t_whole)*(p_end - p_start) / (t_whole * t_whole);
  Eigen::Vector3d ans;
  ans << p_des, v_des, a_des;
  return ans;
}
Position get_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos){
  float phase = input_phase;

  Eigen::Vector3d p_x;
  Eigen::Vector3d p_y;
  Eigen::Vector3d p_z;
  Position pos;

  p_x = simple_cal_p(start_pos.x, end_pos.x, phase, t_swing, false);
  p_y = simple_cal_p(start_pos.y, end_pos.y, phase, t_swing, false);

  float max_clearance = 0.1;
  float mid = std::max(end_pos.z, start_pos.z) + max_clearance;
  
  if (phase < 0.5) {
    p_z = simple_cal_p(start_pos.z, mid, phase, t_swing, true);
  }
  else {
    p_z = simple_cal_p(mid, end_pos.z, phase-0.5, t_swing, true);
  }
  pos.x = p_x[0];
  pos.y = p_y[0];
  pos.z = p_z[0];

  dP[0] = p_x[1];
  dP[1] = p_y[1];
  dP[2] = p_z[1];

  return pos;
}

Eigen::Vector3d get_swing_foot_trajectory(float input_phase, Eigen::Vector3d start_pos, Eigen::Vector3d end_pos){
  float phase = input_phase;

  Eigen::Vector3d p_x;
  Eigen::Vector3d p_y;
  Eigen::Vector3d p_z;
  Eigen::Vector3d pos;

  p_x = simple_cal_p(start_pos[0], end_pos[0], phase, t_swing, false);
  p_y = simple_cal_p(start_pos[1], end_pos[1], phase, t_swing, false);

  float max_clearance = 0.1;
  float mid = std::max(end_pos[2], start_pos[2]) + max_clearance;

  if (phase < 0.5) {
    p_z = simple_cal_p(start_pos[2], mid, phase, t_swing, true);
  }
  else {
    p_z = simple_cal_p(mid, end_pos[2], phase-0.5, t_swing, true);
  }
  pos[0] = p_x[0];
  pos[1] = p_y[0];
  pos[2] = p_z[0];

  dP[0] = p_x[1];
  dP[1] = p_y[1];
  dP[2] = p_z[1];

  return pos;
}

Position get_swing_foot_trajectory1(float input_phase, Position start_pos, Position end_pos){

  float phase = input_phase;

  Position pos;
  pos.x = (1 - phase) * start_pos.x + phase * end_pos.x;
  pos.y = (1 - phase) * start_pos.y + phase * end_pos.y;
  pos.z = (1 - phase) * start_pos.z + phase * end_pos.z;

  return pos;
}