#ifndef __BIKEBOT_CONTROL_H
#define __BIKEBOT_CONTROL_H

#include "motor_drive.h"
#include <fstream>
#include "licycle_config.h"
#include <vector>

//define motor_cmd_unit
typedef struct{
	float p;
	float v;
	float kp;
	float kd;
    float t;
} Motor_cmd_unit;
//define motor_cmd
typedef struct{
	Motor_cmd_unit cmd[6];
} Motor_cmd;

//定义leg_state结构体
typedef struct{
    CBData cbdata[6];
    float rpy[3];
    float omega[3];
    float omega_world[3];
    float acc[3];
    float com_velocity[3];
    float com_position[3];
    Eigen::Vector3d foot_p[2];
    float com_height;
    float vicon_pos[3];
    float vicon_rpy[3];
    float vicon_COMvel[3];
    float vicon_COMpos[3];
}  Leg_state;

//定义leg_cmd结构体
typedef struct{
    float torque[6];
}  Leg_command;

void thread_setup(void);
void control_threadcreate(void);

void can0_tx(uint8_t tdata[],uint8_t id);
void can1_tx(uint8_t tdata[],uint8_t id);
void legstate_update();
void motor_control(Leg_command leg_cmd);
void motor_cmd_write(Motor_cmd Mcmd);
void CAN_init();
void Sleep_us(int us);
void reset_motors();
void setup_motors();
void damp_motors();
void setpoint(float x,float y,float z);
void setpoint1(float x,float y,float z);
void footPoint_pos_Inf();

#endif