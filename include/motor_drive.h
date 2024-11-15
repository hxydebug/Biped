#ifndef __MOTOR_DRIVE_H
#define __MOTOR_DRIVE_H

#include <stdint.h>
#include <iostream>
#include "opmath.h"

#define biass 0


#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -48.0f
#define T_MAX 48.0f

static uint8_t reset[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static uint8_t set_foc[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

static float initial_angle_bias[6] = {0.6224, -0.7183, -1.4127, -1.4616, 0.4526, 0.5800};

typedef struct{
	uint8_t data[8];
} CANMessage;


typedef struct{
	int ID;
	float p;
	float v;
	float t;
} CBData;

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void cmd_transfer(uint8_t id,CANMessage *msg,float p,float v,float kp,float kd,float t);
void cb_transfer(CANMessage msg,CBData *cb);
void cb_Inf(CBData *cb);
void cb_trans(CANMessage *msg,CBData *cb);

#endif