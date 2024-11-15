#include "motor_drive.h"
#include <math.h>

float rate = 1.0;

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void cb_transfer(CANMessage msg,CBData *cb){
    int p_int = (msg.data[1]<<8)|msg.data[2];
    int v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
    int t_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
    cb->ID = msg.data[0];
    cb->p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    cb->v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    cb->t = uint_to_float(t_int, T_MIN, T_MAX, 12);
}

void cb_trans(CANMessage *msg,CBData *cb){
		for(int i=0;i<6;i++){
			cb_transfer(msg[i],cb + i);
            cb[i].p += -initial_angle_bias[i];
		}
		
}

void cb_Inf(CBData *cb){
		printf("ID:%d ",cb->ID);
		printf("p:%f ",cb->p*180.0/PII);
//		printf("p:%f ",cb->p);
//		printf("v:%f ",cb->v);
		// printf("T:%f ",cb->t);
		printf("\n");
}

void cmd_transfer(uint8_t id,CANMessage *msg,float p,float v,float kp,float kd,float t){
    if(id <= 0) id = 1;
    if(id >= 7) id = 6;
    
    //角度限制
    if(id == 1){
        if(p < -50.0*PII/180.0) p = -50.0*PII/180.0;
        if(p > 60.0*PII/180.0) p = 60.0*PII/180.0;
    }
    if(id == 4){
        if(p < -60.0*PII/180.0) p = -60.0*PII/180.0;
        if(p > 50.0*PII/180.0) p = 50.0*PII/180.0;
    }

    if(id == 2){
        if(p < -120.0*PII/180.0) p = -120.0*PII/180.0;
        if(p > 60.0*PII/180.0) p = 60.0*PII/180.0;
    }
    if(id == 5){
        if(p < -60.0*PII/180.0) p = -60.0*PII/180.0;
        if(p > 120.0*PII/180.0) p = 120.0*PII/180.0;
    }
    
    if(id == 3){
        if(p < 35.0*PII/180.0) p = 35.0*PII/180.0;
        if(p > 150.0*PII/180.0) p = 150.0*PII/180.0;
    }
    if(id == 6){
        if(p < -150.0*PII/180.0) p = -150.0*PII/180.0;
        if(p > -35.0*PII/180.0) p = -35.0*PII/180.0;
    }

    //return absolute angle by plus bias angle
    int i = id - 1;
    p += initial_angle_bias[i];
    
    //力矩限制
    if(t < -28.0) t = -28.0;
    if(t > 28.0) t = 28.0;

    int p_int = float_to_uint(p,P_MIN,P_MAX,16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(t, T_MIN, T_MAX, 12);
    msg->data[0] = p_int>>8;
    msg->data[1] = p_int&0xFF;
    msg->data[2] = v_int>>4;
    msg->data[3] = ((v_int&0xF)<<4) + (kp_int>>8);
    msg->data[4] = kp_int&0xFF;
    msg->data[5] = kd_int>>4;
    msg->data[6] = ((kd_int&0xF)<<4) + (t_int>>8);
    msg->data[7] = t_int&0xFF;
}