#ifndef __LEG_CONTROL_H
#define __LEG_CONTROL_H

#include "opmath.h"
#include "bikebot_control.h"
#include "swing_leg_controller.h"
#include "stance_leg_controller.h"
#include "control.h"

typedef struct{
	float error[6];
} Posdiff;

typedef struct{
	float q[6];
} Inform;


class leg_controller{

public:
	leg_controller(Leg_state *robot,gait_generator *gait_gen,swing_leg_controller *swc, stance_leg_controller *stc);
	void get_action(Leg_command *cmd,int Run_mode, Eigen::VectorXd stc_tau);
	void goto_xyz(float xx,float yy,float zz,Leg direction);
	void get_inform(double i0,double i1,double i2,double i3,double i4,double i5);
	void get_Tau(double Tau);
	Posdiff get_error(void);
	void set_PDGain();
	Eigen::VectorXd tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT);

private:
	Leg_state *leg;

	Angle angle[2];
	Angle angleV[2];
	Position position[2];
	Position velocity[2];
	Posdiff poserror;
	Eigen::VectorXd pGain,dGain;
	Eigen::VectorXd posT,angT;

	std::vector<float> x_position;
	std::vector<float> z_position;

	gait_generator *gait_generate;
 	swing_leg_controller *swctr;
	stance_leg_controller *stctr;
};

//leg model
void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz);
void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float step,Leg leg);
void chabu(Position *pos,float step,Leg leg);
void init_linear(Position *pdes,Position *pini,float tf,Leg leg);
void linear(Position *pos,float step,Leg leg);
float linear(float ini,float des,float tf,float step);


#endif