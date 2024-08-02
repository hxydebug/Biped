#include "leg_controller.h"
using namespace std;

float a0[2][3] = {0};
float a1[2][3] = {0};
float a2[2][3] = {0};
float a3[2][3] = {0};

float kk[2][3] = {0};
float bb[2][3] = {0};

double rollMax = 2.0*PI/180;

float kp = 8;
float kd = 0.2;
float vel_ref = 0;


Inform information;
float global_force;
float global_Tau;
int if_getTau = 0;
double collision_height;

int impact_happened = 0;
int timeflag = 0;
int timecount = 0;
int start_count = 0;

Position init_pos[2];
Angle init_angle[2];

leg_controller::leg_controller(Leg_state *robot,gait_generator *gait_gen,swing_leg_controller *swc, stance_leg_controller *stc){
	leg = robot;
	leg_controller::set_PDGain();
	posT.resize(6);
  	angT.resize(6);
	//set the initial position
	set_xyz(l_leg,&init_angle[0],0.06,0.11,-0.32);//0.06,0.11,-0.32
  	set_xyz(r_leg,&init_angle[1],0.06,-0.11,-0.32);

	posT << init_angle[0].q[0],init_angle[0].q[1],init_angle[0].q[2],
          init_angle[1].q[0],init_angle[1].q[1],init_angle[1].q[2];
  	angT << 0,0,0,0,0,0;

	Kinematics(&init_angle[0],&init_pos[0],0);
    Kinematics(&init_angle[1],&init_pos[1],1);

	gait_generate = gait_gen;
	swctr = swc;
	stctr = stc;

	timer = 0;
}

void leg_controller::set_PDGain(){
	pGain.resize(6);
	dGain.resize(6);
	pGain.setConstant(20.0);
	dGain.setConstant(0.2);

}

void leg_controller::get_inform(double i0,double i1,double i2,double i3,double i4,double i5){
	information.q[0] = i0;
	information.q[1] = i1;
	information.q[2] = i2;
	information.q[3] = i3;
	information.q[4] = i4;
	information.q[5] = i5;
}
void leg_controller::get_Tau(double Tau){
	if_getTau = 1;
	global_Tau = fabs(Tau);
}

Posdiff leg_controller::get_error(void){
	return poserror;
}

void leg_controller::get_action(Motor_cmd *Mcmd, int Run_mode, Eigen::VectorXd stc_tau, Eigen::VectorXd user_cmd){

	Eigen::VectorXd Tau_e(6);
	// calculate the current joint angles
	Eigen::VectorXd cur_angle(6);
	Eigen::VectorXd cur_angleV(6);
	for(int i(0);i<6;i++){
			cur_angle[i] = leg->cbdata[i].p;
			cur_angleV[i] = leg->cbdata[i].v;
	}

	if(Run_mode==0){
		pGain.setConstant(20.0);
		
		Tau_e = leg_controller::tau(cur_angle,cur_angleV,posT,angT);
	}
	else{
		//gait_generator
		gait_generate->update(timer);
		
		// ground reaction force calculate    ***** it can be put in another pthread *****
		if (timer < 0.04){
			stc_tau.setConstant(0);
		}
		// stc_tau.setConstant(0);

		// initial
		for(int i(0);i<6;i++){
			Mcmd->cmd[i].p = 0;
			Mcmd->cmd[i].v = 0;
		}

		// position controller
		swctr->update(timer);
		Eigen::VectorXd swc_tau = swctr->get_action(user_cmd);

		Eigen::VectorXd ltau(3),rtau(3);
		if (gait_generate->leg_state[0] == stance_leg || gait_generate->leg_state[0] == Early_Contact) {
			// left leg stance
			ltau = swc_tau.head(3) + stc_tau.head(3);
			Mcmd->cmd[0].kp = 0;
			Mcmd->cmd[1].kp = 0;
			Mcmd->cmd[2].kp = 0;
			Mcmd->cmd[0].kd = 0;
			Mcmd->cmd[1].kd = 0;
			Mcmd->cmd[2].kd = 0;
		}
		else {
			// left leg swing
			ltau = swc_tau.head(3);
			float Mkp = 65;
			float Mkd = 2.5;
			Mcmd->cmd[0].kp = Mkp;
			Mcmd->cmd[1].kp = Mkp;
			Mcmd->cmd[2].kp = Mkp;
			Mcmd->cmd[0].kd = Mkd;
			Mcmd->cmd[1].kd = Mkd;
			Mcmd->cmd[2].kd = Mkd;
			if (gait_generate->normalized_phase[0]>0.9){
				Mkp = 10;
				Mkd = 2.5;
				Mcmd->cmd[0].kp = Mkp;
				Mcmd->cmd[1].kp = Mkp;
				Mcmd->cmd[2].kp = Mkp;
				Mcmd->cmd[0].kd = Mkd;
				Mcmd->cmd[1].kd = Mkd;
				Mcmd->cmd[2].kd = Mkd;
			}
			
			// Mcmd->cmd[0].v = swctr->angveltarget[0][0];
			// Mcmd->cmd[1].v = swctr->angveltarget[0][1];
			// Mcmd->cmd[2].v = swctr->angveltarget[0][2];
			Mcmd->cmd[0].p = swctr->angleTarget[0].q[0];
			Mcmd->cmd[1].p = swctr->angleTarget[0].q[1];
			Mcmd->cmd[2].p = swctr->angleTarget[0].q[2];

		}
		if (gait_generate->leg_state[1] == stance_leg || gait_generate->leg_state[1] == Early_Contact) {
			// right leg stance
			rtau = swc_tau.tail(3) + stc_tau.tail(3);
			Mcmd->cmd[3].kp = 0;
			Mcmd->cmd[4].kp = 0;
			Mcmd->cmd[5].kp = 0;
			Mcmd->cmd[3].kd = 0;
			Mcmd->cmd[4].kd = 0;
			Mcmd->cmd[5].kd = 0;

		}
		else {
			// right leg swing
			rtau = swc_tau.tail(3);
			float Mkp = 65;
			float Mkd = 2.5;
			Mcmd->cmd[3].kp = Mkp;
			Mcmd->cmd[4].kp = Mkp;
			Mcmd->cmd[5].kp = Mkp;
			Mcmd->cmd[3].kd = Mkd;
			Mcmd->cmd[4].kd = Mkd;
			Mcmd->cmd[5].kd = Mkd;
			if (gait_generate->normalized_phase[1]>0.9){
				Mkp = 10;
				Mkd = 2.5;
				Mcmd->cmd[3].kp = Mkp;
				Mcmd->cmd[4].kp = Mkp;
				Mcmd->cmd[5].kp = Mkp;
				Mcmd->cmd[3].kd = Mkd;
				Mcmd->cmd[4].kd = Mkd;
				Mcmd->cmd[5].kd = Mkd;
			}
			// Mcmd->cmd[3].v = swctr->angveltarget[1][0];
			// Mcmd->cmd[4].v = swctr->angveltarget[1][1];
			// Mcmd->cmd[5].v = swctr->angveltarget[1][2];
			Mcmd->cmd[3].p = swctr->angleTarget[1].q[0];
			Mcmd->cmd[4].p = swctr->angleTarget[1].q[1];
			Mcmd->cmd[5].p = swctr->angleTarget[1].q[2];

		}

		Tau_e << ltau, rtau;

		timer += 0.001;
	}

	
	for(int i(0);i<6;i++){
		Mcmd->cmd[i].t = Tau_e[i];
	}

	//torque_limit
	for(int i(0);i<3;i++){
		if(Mcmd->cmd[i].t < -28.0) Mcmd->cmd[i].t = -28.0;
		if(Mcmd->cmd[i].t > 28.0) Mcmd->cmd[i].t = 28.0;
	}
	for(int i(3);i<6;i++){
		if(Mcmd->cmd[i].t < -28.0) Mcmd->cmd[i].t = -28.0;
		if(Mcmd->cmd[i].t > 28.0) Mcmd->cmd[i].t = 28.0;
	}
	// cout<<Tau_e<<endl;
}

Eigen::VectorXd leg_controller::tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT){
  
  return dGain.cwiseProduct(vT-vA) + pGain.cwiseProduct(pT-pA);

}

void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz){
  Position pos;
  pos.x = xx;
  pos.y = yy;
  pos.z = zz;

  Inv_kinematics_ref(angle,&pos,leg);
}

void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float tf,Leg leg){
	
	float p0[3];
	float pf[3];
	float v0[3];
	float vf[3];
	
	p0[0] = pini->x;
	p0[1] = pini->y;
	p0[2] = pini->z;
	
	pf[0] = pdes->x;
	pf[1] = pdes->y;
	pf[2] = pdes->z;
	
	v0[0] = vini->x;
	v0[1] = vini->y;
	v0[2] = vini->z;
	
	vf[0] = vdes->x;
	vf[1] = vdes->y;
	vf[2] = vdes->z;
	
	for(int i=0;i<3;i++){
		a0[leg][i] = p0[i];
		a1[leg][i] = v0[i];
		a2[leg][i] = 3.0/(tf*tf)*(pf[i]-p0[i])-2.0/tf*v0[i]-1.0/tf*vf[i];
		a3[leg][i] = -2.0/(tf*tf*tf)*(pf[i]-p0[i])+1.0/(tf*tf)*(vf[i]+v0[i]);
	}
}

void chabu(Position *pos,float step,Leg leg){
	float p[3];
	for(int i=0;i<3;i++){
		p[i] = a0[leg][i] + a1[leg][i]*step + a2[leg][i]*step*step +a3[leg][i]*step*step*step;
	}
	pos->x = p[0];
	pos->y = p[1];
	pos->z = p[2];
	
}

void init_linear(Position *pdes,Position *pini,float tf,Leg leg){

	float p0[3];
	float pf[3];

	p0[0] = pdes->x;
	p0[1] = pini->y;
	p0[2] = pini->z;

	pf[0] = pdes->x;
	pf[1] = pdes->y;
	pf[2] = pdes->z;

	for(int i=0;i<3;i++){
		bb[leg][i] = p0[i];
		kk[leg][i] = (pf[i]-p0[i])/tf;
		
	}
}

void linear(Position *pos,float step,Leg leg){
	float p[3];
	for(int i=0;i<3;i++){
		p[i] = bb[leg][i] + kk[leg][i]*step;
	}
	pos->x = p[0];
	pos->y = p[1];
	pos->z = p[2];
	
}

float linear(float ini,float des,float tf,float step){
	float p0 = ini;
	float pf = des;
	float b0,b1;
	b0 = p0;
	b1 = (pf-p0)/tf;
	float p = b0+b1*step;
	return p;
}