#ifndef __CONTROL_H
#define __CONTROL_H
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "opmath.h"

#define fl 0
#define fr 1

#define l_leg 0
#define r_leg 1

#define Len0 0.08915f
#define Len1 0.2202f
#define Len2 0.24816f
#define Len2_change 0.23f

#define wid 0.1f
#define detx 0.007f
#define detz 0.046f

typedef int Leg;

typedef struct{
	float q[3];
} Angle;

typedef struct{
	float x;
	float y;
	float z;
} Position;

//general
void Kinematics(Angle *angle,Position *position,Leg leg);
void Inv_kinematics(Angle *angle,Position *position,Leg leg);
void Kinematics_ref(Angle *angle,Position *position,Leg leg);
void Inv_kinematics_ref(Angle *angle,Position *position,Leg leg);
void Kinematics_L2change(Angle *angle,Position *position,Leg leg);
Eigen::MatrixXd calcu_Jaco(Eigen::Vector3d angle,Leg leg);
Eigen::MatrixXd calcu_Jaco(Angle angle,Leg leg);
Position getFootPositionInBaswFrame(Eigen::VectorXd angle, Leg leg);
Position getFootPositionInBaswFrame_L2change(Eigen::VectorXd angle, Leg leg);



#endif