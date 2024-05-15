#include "control.h"

//general
Eigen::MatrixXd calcu_Jaco(Eigen::Vector3d angle,Leg leg){
	Eigen::MatrixXd Mat(3,3);
    float q0 = angle[0];
    float q1 = angle[1];
    float q2 = angle[2];
    float L0 = Len0;
    float L1 = Len1;
    float L2 = Len2;
	if(leg == l_leg){
		Mat << 0, L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L1*cos(q1), L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L0*sin(q0) + L1*cos(q0)*cos(q1), - L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - L1*sin(q0)*sin(q1), -L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0),   L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1),  L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));
	}
	else{
		Mat << 0, -L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L1*cos(q1), -L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			L0*sin(q0) + L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) + L1*cos(q0)*cos(q1), - L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - L1*sin(q0)*sin(q1), -L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			L1*cos(q1)*sin(q0) - L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)),   L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1),  L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));				
	}
	return Mat;
}

Eigen::MatrixXd calcu_Jaco(Angle angle,Leg leg){
	Eigen::MatrixXd Mat(3,3);
	float q0 = angle.q[0];
	float q1 = angle.q[1];
	float q2 = angle.q[2];
	float L0 = Len0;
	float L1 = Len1;
	float L2 = Len2;
	if(leg == l_leg){
		Mat << 0, L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L1*cos(q1), L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L0*sin(q0) + L1*cos(q0)*cos(q1), - L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - L1*sin(q0)*sin(q1), -L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0),   L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1),  L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));
	}
	else{
		Mat << 0, -L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L1*cos(q1), -L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			L0*sin(q0) + L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) + L1*cos(q0)*cos(q1), - L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) - L1*sin(q0)*sin(q1), -L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			L1*cos(q1)*sin(q0) - L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)),   L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1),  L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));				
	}
	return Mat;
}

void Kinematics(Angle *angle,Position *position,Leg leg){

  float q0 = angle->q[0];
  float q1 = -angle->q[1];
  float q2 = -angle->q[2];
  float L0 = Len0;
  float L1 = Len1;
  float L2 = Len2;
  if(leg == fl){
    position->x = -L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - L1*sin(q1);
    position->y = L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0);
    position->z = L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);
    position->y += width/2;
    position->x -= detx;
    position->z -= detz;
  }
  else{
    position->x = L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L1*sin(q1);
    position->y = -L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0);
    position->z = -L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);
    position->y -= width/2;
    position->x -= detx;
    position->z -= detz;
  }

}

void Inv_kinematics(Angle *angle,Position *position,Leg leg){
	
  float x = position->x;
  float y = position->y;
  float z = position->z;
  float L0 = Len0;
  float L1 = Len1;
  float L2 = Len2;
  if(leg == fl){
    y -= width/2;
    x += detx;
    z += detz;
    x = -x;

    angle->q[2] = acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((z+sqrt(y*y+z*z-L0*L0))/(y+L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = -x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
	else{

    y += width/2;
    x += detx;
    z += detz;
    x = -x;

		angle->q[2] = -acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((z+sqrt(y*y+z*z-L0*L0))/(y-L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
}

void Kinematics_ref(Angle *angle,Position *position,Leg leg){

  float q0 = angle->q[0];
  float q1 = -angle->q[1];
  float q2 = -angle->q[2];
  float L0 = Len0;
  float L1 = Len1;
  float L2 = Len2;
  if(leg == fl){
    position->x = -L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - L1*sin(q1);
    position->y = L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0);
    position->z = L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);
  }
  else{
    position->x = L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L1*sin(q1);
    position->y = -L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0);
    position->z = -L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);

  }

}

void Inv_kinematics_ref(Angle *angle,Position *position,Leg leg){
	
  float x = -position->x;
  float y = position->y;
  float z = position->z;
  float L0 = Len0;
  float L1 = Len1;
  float L2 = Len2;
  if(leg == fl){
    angle->q[2] = acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((z+sqrt(y*y+z*z-L0*L0))/(y+L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = -x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
	else{

		angle->q[2] = -acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((z+sqrt(y*y+z*z-L0*L0))/(y-L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
}

Position getFootPositionInBaswFrame(Eigen::VectorXd angle, Leg leg){
  auto pos = angle;
  Position nowposi;
  Angle nowang;
  if(leg == l_leg){
    for(int i(0);i<3;i++){
        nowang.q[i] = pos[i];
    }
  }
  else{
    for(int i(0);i<3;i++){
        nowang.q[i] = pos[i+3];
    }
  }

  Kinematics(&nowang,&nowposi,leg);
  return nowposi;
  
}