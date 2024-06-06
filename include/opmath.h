#ifndef __OPMATH_H
#define __OPMATH_H

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#define PI 3.1415926535

//math tool
Eigen::Matrix3d rpy2romatrix(double roll,double pitch,double yaw);
Eigen::AngleAxisd romatrix2AngleAxis(Eigen::Matrix3d rotation_matrix);
void quaToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy);
void toEulerAngle(Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
#endif