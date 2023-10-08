#ifndef __OPMATH_H
#define __OPMATH_H

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#define PI 3.1415926535

//math tool
Eigen::Matrix3d rpy2romatrix(double roll,double pitch,double yaw);

#endif