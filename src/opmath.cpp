#include "opmath.h"

Eigen::Matrix3d rpy2romatrix(double roll,double pitch,double yaw){
    Eigen::Vector3d ea(roll, pitch, yaw);
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX());
    return rotation_matrix3;
}

Eigen::AngleAxisd romatrix2AngleAxis(Eigen::Matrix3d rotation_matrix){
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(rotation_matrix);
    return rotation_vector;
}

void quaToRpy(Eigen::Vector4d &qua,Eigen::Vector3d &rpy){
    // w, x, y, z
    Eigen::Quaterniond quaternion(qua[0],qua[1],qua[2],qua[3]);
    toEulerAngle(quaternion, rpy[0], rpy[1], rpy[2]);
}

void toEulerAngle(Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
if (fabs(sinp) >= 1)
pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
else
pitch = asin(sinp);

// yaw (z-axis rotation)
double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
yaw = atan2(siny_cosp, cosy_cosp);
}

Eigen::AngleAxisd rpy2AngleAxis(Eigen::Vector3d rpy){
    Eigen::Vector3d ea(rpy[0],rpy[1],rpy[2]);
    Eigen::AngleAxisd rotation_vector;
    rotation_vector = Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ()) * 
                      Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                      Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX());
    return rotation_vector;
}
