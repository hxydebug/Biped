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



