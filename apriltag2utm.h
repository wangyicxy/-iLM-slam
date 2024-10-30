#ifndef _APRILTAG2UTM_H
#define _APRILTAG2UTM_H


#include <ceres/ceres.h>

Eigen::Matrix<double,3,1> apr2utm(double b11,double b12, double b21, double b22, double b31, double b32, double b41, double b42,double d11, double d22, double d33 ,double d44,double& x, double& y, double& z);

#endif
