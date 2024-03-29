#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"

struct EKF {
    EKF();

    void reset();

    //void predict(const std_msgs::Float64MultiArray::ConstPtr& array, double dt);
    void predict(double Vl, double Vr , double pitch, double roll, double dt);

    void correct(const Eigen::Vector3d &delta);

    Eigen::Matrix<double, 6, 1> x_;
    Eigen::Matrix<double, 6, 6> F;
    Eigen::Matrix<double, 6, 6> P;

    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 6, 6> Q;
};

#endif // EKF_H
