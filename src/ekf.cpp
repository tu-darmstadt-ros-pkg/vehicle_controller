/// HEADER
#include <vehicle_controller/ekf.h>

/// SYSTEM
#include <Eigen/Dense>
#include <ros/console.h>

#include<tf/tf.h>
#include<ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

namespace {

double angleDiff(const double a1, const double a2)
{
    return std::atan2(std::sin(a1 - a2), std::cos(a1 - a2)) ;
}

}

EKF::EKF()
{
    x_ = Eigen::Matrix<double, 6, 1>::Zero();

//    x_(0,0) = 0;
//    x_(1,0) = 0;
//    x_(2,0) = 0;
//    x_(3,0) = -0.45;
//    x_(4,0) = 0.45;
//    x_(5,0) = -0.004;

    x_(0,0) = 0;
    x_(1,0) = 0;
    x_(2,0) = 0;
    x_(3,0) = -0.15;
    x_(4,0) = 0.15;
    x_(5,0) = -0.004;


    F = Eigen::Matrix<double, 6, 6>::Zero();
    P = Eigen::Matrix<double, 6, 6>::Identity();
    R = Eigen::Matrix<double, 3, 3>::Zero();
    Q = Eigen::Matrix<double, 6, 6>::Zero();

    R(0,0) = pow(0.0001, 2);
    R(1,1) = pow(0.0001, 2);
    R(2,2) = pow(0.01*M_PI/180, 2);

    Q(0,0) = pow(0.001, 2);
    Q(1,1) = pow(0.001, 2);
    Q(2,2) = pow(0.05*M_PI/180, 2);
    Q(3,3) = pow(0.001, 2);
    Q(4,4) = pow(0.001, 2);
    Q(5,5) = pow(0.001, 2);

    P(3,3) = 1;
    P(3,4) = 1;
    P(4,3) = 1;
    P(4,4) = 1;
    P(5,5) = 1;

//    //Like in the paper
//    Q(0,0) = pow(0.2, 2);
//    Q(1,1) = pow(0.2, 2);
//    Q(2,2) = pow(5*M_PI/180, 2);
//    Q(3,3) = pow(0.01, 2);
//    Q(4,4) = pow(0.01, 2);
//    Q(5,5) = pow(0.01, 2);

}

void EKF::reset()
{   
    x_(0,0) = 0;
    x_(1,0) = 0;
    x_(2,0) = 0;
    x_(3,0) = -0.2;
    x_(4,0) = 0.2;
    x_(5,0) = -0.004;

    F = Eigen::Matrix<double, 6, 6>::Zero();
    P = Eigen::Matrix<double, 6, 6>::Identity();

    P(3,3) = 1;
    P(3,4) = 1;
    P(4,3) = 1;
    P(4,4) = 1;
    P(5,5) = 1;
}


//void EKF::predict(const std_msgs::Float64MultiArray::ConstPtr& array, double dt)
void EKF::predict(double Vl, double Vr, double pitch, double roll, double dt)
{

  if(fabs(x_(3,0) - x_(4,0)) < 0.01){
    ROS_INFO("y ICR for left and right track too close, resetting");
    reset();
  }

    double theta = x_(2,0);
    double y_ICRr = x_(3,0);
    double y_ICRl = x_(4,0);
    double x_ICR = x_(5,0);

    double vx = (Vr*y_ICRl - Vl*y_ICRr)/fabs(y_ICRl - y_ICRr) * std::cos(pitch);
    double vy = x_ICR*(Vl-Vr)/fabs(y_ICRl - y_ICRr) * std::cos(roll);
    double omega = -(Vl - Vr)/fabs(y_ICRl - y_ICRr) * std::cos(roll) * std::cos(pitch);

    x_(0,0) = x_(0,0) + dt*(vx*std::cos(theta) - vy*std::sin(theta));
    x_(1,0) = x_(1,0) + dt*(vy*std::cos(theta) + vx*std::sin(theta));
    x_(2,0) = theta + dt*omega;
    x_(3,0) = y_ICRr;
    x_(4,0) = y_ICRl;
    x_(5,0) = x_ICR;


    Eigen::Matrix<double, 6, 6> L;
    L = dt*Eigen::Matrix<double, 6, 6>::Identity();

//    F.block(0,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
//    F(0,2) = dt*(-vx*std::sin(theta) - vy*std::cos(theta));
//    F(1,2) = dt*(vx*std::cos(theta) - vy*std::sin(theta));
//    F.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Zero();
//    F.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
//    F(0,3) = dt*((y_ICRl*(Vr-Vl)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta)
//                 - (x_ICR*(Vl-Vr)/(pow(y_ICRl-y_ICRr,2)))*std::sin(theta));
//    F(0,4) = dt*((y_ICRr*(Vl-Vr)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta)
//                 + (x_ICR*(Vl-Vr)/(pow((y_ICRl-y_ICRr),2)))*std::sin(theta));
//    F(0,5) = -dt*((Vl-Vr)/(y_ICRl-y_ICRr))*std::sin(theta);
//    F(1,3) = dt*((y_ICRl*(Vr-Vl)/(pow((y_ICRl-y_ICRr),2)))*std::sin(theta)
//                 + (x_ICR*(Vl-Vr)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta));
//    F(1,4) = dt*((y_ICRr*(Vl-Vr)/(pow((y_ICRl-y_ICRr),2)))*std::sin(theta)
//                 - (x_ICR*(Vl-Vr)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta));
//    F(1,5) = dt*((Vl-Vr)/(y_ICRl-y_ICRr))*std::cos(theta);
//    F(2,3) = -dt*(Vl-Vr)/(pow((y_ICRl-y_ICRr),2));
//    F(2,4) = dt*(Vl-Vr)/(pow((y_ICRl-y_ICRr),2));
//    F(2,5) = 0;

    double Vl_x = Vl * std::cos(roll) * std::cos(pitch);
    double Vr_x = Vr * std::cos(roll) * std::cos(pitch);

    F.block(0,0,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
    F(0,2) = dt*(-vx*std::sin(theta) - vy*std::cos(theta));
    F(1,2) = dt*(vx*std::cos(theta) - vy*std::sin(theta));
    F.block(3,0,3,3) = Eigen::Matrix<double, 3, 3>::Zero();
    F.block(3,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity();
    F(0,3) = dt*((y_ICRl*(Vr_x-Vl_x)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta)
                 - (x_ICR*(Vl_x-Vr_x)/(pow(y_ICRl-y_ICRr,2)))*std::sin(theta));
    F(0,4) = dt*((y_ICRr*(Vl_x-Vr_x)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta)
                 + (x_ICR*(Vl_x-Vr_x)/(pow((y_ICRl-y_ICRr),2)))*std::sin(theta));
    F(0,5) = -dt*((Vl_x-Vr_x)/(y_ICRl-y_ICRr))*std::sin(theta);
    F(1,3) = dt*((y_ICRl*(Vr_x-Vl_x)/(pow((y_ICRl-y_ICRr),2)))*std::sin(theta)
                 + (x_ICR*(Vl_x-Vr_x)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta));
    F(1,4) = dt*((y_ICRr*(Vl_x-Vr_x)/(pow((y_ICRl-y_ICRr),2)))*std::sin(theta)
                 - (x_ICR*(Vl_x-Vr_x)/(pow((y_ICRl-y_ICRr),2)))*std::cos(theta));
    F(1,5) = dt*((Vl_x-Vr_x)/(y_ICRl-y_ICRr))*std::cos(theta);
    F(2,3) = -dt*(Vl_x-Vr_x)/(pow((y_ICRl-y_ICRr),2));
    F(2,4) = dt*(Vl_x-Vr_x)/(pow((y_ICRl-y_ICRr),2));
    F(2,5) = 0;


    P = F*P*F.transpose() + L*Q*L.transpose();
}


void EKF::correct(const Eigen::Vector3d& delta)
{

    Eigen::MatrixXd M = Eigen::Matrix<double, 3,3>::Identity();

    Eigen::Matrix<double, 3, 6> H;
    H.block(0,0,3,3) = M;
    H.block(0,3,3,3) = Eigen::Matrix<double, 3, 3>::Zero();

    Eigen::Matrix<double, 6, 3> K;
    Eigen::Matrix<double, 3, 3> T;

    T = H*P*H.transpose() + M*R*M.transpose();

    K = P*H.transpose()*T.inverse();

    Eigen::Vector3d innovation = delta - x_.block(0,0,3,1);

    innovation(2) = angleDiff(delta(2), x_(2));

    x_ += K*innovation;

    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();

    P = (I - K*H)*P;

//    for(int i=0;i<6;i++){
//      for(int j=0;j<6;j++){
//        std::cout << P(i,j) <<" , ";
//      }
//      std::cout << std::endl;
//    }
}
