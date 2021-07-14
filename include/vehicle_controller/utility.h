#ifndef UTILITY_H
#define UTILITY_H

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>


struct Point
{
  float x;
  float y;
  float orientation;
};

struct Leg
{
  Point p1;
  Point p2;
  float course;

  bool backward;
  double speed;
  double length2;
  double length;
  double percent;
  ros::Time start_time;
  ros::Time finish_time;
};

typedef std::vector<Leg> Legs;

inline static double constrainAngle_0_2pi(double x)
{
    x = fmod(x, 2.0 * M_PI);
    if (x < 0)
        x += 2.0 * M_PI;
    return x;
}

inline static double constrainAngle_mpi_pi(double x)
{
    x = fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0)
        x += 2.0 * M_PI;
    return x - M_PI;
}

inline static double angularNorm(double diff)
{
    static const double M_2PI = 2.0 * M_PI;
    diff -= floor(diff/M_2PI + .5)*M_2PI;
    return diff;
}

inline static double euclideanDistance(geometry_msgs::Point const & p0, geometry_msgs::Point const & p1)
{
  return std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2) + pow(p1.z - p0.z, 2));
}

inline static double euclideanDistance2D(geometry_msgs::Point const & p0, geometry_msgs::Point const & p1)
{
  return std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2));
}


inline static Eigen::Quaterniond geomQuat2EigenQuat(geometry_msgs::Quaternion const & quat)
{
    return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

inline static geometry_msgs::PoseStamped createPoseFromQuatAndPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation){
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = position(0);
  pose.pose.position.y = position(1);
  pose.pose.position.z = position(2);
  pose.pose.orientation.w = orientation.w();
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  return pose;
}

#endif // UTILITY_H
