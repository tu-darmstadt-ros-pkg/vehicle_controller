#ifndef ROBOT_CONTROL_STATE_H
#define ROBOT_CONTROL_STATE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

struct RobotControlState
{
    // Current robot state variables
    geometry_msgs::Vector3 velocity_linear;
    geometry_msgs::Vector3 velocity_angular;
    geometry_msgs::Pose    pose;
    double                 dt;

    inline void setRobotState(const geometry_msgs::Vector3 &velocity_linear,
                              const geometry_msgs::Vector3 &velocity_angular,
                              const geometry_msgs::Pose &pose,
                              const double dt);


    // Current control state variables
    double                 desired_velocity_linear;
    geometry_msgs::Vector3 desired_position;

    double                 error_2_path_angular;
    double                 error_2_carrot_angular;
    double                 carrot_distance;
    double                 signed_carrot_distance_2_robot;
    bool                   approaching_goal_point;

    inline void clearControlState();
    inline void setControlState(double desired_velocity_linear,
                                geometry_msgs::Vector3 desired_position,
                                double error_2_path_angular,
                                double error_2_carrot_angular,
                                double carrot_distance,
                                double signed_carrot_distance_2_robot,
                                bool approaching_goal_point);
};

void RobotControlState::setRobotState(geometry_msgs::Vector3 const & velocity_linear,
                                      geometry_msgs::Vector3 const & velocity_angular,
                                      geometry_msgs::Pose const & pose,
                                      double const dt)
{
    this->velocity_linear  = velocity_linear;
    this->velocity_angular = velocity_angular;
    this->pose             = pose;
    this->dt               = dt;
}

inline void RobotControlState::setControlState(double desired_velocity_linear,
                            geometry_msgs::Vector3 desired_position,
                            double error_2_path_angular,
                            double error_2_carrot_angular,
                            double carrot_distance,
                            double signed_carrot_distance_2_robot,
                            bool approaching_goal_point)
{
    this->desired_velocity_linear = desired_velocity_linear;
    this->desired_position        = desired_position;
    this->error_2_path_angular    = error_2_path_angular;
    this->error_2_carrot_angular  = error_2_carrot_angular;
    this->carrot_distance         = carrot_distance;
    this->signed_carrot_distance_2_robot = signed_carrot_distance_2_robot;
    this->approaching_goal_point  = approaching_goal_point;
}

void RobotControlState::clearControlState()
{
    desired_velocity_linear = 0.0;
    desired_position        = geometry_msgs::Vector3();
    error_2_path_angular    = 0.0;
    error_2_carrot_angular  = 0.0;
    carrot_distance         = 0.0;
    signed_carrot_distance_2_robot = 0.0;
    approaching_goal_point  = false;
}

#endif // ROBOT_CONTROL_STATE_H
