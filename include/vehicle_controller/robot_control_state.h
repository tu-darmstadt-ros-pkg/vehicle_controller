/*******************************************************************************
 * Copyright (c) 2016, Paul Manns
 *
 * All rights reserved.
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef ROBOT_CONTROL_STATE_H
#define ROBOT_CONTROL_STATE_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

struct RobotControlState
{
    // Current robot state variables
    geometry_msgs::Point   velocity_linear;
    geometry_msgs::Point   velocity_angular;
    geometry_msgs::Pose    pose;
    double                 dt;

    inline void setRobotState(const geometry_msgs::Point &velocity_linear,
                              const geometry_msgs::Point &velocity_angular,
                              const geometry_msgs::Pose &pose,
                              const double dt);


    // Current control state variables
    double                 desired_velocity_linear;
    geometry_msgs::Point   desired_position;

    double                 error_2_path_angular;
    double                 error_2_carrot_angular;
    double                 carrot_distance;
    double                 signed_carrot_distance_2_robot;
    bool                   approaching_goal_point;

    inline void clearControlState();
    inline void setControlState(double desired_velocity_linear,
                                geometry_msgs::Point desired_position,
                                double error_2_path_angular,
                                double error_2_carrot_angular,
                                double carrot_distance,
                                double signed_carrot_distance_2_robot,
                                bool approaching_goal_point);
};

void RobotControlState::setRobotState(geometry_msgs::Point const & velocity_linear,
                                      geometry_msgs::Point const & velocity_angular,
                                      geometry_msgs::Pose const & pose,
                                      double const dt)
{
    this->velocity_linear  = velocity_linear;
    this->velocity_angular = velocity_angular;
    this->pose             = pose;
    this->dt               = dt;
}

inline void RobotControlState::setControlState(double desired_velocity_linear,
                            geometry_msgs::Point desired_position,
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
    desired_position        = geometry_msgs::Point();
    error_2_path_angular    = 0.0;
    error_2_carrot_angular  = 0.0;
    carrot_distance         = 0.0;
    signed_carrot_distance_2_robot = 0.0;
    approaching_goal_point  = false;
}

#endif // ROBOT_CONTROL_STATE_H
