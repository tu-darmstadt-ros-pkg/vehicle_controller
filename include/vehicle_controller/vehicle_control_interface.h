/*******************************************************************************
 * Copyright (c) 2016, Stefan Kohlbrecher, Paul Manns
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

#ifndef VEHICLE_CONTROL_INTERFACE_H
#define VEHICLE_CONTROL_INTERFACE_H

#include <vehicle_controller/motion_parameters.h>
#include <vehicle_controller/robot_control_state.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class VehicleControlInterface
{
  public:
    virtual void configure(ros::NodeHandle& pnh, MotionParameters* mp) = 0;
    virtual void executeTwist(const geometry_msgs::Twist& twist) = 0;
    virtual void executeUnlimitedTwist(const geometry_msgs::Twist& twist) = 0;

    virtual void executeMotionCommand(RobotControlState rcs) = 0;

    virtual void stop() = 0;
    virtual double getCommandedSpeed() const = 0;
    virtual std::string getName() = 0;

    virtual bool hasReachedFinalOrientation(double goal_angle_error, double tol) = 0;

};

#endif
