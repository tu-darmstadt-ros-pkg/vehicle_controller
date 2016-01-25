/*
    Copyright (c) 2014, Stefan Kohlbrecher
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef FOUR_WHEEL_STEER_CONTROLLER_H
#define FOUR_WHEEL_STEER_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <monstertruck_msgs/MotionCommand.h>

#include "vehicle_control_interface.h"

class FourWheelSteerController: public VehicleControlInterface
{
  public:
    virtual void configure(ros::NodeHandle& params, MotionParameters* mp);

    inline virtual void executeUnlimitedTwist(const geometry_msgs::Twist& velocity)
    {
        executeTwist(velocity);
    }

    void limitSpeed(double & speed);

    virtual void executeTwist(const geometry_msgs::Twist& velocity);

    virtual void executeMotionCommand(RobotControlState rcs);

    virtual void stop();

    virtual double getCommandedSpeed() const;

    inline virtual std::string getName()
    {
      return "Four Wheel Steering Controller";
    }

    void setDriveCommand(double speed, double kappa, double tan_gamma);

    inline virtual bool hasReachedFinalOrientation(double goal_angle_error, double tol)
    {
        /// for nonsymmetric 4 wheel robot!
        return std::abs(goal_angle_error) < tol;
    }

  protected:
    ros::Publisher drivePublisher_;

    monstertruck_msgs::MotionCommand drive;

    MotionParameters* mp_;

    double max_steeringangle;
};

#endif
