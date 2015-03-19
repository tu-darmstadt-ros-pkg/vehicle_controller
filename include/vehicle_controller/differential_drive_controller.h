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

#ifndef DIFFERENTIAL_DRIVE_CONTROLLER_H
#define DIFFERENTIAL_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "vehicle_control_interface.h"

class DifferentialDriveController: public VehicleControlInterface
{
  public:
    virtual void configure(ros::NodeHandle& params, MotionParameters* mp)
    {
      mp_ = mp;

      ros::NodeHandle nh;
      cmdVelRawPublisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);

      // Get max speed, to calc max angular rate
      params.getParam("max_speed", max_speed_);

      params.getParam("wheel_separation", wheel_separation_);

      max_angular_rate_ = max_speed_ / (wheel_separation_ * 0.5);
    }

    virtual void executeTwist(const geometry_msgs::Twist& inc_twist)
    {
      twist = inc_twist;
      this->limitTwist(twist);
      cmdVelRawPublisher_.publish(twist);
    }

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error, double carrot_distance, double speed)
    {
      float sign = speed < 0.0 ? -1.0 : 1.0;

      twist.linear.x = speed;

      if (sign < 0){
        twist.angular.z = carrot_orientation_error / carrot_distance * 1.5 * 0.25;
      }else{
        twist.angular.z = carrot_relative_angle / carrot_distance * 1.5;
      }

      this->limitTwist(twist);

      cmdVelRawPublisher_.publish(twist);
    }

    virtual void stop()
    {
      twist.angular.z = 0.0;
      twist.linear.x = 0.0;
      cmdVelRawPublisher_.publish(twist);
    }

    virtual double getCommandedSpeed() const
    {
      return twist.linear.x;
    }

    virtual std::string getName()
    {
      return "Differential Drive Controller";
    }

    void limitTwist(geometry_msgs::Twist& twist)
    {
      float speed = twist.linear.x;

      mp_->limitSpeed(speed);

      double angular_rate = twist.angular.z;

      //Limit angular rate to be achievable
      if (angular_rate > max_angular_rate_){
        angular_rate = max_angular_rate_;
      }else if (angular_rate < -max_angular_rate_){
        angular_rate = -max_angular_rate_;
      }

      //Calculate the speed reduction factor that we need to apply to be able to achieve desired angular rate.
      double speed_reduction_factor = (max_speed_ - fabs(angular_rate) * (wheel_separation_ * 0.5)) / max_speed_;

      if (fabs(speed) > speed_reduction_factor * max_speed_){
        speed = speed * speed_reduction_factor;
      }

      twist.linear.x = speed;
      twist.angular.z = angular_rate;
    }

  protected:
    ros::Publisher cmdVelRawPublisher_;

    geometry_msgs::Twist twist;

    MotionParameters* mp_;

    double max_speed_;
    double max_angular_rate_;
    double wheel_separation_;
};

#endif
