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
#include "vehicle_control_interface.h"

class FourWheelSteerController: public VehicleControlInterface
{
  public:
    virtual void configure(ros::NodeHandle& params, MotionParameters* mp)
    {
      mp_ = mp;
      mp_->USE_FINAL_TWIST_ = false;

      ros::NodeHandle nh;
      drivePublisher_ = nh.advertise<monstertruck_msgs::MotionCommand>("drive", 1);

      max_steeringangle = 30.0 * M_PI/180.0;
      params.getParam("max_steeringangle", max_steeringangle);
    }

    virtual void executeUnlimitedTwist(const geometry_msgs::Twist& velocity)
    {
        executeTwist(velocity);
    }

    void limitSpeed(double & speed)
    {
        double inclination_max_speed = std::max(fabs(speed) * (1.0 - mp_->current_inclination * mp_->inclination_speed_reduction_factor), 0.0);
        if (speed > 0.0) {
          if (speed > mp_->max_controller_speed_) speed = mp_->max_controller_speed_;
          if (speed > inclination_max_speed) speed = inclination_max_speed;
          if (speed < mp_->min_speed) speed = mp_->min_speed;
        } else if (speed < 0.0) {
          if (speed < -mp_->max_controller_speed_) speed = -mp_->max_controller_speed_;
          if (speed < -inclination_max_speed) speed = -inclination_max_speed;
          if (speed > -mp_->min_speed) speed = -mp_->min_speed;
        }
    }

    virtual void executeTwist(const geometry_msgs::Twist& velocity)
    {
      double backward = (velocity.linear.x < 0) ? -1.0 : 1.0;
      double speed = backward * sqrt(velocity.linear.x*velocity.linear.x + velocity.linear.y*velocity.linear.y);

      limitSpeed(speed);

      float kappa = velocity.angular.z * speed;
      float tan_gamma = tan(velocity.linear.y / velocity.linear.x);

      setDriveCommand(speed, kappa, tan_gamma);      
    }

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error, double carrot_distance, double speed)
    {
      float sign = speed < 0.0 ? -1.0 : 1.0;
      float kappa     = sign * carrot_orientation_error / carrot_distance * 1.5;
      float tan_gamma = tan(carrot_relative_angle - carrot_orientation_error);

      this->setDriveCommand(speed, kappa ,tan_gamma);
    }

    virtual void stop()
    {
      drive.speed = 0.0;
      drivePublisher_.publish(drive);
    }

    virtual double getCommandedSpeed() const
    {
      return drive.speed;
    }

    virtual std::string getName()
    {
      return "Four Wheel Steering Controller";
    }

    void setDriveCommand(double speed, double kappa, double tan_gamma) {

      float B = 0.16; // half wheel distance (front - rear)

      limitSpeed(speed);
      drive.speed = speed;

      if (drive.speed != 0.0) {
        float max_kappa = tan(max_steeringangle) / B;
        if (kappa >= max_kappa) {
          kappa = max_kappa;
          tan_gamma = 0;

        } else if (kappa <= -max_kappa) {
          kappa = -max_kappa;
          tan_gamma = 0;

        } else {
          float max_tan_gamma = tan(max_steeringangle) - fabs(kappa) * B;
          if (tan_gamma >  max_tan_gamma) tan_gamma =  max_tan_gamma;
          if (tan_gamma < -max_tan_gamma) tan_gamma = -max_tan_gamma;
        }

        drive.steerAngleFront = atan( tan_gamma + kappa * B);
        drive.steerAngleRear  = atan(-tan_gamma + kappa * B);
      }
      drivePublisher_.publish(drive);
    }

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
