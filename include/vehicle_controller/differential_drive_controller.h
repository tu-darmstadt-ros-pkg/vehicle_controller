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
#include <monstertruck_msgs/Pdout.h>
#include "vehicle_control_interface.h"
#include <algorithm>
#include <fstream>
#include <queue>

#include <dynamic_reconfigure/server.h>
#include <vehicle_controller/PdParamsConfig.h>


class DifferentialDriveController: public VehicleControlInterface
{
  public:
    virtual ~DifferentialDriveController();

    virtual void configure(ros::NodeHandle& params, MotionParameters* mp);

    void pdGainCallback(vehicle_controller::PdParamsConfig & config, uint32_t level);

    virtual void executeUnlimitedTwist(const geometry_msgs::Twist& inc_twist);

    virtual void executeTwist(const geometry_msgs::Twist& inc_twist);

    void executePDControlledMotionCommand(double e_angle, double e_position, double dt, double cmded_speed);

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error,
                                      double carrot_distance, double speed,
                                      double signed_carrot_distance_2_robot, double dt);

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error, double carrot_distance, double speed);

    virtual void stop();

    inline virtual double getCommandedSpeed() const
    {
      return twist.linear.x;
    }

    inline virtual std::string getName()
    {
      return "Differential Drive Controller";
    }

    void limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate);

  protected:
    ros::Publisher cmdVelRawPublisher_;
    ros::Publisher pdoutPublisher_;
    geometry_msgs::Twist twist;
    MotionParameters* mp_;

  private:

    double wheel_separation_;



    dynamic_reconfigure::Server<vehicle_controller::PdParamsConfig> * dr_server_;
};

#endif
