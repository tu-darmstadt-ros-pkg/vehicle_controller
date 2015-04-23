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
    virtual ~DifferentialDriveController()
    {
        if(dr_server_)
            delete dr_server_;
    }

    virtual void configure(ros::NodeHandle& params, MotionParameters* mp)
    {
        mp_ = mp;

        ros::NodeHandle nh;
        cmdVelRawPublisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);
        pdoutPublisher_ = nh.advertise<monstertruck_msgs::Pdout>("pdout", 1);

        // Get max speed, to calc max angular rate
        params.getParam("max_controller_speed", mp_->max_controller_speed_);
        params.getParam("max_unlimited_speed", mp_->max_unlimited_speed_);

        params.getParam("wheel_separation", wheel_separation_);

        params.getParam("max_controller_angular_rate", mp_->max_controller_angular_rate_);
        params.getParam("max_unlimited_angular_rate", mp_->max_unlimited_angular_rate_);
        // alternative to a fixed param would be the physically possible rate:
        //       max_angular_rate_ = max_speed_ / (wheel_separation_ * 0.5);
        // However, this turned out to be way to fast for the robot on the
        // oil site platform.

        KP_ANGLE_ = 2.0;
        KD_ANGLE_ = 0.5;
        KP_POSITION_ = 0.5;
        KD_POSITION_ = 0.0;

        dr_server_ = new dynamic_reconfigure::Server<vehicle_controller::PdParamsConfig>;
        dr_server_->setCallback(boost::bind(&DifferentialDriveController::pdGainCallback, this, _1, _2));
    }

    void pdGainCallback(vehicle_controller::PdParamsConfig & config, uint32_t level)
    {
        ROS_INFO("Reconfiguring vehicle controller.");
        KP_ANGLE_ = config.angle_p_gain;
        KD_ANGLE_ = config.angle_d_gain;
        KP_POSITION_ = config.position_p_gain;
        KD_POSITION_ = config.position_d_gain;
    }

    virtual void executeUnlimitedTwist(const geometry_msgs::Twist& inc_twist)
    {
        twist = inc_twist;
        this->limitTwist(twist, mp_->max_unlimited_speed_, mp_->max_unlimited_angular_rate_);
        cmdVelRawPublisher_.publish(twist);
    }

    virtual void executeTwist(const geometry_msgs::Twist& inc_twist)
    {
      twist = inc_twist;
      this->limitTwist(twist, mp_->max_controller_speed_, mp_->max_controller_angular_rate_);
      cmdVelRawPublisher_.publish(twist);
    }


    void executePDControlledMotionCommand(double e_angle, double e_position, double dt)
    {
        static double previous_e_angle = e_angle;
        static double previous_e_position = e_position;

        double de_angle_dt    = (e_angle - previous_e_angle) / dt; //? uncontinuity @ orientation_error vs relative_angle switch
        double de_position_dt = (e_position - previous_e_position) / dt;

        double speed = KP_POSITION_ * e_position + KD_POSITION_ * de_position_dt;
        double z_twist = KP_ANGLE_ * e_angle + KD_ANGLE_ * de_angle_dt;

        twist.linear.x = speed;
        twist.angular.z = z_twist;

        if(fabs(e_angle) > M_PI_4 && e_position < 0.0)
        {
            twist.linear.x = 0.0;
            // twist.angular.z = -twist.angular.z;
        }

        this->limitTwist(twist, mp_->max_controller_speed_, mp_->max_controller_angular_rate_);
        cmdVelRawPublisher_.publish(twist);

//        ROS_INFO("[PD INFO] e = (%f %f), c = (%f %f), cl = (%f %f)",
//                 e_position, e_angle / M_PI * 180, speed, z_twist / M_PI * 180,
//                 twist.linear.x, twist.angular.z / M_PI * 180);

//        std::fstream fs;
//        fs.open ("pd_tracker.csv", std::fstream::out | std::fstream::app);
//        fs << dt << "," << e_position << "," <<  de_position_dt << ","
//           << e_angle << "," << de_angle_dt << ","
//           << speed << "," << twist.linear.x << ","
//           << z_twist / M_PI * 180 << "," << twist.angular.z / M_PI * 180
//           << std::endl;

        monstertruck_msgs::Pdout pdout;
        pdout.dt = dt;
        pdout.e_position = e_position;
        pdout.e_angle = e_angle;
        pdout.de_position_dt = de_position_dt;
        pdout.de_angle_dt = de_angle_dt;
        pdout.speed = speed;
        pdout.z_twist_deg = z_twist / M_PI * 180;
        pdout.speed_real = twist.linear.x;
        pdout.z_twist_deg_real = twist.angular.z / M_PI * 180;
        pdoutPublisher_.publish(pdout);

        previous_e_angle = e_angle;
        previous_e_position = e_position;
    }

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error,
                                      double carrot_distance, double speed,
                                      double signed_carrot_distance_2_robot, double dt)
    {
        double e_angle = speed < 0 ? carrot_orientation_error : carrot_relative_angle;
        // ROS_INFO("s -e e = %f %f %f", speed, carrot_orientation_error, carrot_relative_angle);

        // executePDControlledMotionCommand()
        if(signed_carrot_distance_2_robot < 0 && fabs(e_angle) > M_PI_4)
            e_angle = carrot_relative_angle;

        executePDControlledMotionCommand(e_angle, signed_carrot_distance_2_robot, dt);
        // executeMotionCommand(carrot_relative_angle, carrot_orientation_error, carrot_distance, speed);
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

      this->limitTwist(twist, mp_->max_controller_speed_, mp_->max_controller_angular_rate_);
      ROS_INFO("[MC INFO] cl = (%f %f)", twist.linear.x, twist.angular.z / M_PI * 180);

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

    void limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate)
    {
      float speed = twist.linear.x;

      mp_->limitSpeed(speed);

      double angular_rate = twist.angular.z;

      //Limit angular rate to be achievable
      angular_rate = std::max(-max_angular_rate, std::min(max_angular_rate, angular_rate));

      //Calculate the speed reduction factor that we need to apply to be able to achieve desired angular rate.
      double speed_reduction_factor = (max_speed - fabs(angular_rate) * (wheel_separation_ * 0.5)) / max_speed;

      if (fabs(speed) > fabs(speed_reduction_factor) * max_speed)
      {
        speed = speed * fabs(speed_reduction_factor);
      }

      twist.linear.x = speed;
      twist.angular.z = angular_rate;
    }

  protected:
    ros::Publisher cmdVelRawPublisher_;
    ros::Publisher pdoutPublisher_;


    geometry_msgs::Twist twist;

    MotionParameters* mp_;

  private:
    double KP_ANGLE_ = 2.0;
    double KD_ANGLE_ = 0.50;
    double KP_POSITION_ = 0.5;
    double KD_POSITION_ = 0.0; //0.5;
    double wheel_separation_;

    dynamic_reconfigure::Server<vehicle_controller::PdParamsConfig> * dr_server_;
};

#endif
