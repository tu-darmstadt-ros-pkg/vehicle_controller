/*
    Copyright (c) 2015, Paul Manns, Stefan Kohlbrecher
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

#include <vehicle_controller/vehicle_control_interface.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <monstertruck_msgs/Pdout.h>
#include <algorithm>
#include <fstream>
#include <queue>

#include <dynamic_reconfigure/server.h>
#include <vehicle_controller/PdParamsConfig.h>
#include <vehicle_controller/PdParamsArgoConfig.h>


class DifferentialDriveController: public VehicleControlInterface
{
  public:
    virtual ~DifferentialDriveController();

    virtual void configure(ros::NodeHandle& params, MotionParameters* mp);

    inline virtual bool hasReachedFinalOrientation(double goal_angle_error, double tol)
    {
        if(mp_->isYSymmetric())
        {
            return std::abs(goal_angle_error) < tol || std::abs(goal_angle_error - M_PI) < tol || std::abs(goal_angle_error + M_PI) < tol;
        }
        else
        {
            return std::abs(goal_angle_error) < tol;
        }
    }

    virtual void executeUnlimitedTwist(const geometry_msgs::Twist& inc_twist);

    virtual void executeTwist(const geometry_msgs::Twist& inc_twist);

    /**
     * @brief DifferentialDriveController::executePDControlledMotionCommand
     * @param e_angle the angular error which is assumed to lie inside [-pi,pi]
     * @param e_position the position error
     * @param dt time difference between two control loop iterates
     */
    void executePDControlledMotionCommand(double e_angle, double e_position, double dt, double cmded_speed, bool approaching_goal_point);

    virtual void executeMotionCommand(RobotControlState rcs);

    virtual void executeMotionCommandSimple(RobotControlState rcs);

    virtual void stop();

    inline virtual double getCommandedSpeed() const
    {
      return twist.linear.x;
    }

    inline virtual std::string getName()
    {
      return "Differential Drive Controller";
    }

  protected:
    ros::Publisher cmd_vel_raw_pub_;
    ros::Publisher pdout_pub_;

    geometry_msgs::Twist twist;
    MotionParameters* mp_;

    template <typename TPD> void pdParamCallback(TPD & config, uint32_t level)
    {
        KP_ANGLE_ = config.angle_p_gain;
        KD_ANGLE_ = config.angle_d_gain;
        KP_POSITION_ = config.position_p_gain;
        KD_POSITION_ = config.position_d_gain;
        mp_->commanded_speed = config.speed;
        SPEED_REDUCTION_GAIN_ = config.speed_reduction_gain;
        mp_->USE_FINAL_TWIST_ = config.use_final_twist;
        mp_->FINAL_TWIST_TRIALS_MAX_ = config.final_twist_trials_max;
        mp_->flipper_low_position = config.flipper_low_position;
        mp_->flipper_high_position = config.flipper_high_position;
        mp_->flipper_switch_position = config.flipper_switch_position;
    }

    void limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate) const;

  private:

    double KP_ANGLE_;
    double KD_ANGLE_;
    double KP_POSITION_;
    double KD_POSITION_;
    double SPEED_REDUCTION_GAIN_;

    //
    // TODO
    // This is a weird design error. The dr_server_ belongs to the MotionParameters object
    // or the controller itself but definitely not in this class
    // This has to be fixed!!!
    //
    dynamic_reconfigure::Server<vehicle_controller::PdParamsConfig>     * dr_default_server_ = 0;
    dynamic_reconfigure::Server<vehicle_controller::PdParamsArgoConfig> * dr_argo_server_ = 0;
};

#endif
