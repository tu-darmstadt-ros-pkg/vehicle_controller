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

#include <vehicle_controller/ekf.h>

class DifferentialDriveController: public VehicleControlInterface
{
  public:
    DifferentialDriveController();

    void configure(ros::NodeHandle& params, MotionParameters* mp) override;

    inline bool hasReachedFinalOrientation(double goal_angle_error, double tol, bool reverse_allowed) override
    {
        if (reverse_allowed)
        {
            return std::abs(goal_angle_error) < tol || std::abs(goal_angle_error - M_PI) < tol || std::abs(goal_angle_error + M_PI) < tol;
        }
        else
        {
            return std::abs(goal_angle_error) < tol;
        }
    }

    void executeUnlimitedTwist(const geometry_msgs::Twist& inc_twist) override;

    void executeTwist(const geometry_msgs::Twist& inc_twist) override;
    void executeTwist(const geometry_msgs::Twist& inc_twist, RobotControlState rcs, double yaw, double pitch, double roll) override;

    /**
     * @brief DifferentialDriveController::executePDControlledMotionCommand
     * @param e_angle the angular error which is assumed to lie inside [-pi,pi]
     * @param e_position the position error
     * @param dt time difference between two control loop iterates
     */
    void executePDControlledMotionCommand(double e_angle, double e_position, double dt, double cmded_speed, bool approaching_goal_point);

    void executeMotionCommand(RobotControlState rcs) override;

    virtual void executeMotionCommandSimple(RobotControlState rcs);

    void stop() override;

    inline double getCommandedSpeed() const override
    {
      return twist_.linear.x;
    }

    inline double getCommandedRotationalRate() const override
    {
      return twist_.angular.z;
    }

    inline std::string getName() override
    {
      return "Differential Drive Controller";
    }

  protected:
    ros::NodeHandle nh;
    ros::NodeHandle nh_dr_pdparams;
    ros::Publisher cmd_vel_raw_pub_;
    ros::Publisher pdout_pub_;

    geometry_msgs::Twist twist_;
    MotionParameters* mp_;

    ros::Time ekf_lastTime;
    geometry_msgs::Twist ekf_lastCmd;
    EKF ekf;
    bool ekf_setInitialPose = false;
    bool ekf_useEkf = false;
    double ekf_last_yaw, ekf_last_roll, ekf_last_pitch;
    double wheel_separation;

    template <typename TPD> void pdParamCallback(TPD & config, uint32_t level)
    {
        KP_ANGLE_ = config.angle_p_gain;
        KD_ANGLE_ = config.angle_d_gain;
        KP_POSITION_ = config.position_p_gain;
        KD_POSITION_ = config.position_d_gain;
        mp_->commanded_speed = config.speed;
        SPEED_REDUCTION_GAIN_ = config.speed_reduction_gain;
        mp_->speed_p_gain = config.speed_p_gain;
        mp_->use_final_twist = config.use_final_twist;
        mp_->final_twist_trials_max = config.final_twist_trials_max;

        ekf_useEkf = config.use_ekf;
        if(!ekf_useEkf){
          ekf_setInitialPose = false;
        }
        if(config.use_affw){
          cmd_vel_raw_pub_         = nh.advertise<geometry_msgs::Twist>("/affw_ctrl/target_vel", 1, true);
        }
        else{
          cmd_vel_raw_pub_         = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1, true);
        }
    }

    void limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate, bool keep_curvature = false) const;

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
    std::shared_ptr<dynamic_reconfigure::Server<vehicle_controller::PdParamsConfig>> dr_default_server_;
};

#endif
