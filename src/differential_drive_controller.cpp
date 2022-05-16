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
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
    THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <vehicle_controller/differential_drive_controller.h>

DifferentialDriveController::DifferentialDriveController():
  nh_dr_pdparams("~/pd_params")
{
}

void DifferentialDriveController::configure(ros::NodeHandle& params, MotionParameters* mp)
{
    mp_ = mp;

    //ros::NodeHandle nh;
    cmd_vel_raw_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);
    pdout_pub_       = nh.advertise<monstertruck_msgs::Pdout>("pdout", 1);

    params.getParam("max_controller_speed", mp_->max_controller_speed);
    params.getParam("max_unlimited_speed", mp_->max_unlimited_speed);
    params.getParam("max_controller_angular_rate", mp_->max_controller_angular_rate);
    params.getParam("max_unlimited_angular_rate", mp_->max_unlimited_angular_rate);
    params.getParam("wheel_separation", wheel_separation);


    KP_ANGLE_    = 2.0;
    KD_ANGLE_    = 0.5;
    KP_POSITION_ = 0.5;
    KD_POSITION_ = 0.0;
    SPEED_REDUCTION_GAIN_ = 2.0;


    dr_default_server_ = std::make_shared<dynamic_reconfigure::Server<vehicle_controller::PdParamsConfig>>(nh_dr_pdparams);
    dr_default_server_->setCallback(
          boost::bind(&DifferentialDriveController::pdParamCallback<vehicle_controller::PdParamsConfig>, this, _1, _2));

}

void DifferentialDriveController::executeUnlimitedTwist(const geometry_msgs::Twist& inc_twist)
{
    twist_ = inc_twist;
    twist_.angular.z = std::max(-mp_->max_unlimited_angular_rate,
                                std::min(mp_->max_unlimited_angular_rate, twist_.angular.z));
    twist_.linear.x  = std::max(-mp_->max_unlimited_speed,
                                std::min(mp_->max_unlimited_speed, twist_.linear.x));
    cmd_vel_raw_pub_.publish(twist_);
}

void DifferentialDriveController::executeTwist(const geometry_msgs::Twist& inc_twist)
{
    twist_ = inc_twist;
    this->limitTwist(twist_, mp_->max_controller_speed, mp_->max_controller_angular_rate, true);
    cmd_vel_raw_pub_.publish(twist_);
}


void DifferentialDriveController::executeTwist(const geometry_msgs::Twist& inc_twist, RobotControlState rcs, double yaw ,double pitch, double roll)
{
    twist_ = inc_twist;
    this->limitTwist(twist_, mp_->max_controller_speed, mp_->max_controller_angular_rate, true);

    //ROS_INFO("limited: lin: %f, ang: %f", twist_.linear.x, twist_.angular.z);

    if(ekf_useEkf){
      if (!ekf_setInitialPose){
        ekf.x_(0,0) = rcs.pose.position.x;
        ekf.x_(1,0) = rcs.pose.position.y;
        ekf.x_(2,0) = yaw;

        ekf_setInitialPose = true;
        ekf_lastTime = ros::Time::now();
        ekf_lastCmd = twist_;

        ekf_last_pitch = pitch;
        ekf_last_roll = roll;
        ekf_last_yaw = yaw;

        cmd_vel_raw_pub_.publish(twist_);
      }
      else{
        double dt = (ros::Time::now().toSec() - ekf_lastTime.toSec());

        double v_lin = ekf_lastCmd.linear.x;
        double v_ang = ekf_lastCmd.angular.z;

        double Vl_ = v_lin - wheel_separation/2 * v_ang;
        double Vr_ = v_lin + wheel_separation/2 * v_ang;

        if(dt > 0){
          ekf.predict(Vl_, Vr_, ekf_last_pitch, ekf_last_roll, dt);

          Eigen::Vector3d delta;
          delta(0) = rcs.pose.position.x;
          delta(1) = rcs.pose.position.y;
          delta(2) = yaw;
          ekf.correct(delta);

          double omega = -(Vl_ - Vr_)/fabs(ekf.x_(4) - ekf.x_(3))  * std::cos(ekf_last_roll) * std::cos(ekf_last_pitch);
          //ROS_INFO("omega: %f, twist_: %f, pose.twist_: %f, yaw_diff: %f", omega, cmd.angular.z, robot_control_state.velocity_angular.z, (yaw-ekf_last_yaw)/dt);

          double y_ICRr = ekf.x_(3,0);
          double y_ICRl = ekf.x_(4,0);

          double vl_corrected = twist_.linear.x - y_ICRl * twist_.angular.z;
          double vr_corrected = twist_.linear.x - y_ICRr * twist_.angular.z;

//          ROS_INFO("vl: %f, vl_corrected: %f", Vl_, vl_corrected);
//          ROS_INFO("vr: %f, vr_corrected: %f", Vr_, vr_corrected);
//          ROS_INFO("vlin: %f, vlin_corrected: %f", twist_.linear.x, (vl_corrected + vr_corrected)/2);

          twist_.linear.x = (vl_corrected + vr_corrected) / 2;
          twist_.angular.z = (vr_corrected - vl_corrected) / wheel_separation;

          cmd_vel_raw_pub_.publish(twist_);

          //ROS_INFO("yl: %f, yr: %f, x: %f",ekf.x_(4), ekf.x_(3), ekf.x_(5) );

          double icr = (ekf.x_(4) + ekf.x_(3));
          //ROS_INFO("ICR: %f", icr);

          ekf_lastCmd = twist_;
          ekf_lastTime = ros::Time::now();
          ekf_last_pitch = pitch;
          ekf_last_roll = roll;
          ekf_last_yaw = yaw;
        }

      }
    }
    else{
      cmd_vel_raw_pub_.publish(twist_);
    }


}

void DifferentialDriveController::executePDControlledMotionCommand(double e_angle,
    double e_position, double dt, double cmded_speed, bool approaching_goal_point)
{
    static double previous_e_angle = e_angle;
    static double previous_e_position = e_position;

    double de_angle_dt    = (e_angle - previous_e_angle) / dt; // causes discontinuity @ orientation_error vs relative_angle switch
    double de_position_dt = (e_position - previous_e_position) / dt;
    if (dt <= 0.0) // TODO: Maybe replace by dt < epsilon for some small epsilon
    {
        ROS_WARN("[vehicle_controller] [differential_drive_controller] dt between two measurements is 0, ignoring D-part of control law.");
        de_angle_dt = 0.0;
        de_position_dt = 0.0;
    }


    double speed   = approaching_goal_point ? std::abs(cmded_speed) * (e_position < 0 ? -1.0 : 1.0) :
                                              KP_POSITION_ * e_position + KD_POSITION_ * de_position_dt;
    double z_angular_rate = KP_ANGLE_ * e_angle + KD_ANGLE_ * de_angle_dt;

    if(fabs(speed) > fabs(cmded_speed))
    {
        speed = (speed < 0 ? -1.0 : 1.0) * fabs(cmded_speed);
    }

    twist_.linear.x = speed;
    twist_.angular.z = z_angular_rate;
    this->limitTwist(twist_, mp_->max_controller_speed, mp_->max_controller_angular_rate, false);
    cmd_vel_raw_pub_.publish(twist_);

    if (pdout_pub_.getNumSubscribers() > 0)
    {
      monstertruck_msgs::Pdout pdout;
      pdout.header.frame_id = "world";
      pdout.header.stamp = ros::Time::now();
      pdout.approaching_goal_point = approaching_goal_point;
      pdout.dt = dt;
      pdout.e_position = e_position;
      pdout.e_angle = e_angle;
      pdout.de_position_dt = de_position_dt;
      pdout.de_angle_dt = de_angle_dt;
      pdout.speed = speed;
      pdout.z_twist = z_angular_rate;
      pdout.z_twist_real = twist_.angular.z;
      pdout.z_twist_deg = z_angular_rate / M_PI * 180;
      pdout.speed_real = twist_.linear.x;
      pdout.z_twist_deg_real = twist_.angular.z / M_PI * 180;
      pdout_pub_.publish(pdout);
    }

    previous_e_angle = e_angle;
    previous_e_position = e_position;
}

void DifferentialDriveController::executeMotionCommand(RobotControlState rcs)
{
    double e_angle = rcs.error_2_path_angular;
    if(e_angle > M_PI + 1e-2 || e_angle < -M_PI - 1e-2)
    {
        ROS_WARN("[vehicle_controller] [differential_drive_controller] Invalid angle was given.");
    }
    if(rcs.desired_velocity_linear == 0.0)
    {
        ROS_DEBUG("[vehicle_controller] [differential_drive_controller] Commanded speed is 0");
        rcs.desired_velocity_linear = 0.0;
    }
    executePDControlledMotionCommand(e_angle,
                                     rcs.signed_carrot_distance_2_robot,
                                     rcs.dt,
                                     rcs.desired_velocity_linear,
                                     rcs.approaching_goal_point);
}

void DifferentialDriveController::executeMotionCommandSimple(RobotControlState rcs)
{
    double sign = rcs.desired_velocity_linear < 0.0 ? -1.0 : 1.0;

    twist_.linear.x = rcs.desired_velocity_linear;
    if (sign < 0)
        twist_.angular.z = rcs.error_2_carrot_angular / rcs.carrot_distance * 1.5 * 0.25;
    else
        twist_.angular.z = rcs.error_2_path_angular / rcs.carrot_distance * 1.5;

    limitTwist(twist_, mp_->max_controller_speed, mp_->max_controller_angular_rate, false);

    cmd_vel_raw_pub_.publish(twist_);
}

void DifferentialDriveController::stop()
{
    twist_.angular.z = 0.0;
    twist_.linear.x  = 0.0;
    cmd_vel_raw_pub_.publish(twist_);
}

// Limits twist within track velocity limits. keep_curvature defines whether to keep the curvature or the angular velocity.
void DifferentialDriveController::limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate, bool keep_curvature) const
{
  if(keep_curvature) {
    double clamped_angular_velocity = std::clamp(twist.angular.z, -max_angular_rate, max_angular_rate);
    double clamp_ratio = std::abs(twist.angular.z) > 1E-7 ? std::abs(clamped_angular_velocity)/std::abs(twist.angular.z) : 1.0;
    double clamped_linear_velocity = clamp_ratio * twist.linear.x;
    double unlimited_velocity_right = clamped_linear_velocity + wheel_separation * clamped_angular_velocity / 2.0;
    double unlimited_velocity_left = clamped_linear_velocity - wheel_separation * clamped_angular_velocity / 2.0;
    double reduction_gain_right =  std::abs(unlimited_velocity_right) > 1E-7 ? std::min(mp_->max_unlimited_speed / std::abs(unlimited_velocity_right), 1.0) : 1.0;
    double reduction_gain_left = std::abs(unlimited_velocity_left) > 1E-7 ?  std::min(mp_->max_unlimited_speed / std::abs(unlimited_velocity_left), 1.0) : 1.0;
    double reduction_gain = std::min(reduction_gain_left, reduction_gain_right);    
    twist.linear.x = reduction_gain * clamped_linear_velocity;
    twist.angular.z = reduction_gain * clamped_angular_velocity;
  }
  else {
    // Optional Code to keep angular velocity component and use remaining velocity budget for linear component
    // double clamped_angular_velocity = std::clamp(twist.angular.z, -max_angular_rate, max_angular_rate);
    // double wheel_velocity_angular_component =  wheel_separation * clamped_angular_velocity / 2.0;
    // double upper_bound_right_wheel = mp_->max_unlimited_speed - wheel_velocity_angular_component;
    // double lower_bound_right_wheel = -mp_->max_unlimited_speed - wheel_velocity_angular_component;
    // double upper_bound_left_wheel = mp_->max_unlimited_speed + wheel_velocity_angular_component;
    // double lower_bound_left_wheel = -mp_->max_unlimited_speed + wheel_velocity_angular_component;
    // twist.linear.x = std::clamp(twist.linear.x, lower_bound_right_wheel, upper_bound_right_wheel);
    // twist.linear.x = std::clamp(twist.linear.x, lower_bound_left_wheel, upper_bound_left_wheel);
    // twist.angular.z = clamped_angular_velocity;    
    double speed = twist.linear.x;
    double angular_rate = twist.angular.z;
    speed        = std::max(-mp_->max_unlimited_speed, std::min(mp_->max_unlimited_speed, speed));
    angular_rate = std::max(-mp_->max_unlimited_angular_rate, std::min(mp_->max_unlimited_angular_rate, angular_rate));
    double m = -mp_->max_controller_speed / mp_->max_controller_angular_rate;
    double t = mp_->max_controller_speed;
    double speedAbsUL = std::min(std::max(0.0, m * std::abs(angular_rate) * SPEED_REDUCTION_GAIN_ + t), max_speed);
    twist.linear.x = std::max(-speedAbsUL, std::min(speed, speedAbsUL));
    twist.angular.z = std::max(-max_angular_rate, std::min(max_angular_rate, angular_rate));  
  }


}
