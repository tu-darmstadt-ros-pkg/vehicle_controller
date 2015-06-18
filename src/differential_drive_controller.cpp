//
// Created by paul on 30.04.15.
//

#include <vehicle_controller/differential_drive_controller.h>

DifferentialDriveController::~DifferentialDriveController()
{
    if(dr_server_)
        delete dr_server_;
}

void DifferentialDriveController::configure(ros::NodeHandle& params, MotionParameters* mp)
{
    mp_ = mp;

    ros::NodeHandle nh;
    cmdVelRawPublisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);
    pdoutPublisher_ = nh.advertise<monstertruck_msgs::Pdout>("pdout", 1);

    params.getParam("max_controller_speed", mp_->max_controller_speed_);
    params.getParam("max_unlimited_speed", mp_->max_unlimited_speed_);
    params.getParam("wheel_separation", wheel_separation_);
    params.getParam("max_controller_angular_rate", mp_->max_controller_angular_rate_);
    params.getParam("max_unlimited_angular_rate", mp_->max_unlimited_angular_rate_);

    mp_->KP_ANGLE_    = 2.0;
    mp_->KD_ANGLE_    = 0.5;
    mp_->KP_POSITION_ = 0.5;
    mp_->KD_POSITION_ = 0.0;
    mp_->DESIRED_SPEED_ = 0.075;
    mp_->USE_FINAL_TWIST_ = true;

    dr_server_ = new dynamic_reconfigure::Server<vehicle_controller::PdParamsConfig>;
    dr_server_->setCallback(boost::bind(&DifferentialDriveController::pdGainCallback, this, _1, _2));
}

void DifferentialDriveController::pdGainCallback(vehicle_controller::PdParamsConfig & config, uint32_t level)
{
    //
    // TODO
    // This is a weird design error. The dr_server_ belongs to the MotionParameters object
    // or the controller itself but definitely not in this class
    // This has to be fixed!!!
    //
    // PS: I am sorry for this, but it's 2h30 am and only one training day left for ARGOS challenge! (Paul)
    mp_->KP_ANGLE_ = config.angle_p_gain;
    mp_->KD_ANGLE_ = config.angle_d_gain;
    mp_->KP_POSITION_ = config.position_p_gain;
    mp_->KD_POSITION_ = config.position_d_gain;
    mp_->DESIRED_SPEED_ = config.speed;
    mp_->USE_FINAL_TWIST_ = config.use_final_twist;
}

void DifferentialDriveController::executeUnlimitedTwist(const geometry_msgs::Twist& inc_twist)
{
    twist = inc_twist;

    double speed = twist.linear.x;
    double angular_rate = twist.angular.z;

    angular_rate = std::max(-mp_->max_unlimited_angular_rate_, std::min<double>(mp_->max_unlimited_angular_rate_, angular_rate));
    speed        = std::max(-mp_->max_unlimited_speed_, std::min(speed, mp_->max_unlimited_speed_));

    twist.linear.x = speed;
    twist.angular.z = angular_rate;
    cmdVelRawPublisher_.publish(twist);
}

void DifferentialDriveController::executeTwist(const geometry_msgs::Twist& inc_twist)
{
    twist = inc_twist;
    this->limitTwist(twist, mp_->max_controller_speed_, mp_->max_controller_angular_rate_);
    cmdVelRawPublisher_.publish(twist);
}

void DifferentialDriveController::executePDControlledMotionCommand(double e_angle, double e_position, double dt, double cmded_speed)
{
    static double previous_e_angle = e_angle;
    static double previous_e_position = e_position;

    // Assumption: e_angle lies in [M_PI_2, M_PI]

    if(e_angle > M_PI_2)
        e_angle = e_angle - M_PI;
    if(e_angle < -M_PI_2)
        e_angle = M_PI + e_angle;

    double de_angle_dt    = (e_angle - previous_e_angle) / dt; // causes discontinuity @ orientation_error vs relative_angle switch
    double de_position_dt = (e_position - previous_e_position) / dt;

    double speed   = mp_->KP_POSITION_ * e_position + mp_->KD_POSITION_ * de_position_dt;
    double z_twist = mp_->KP_ANGLE_ * e_angle + mp_->KD_ANGLE_ * de_angle_dt;

    if(fabs(speed) > fabs(cmded_speed))
        speed = (speed < 0 ? -1.0 : 1.0) * fabs(cmded_speed);

    twist.linear.x = speed;
    twist.angular.z = z_twist;
    this->limitTwist(twist, mp_->max_controller_speed_, mp_->max_controller_angular_rate_);
    cmdVelRawPublisher_.publish(twist);

    monstertruck_msgs::Pdout pdout;
    pdout.header.frame_id = "world";
    pdout.header.stamp = ros::Time::now();
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

void DifferentialDriveController::executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error,
                                  double carrot_distance, double speed,
                                  double signed_carrot_distance_2_robot, double dt)
{
    double e_angle = carrot_relative_angle; // speed < 0 ? carrot_orientation_error : carrot_relative_angle;
    if(e_angle > M_PI + 1e-2 || e_angle < -M_PI -1e-2)
    {
        ROS_WARN("[vehicle_controller] [differential_drive_controller] Invalid angle was given.");
    }
    if(speed == 0.0)
    {
        ROS_INFO("[vehicle_controller] [differential_drive_controller] Commanded speed is 0");
        speed = 0.0;
    }
    executePDControlledMotionCommand(e_angle, signed_carrot_distance_2_robot, dt, speed);
}

void DifferentialDriveController::executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error, double carrot_distance, double speed)
{
    float sign = speed < 0.0 ? -1.0 : 1.0;

    twist.linear.x = speed;

    if (sign < 0){
        twist.angular.z = carrot_orientation_error / carrot_distance * 1.5 * 0.25;
    }else{
        twist.angular.z = carrot_relative_angle / carrot_distance * 1.5;
    }

    this->limitTwist(twist, mp_->max_controller_speed_, mp_->max_controller_angular_rate_);

    cmdVelRawPublisher_.publish(twist);
}

void DifferentialDriveController::stop()
{
    twist.angular.z = 0.0;
    twist.linear.x = 0.0;
    cmdVelRawPublisher_.publish(twist);
}

void DifferentialDriveController::limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate)
{
    double speed = twist.linear.x;
    double angular_rate = twist.angular.z;

    mp_->limitSpeed(speed);

    speed        = std::max(-mp_->max_unlimited_speed_, std::min(mp_->max_unlimited_speed_, speed));
    angular_rate = std::max(-mp_->max_unlimited_angular_rate_, std::min(mp_->max_unlimited_angular_rate_, angular_rate));

    double m = - mp_->max_controller_speed_ / mp_->max_controller_angular_rate_;
    double t = mp_->max_controller_speed_;
    double speedAbsUL = std::min(std::max(0.0, m * std::abs(angular_rate) + t), max_speed);

    speed = std::max(-speedAbsUL, std::min(speed, speedAbsUL));
    angular_rate = std::max(-max_angular_rate, std::min(max_angular_rate, angular_rate));

    twist.linear.x = speed;
    twist.angular.z = angular_rate;
}

