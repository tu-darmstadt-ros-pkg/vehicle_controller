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

void DifferentialDriveController::pdGainCallback(vehicle_controller::PdParamsConfig & config, uint32_t level)
{
    KP_ANGLE_ = config.angle_p_gain;
    KD_ANGLE_ = config.angle_d_gain;
    KP_POSITION_ = config.position_p_gain;
    KD_POSITION_ = config.position_d_gain;
}

void DifferentialDriveController::executeUnlimitedTwist(const geometry_msgs::Twist& inc_twist)
{
    twist = inc_twist;

    double speed = twist.linear.x;
    mp_->limitSpeed(speed);
    double angular_rate = twist.angular.z;
    angular_rate = std::max<double>(-mp_->max_unlimited_angular_rate_, std::min<double>(mp_->max_unlimited_angular_rate_, angular_rate));
    speed = std::max(-mp_->max_unlimited_speed_, std::min(speed, mp_->max_unlimited_speed_));

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

void DifferentialDriveController::executePDControlledMotionCommand(double e_angle, double e_position, double dt)
{
    static double previous_e_angle = e_angle;
    static double previous_e_position = e_position;

    double de_angle_dt    = (e_angle - previous_e_angle) / dt; // causes discontinuity @ orientation_error vs relative_angle switch
    double de_position_dt = (e_position - previous_e_position) / dt;

    // double speed = KP_POSITION_ * e_position + KD_POSITION_ * de_position_dt;
    // double EPS = 0.05;
    // double speed = (e_position > 0 ? 1.0 : -1.0 ) * mp_->max_controller_speed_;
    // if(fabs(e_position) < 0.05)

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

void DifferentialDriveController::executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error,
                                  double carrot_distance, double speed,
                                  double signed_carrot_distance_2_robot, double dt)
{
    double e_angle = speed < 0 ? carrot_orientation_error : carrot_relative_angle;

    if(signed_carrot_distance_2_robot < 0 && fabs(e_angle) > M_PI_4)
        e_angle = carrot_relative_angle;
    executePDControlledMotionCommand(e_angle, signed_carrot_distance_2_robot, dt);
    // executeMotionCommand(carrot_relative_angle, carrot_orientation_error, carrot_distance, speed);
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

    mp_->limitSpeed(speed);
    double angular_rate = twist.angular.z;
    angular_rate = std::max(-mp_->max_unlimited_angular_rate_, std::min(mp_->max_unlimited_angular_rate_, angular_rate));

    double m = (0.12 - mp_->max_unlimited_speed_) / 0.4;
    double t = mp_->max_unlimited_speed_;
    double speedAbsUL = std::min(std::max(0.0, m * std::abs(angular_rate) + t), max_speed);

    speed = std::max(std::min(speed, speedAbsUL), -speedAbsUL);

    angular_rate = std::max(-max_angular_rate, std::min(max_angular_rate, angular_rate));

    twist.linear.x = speed;
    twist.angular.z = angular_rate;
}