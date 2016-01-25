//
// Created by paul on 18.11.15.
//

#include <vehicle_controller/four_wheel_steer_controller.h>

void FourWheelSteerController::configure(ros::NodeHandle& params, MotionParameters* mp)
{
  mp_ = mp;
  mp_->USE_FINAL_TWIST_ = false;

  ros::NodeHandle nh;
  drivePublisher_ = nh.advertise<monstertruck_msgs::MotionCommand>("drive", 1);

  max_steeringangle = 30.0 * M_PI/180.0;
  params.getParam("max_steeringangle", max_steeringangle);
}

void FourWheelSteerController::limitSpeed(double & speed)
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

void FourWheelSteerController::executeTwist(const geometry_msgs::Twist& velocity)
{
  double backward = (velocity.linear.x < 0) ? -1.0 : 1.0;
  double speed = backward * sqrt(velocity.linear.x*velocity.linear.x + velocity.linear.y*velocity.linear.y);

  limitSpeed(speed);

  float kappa = velocity.angular.z * speed;
  float tan_gamma = tan(velocity.linear.y / velocity.linear.x);

  setDriveCommand(speed, kappa, tan_gamma);
}

void FourWheelSteerController::executeMotionCommand(RobotControlState rcs)
{
    double sign      = rcs.desired_velocity_linear < 0.0 ? -1.0 : 1.0;
    double kappa     = sign * rcs.error_2_carrot_angular / rcs.carrot_distance * 1.5;
    double tan_gamma = tan(rcs.error_2_path_angular - rcs.error_2_carrot_angular);

    this->setDriveCommand(rcs.desired_velocity_linear, kappa ,tan_gamma);
}

void FourWheelSteerController::stop()
{
  drive.speed = 0.0;
  drivePublisher_.publish(drive);
}

double FourWheelSteerController::getCommandedSpeed() const
{
  return drive.speed;
}

void FourWheelSteerController::setDriveCommand(double speed, double kappa, double tan_gamma)
{

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
