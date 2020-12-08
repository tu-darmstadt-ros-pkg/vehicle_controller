#include <vehicle_controller/ackermann_pure_pursuit_controller.h>

Ackermann_Pure_Pursuit_Controller::Ackermann_Pure_Pursuit_Controller(ros::NodeHandle& nh_)
  : Controller(nh_), nh_dr_params("~/ackermann_purep_controller_params")
{
}

Ackermann_Pure_Pursuit_Controller::~Ackermann_Pure_Pursuit_Controller()
{
  if(dr_controller_params_server){
    dr_controller_params_server->clearCallback();
  }
}

bool Ackermann_Pure_Pursuit_Controller::configure()
{
  Controller::configure();
  ros::NodeHandle params("~");  
  params.param("vehicle_length", vehicle_length, 0.5);

  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::PurePursuitControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Ackermann_Pure_Pursuit_Controller::controllerParamsCallback, this, _1, _2));

  return true;
}

void Ackermann_Pure_Pursuit_Controller::computeMoveCmd(){
  double lookahead_distance = robot_control_state.carrot_distance;

  const double alpha = robot_control_state.error_2_path_angular;

  double delta = atan2(2. * vehicle_length * sin(alpha), lookahead_distance);

  geometry_msgs::Twist cmd;
  cmd.linear.x = robot_control_state.desired_velocity_linear;
  cmd.linear.y = 0.0;
  cmd.angular.z = delta;

  //cmd_vel_pub.publish(cmd);
  vehicle_control_interface_->executeTwist(cmd, robot_control_state, yaw ,pitch, roll);

}

void Ackermann_Pure_Pursuit_Controller::controllerParamsCallback(vehicle_controller::PurePursuitControllerParamsConfig &config, uint32_t level){
  mp_.carrot_distance = config.lookahead_distance;
}

