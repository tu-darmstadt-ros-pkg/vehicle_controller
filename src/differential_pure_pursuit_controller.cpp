#include <vehicle_controller/differential_pure_pursuit_controller.h>

Differential_Pure_Pursuit_Controller::Differential_Pure_Pursuit_Controller(ros::NodeHandle& nh_)
  : Controller(nh_), nh_dr_params("~/purep_controller_params")
{

}

Differential_Pure_Pursuit_Controller::~Differential_Pure_Pursuit_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    dr_controller_params_server->clearCallback();
  }
}

bool Differential_Pure_Pursuit_Controller::configure()
{
  Controller::configure();

  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::PurePursuitControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Differential_Pure_Pursuit_Controller::controllerParamsCallback, this, _1, _2));

  return true;
}


void Differential_Pure_Pursuit_Controller::computeMoveCmd(){

  double dist_to_carrot = robot_control_state.signed_carrot_distance_2_robot;

  double carrot_distance_y = std::sin(robot_control_state.error_2_path_angular) * dist_to_carrot;

  double curv =  2 * carrot_distance_y / (dist_to_carrot*dist_to_carrot);

  geometry_msgs::Twist cmd;
  cmd.linear.x = robot_control_state.desired_velocity_linear;
  cmd.linear.y = 0.0;
  cmd.angular.z = curv * cmd.linear.x;

  vehicle_control_interface_->executeTwist(cmd, robot_control_state, yaw, pitch, roll);

  //ROS_INFO("lin vel: %f, ang vel: %f", cmd.linear.x, cmd.angular.z);
  //ROS_INFO("curv: %f", curv);
}


void Differential_Pure_Pursuit_Controller::controllerParamsCallback(vehicle_controller::PurePursuitControllerParamsConfig &config, uint32_t level){
  mp_.carrot_distance = config.lookahead_distance;
}


