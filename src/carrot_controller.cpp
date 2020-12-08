#include <vehicle_controller/carrot_controller.h>

Carrot_Controller::Carrot_Controller(ros::NodeHandle& nh_)
  : Controller(nh_), nh_dr_params("~/carrot_controller_params")
{
}

Carrot_Controller::~Carrot_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    dr_controller_params_server->clearCallback();
  }
}

bool Carrot_Controller::configure(){
  Controller::configure();

  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::CarrotControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Carrot_Controller::controllerParamsCallback, this, _1, _2));

  return true;
}


void Carrot_Controller::computeMoveCmd(){
  vehicle_control_interface_->executeMotionCommand(robot_control_state);
}


void Carrot_Controller::controllerParamsCallback(vehicle_controller::CarrotControllerParamsConfig &config, uint32_t level){
  ROS_INFO_STREAM("HALLO" << mp_.carrot_distance << " " << config.carrot_distance);
  mp_.carrot_distance = config.carrot_distance;
  ROS_INFO_STREAM("HALLO2" << mp_.carrot_distance);
}
