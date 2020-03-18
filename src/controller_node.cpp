#include <vehicle_controller/controller_node.h>

Controller_Node::Controller_Node(ros::NodeHandle &nh_)
{
  nh = nh_;
  ros::NodeHandle params("~");
  params.param<std::string>("controller_type", controller_type, "carrot");

  dr_controller_type_server = new dynamic_reconfigure::Server<vehicle_controller::ControllerTypeConfig>;
  dr_controller_type_server->setCallback(boost::bind(&Controller_Node::controllerTypeCallback, this, _1, _2));

  reset();
}

Controller_Node::~Controller_Node(){
}

void Controller_Node::reset(){
  if(controller_type == "daf"){
    control.reset(new Daf_Controller(nh));
  }
  else if(controller_type == "ackermann_pure_pursuit"){
    control.reset(new Ackermann_Pure_Pursuit_Controller(nh));
  }
  else if(controller_type == "differential_pure_pursuit"){
    control.reset(new Differential_Pure_Pursuit_Controller(nh));
  }
  else{
    control.reset(new Carrot_Controller(nh));
  }
  control->configure();
  ROS_INFO("Vehicle Controller Type is: %s", this->control->getName().c_str());
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle nh;

  Controller_Node cn(nh);


  while(ros::ok())
  {
    ros::spin();
  }

  ros::shutdown();
  return 0;
}
