#include <vehicle_controller/controller_node.h>

Controller_Node::Controller_Node(ros::NodeHandle &nh_)
{
  ros::NodeHandle params("~");
  params.param<std::string>("controller_type", controller_type, "carrot");

  if(controller_type == "daf"){
    control.reset(new Daf_Controller(nh_));
  }
  else if(controller_type == "ackermann_pure_pursuit"){
    control.reset(new Ackermann_Pure_Pursuit_Controller(nh_));
  }
  else if(controller_type == "differential_pure_pursuit"){
    control.reset(new Differential_Pure_Pursuit_Controller(nh_));
  }
  else{
    control.reset(new Carrot_Controller(nh_));
  }
  control->configure();
}

Controller_Node::~Controller_Node(){
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
