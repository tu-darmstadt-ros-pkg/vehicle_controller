#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <vehicle_controller/carrot_controller.h>
#include <vehicle_controller/daf_controller.h>
#include <vehicle_controller/controller.h>

class Controller_Node
{

public:

  Controller_Node(ros::NodeHandle& nh_);
  virtual ~Controller_Node();

  boost::shared_ptr<Controller> control;

  std::string controller_type;

};

#endif // VEHICLE_CONTROLLER_H
