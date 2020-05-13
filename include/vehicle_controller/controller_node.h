#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <vehicle_controller/carrot_controller.h>
#include <vehicle_controller/daf_controller.h>
#include <vehicle_controller/ackermann_pure_pursuit_controller.h>
#include <vehicle_controller/differential_pure_pursuit_controller.h>
#include <vehicle_controller/controller.h>

#include <vehicle_controller/lqr_controller.h>

#include <dynamic_reconfigure/server.h>
#include <vehicle_controller/ControllerTypeConfig.h>

class Controller_Node
{
public:
  typedef enum { CARROT, DAF, ACKERM_PP, DIFF_PP , LQR} Control_Type_Enum;

  Controller_Node(ros::NodeHandle& nh_);
  virtual ~Controller_Node();

protected:

  boost::shared_ptr<Controller> control;

  std::string controller_type;

  dynamic_reconfigure::Server<vehicle_controller::ControllerTypeConfig> * dr_controller_type_server = 0;

  ros::NodeHandle nh;

  virtual void reset();

  virtual void controllerTypeCallback(vehicle_controller::ControllerTypeConfig & config, uint32_t level){

    if(config.controller_type == DAF){
      controller_type = "daf";
    }
    else if(config.controller_type == ACKERM_PP){
      controller_type = "ackermann_pure_pursuit";
    }
    else if(config.controller_type == DIFF_PP){
      controller_type = "differential_pure_pursuit";
    }
    else if (config.controller_type == LQR){
      controller_type = "lqr";
    }
    else{
      controller_type = "carrot";
    }
    reset();
  }


};

#endif // VEHICLE_CONTROLLER_H
