#ifndef VEHICLE_CONTROLLER_CONTROLLER_NODE_H
#define VEHICLE_CONTROLLER_CONTROLLER_NODE_H

#include <vehicle_controller/carrot_controller.h>
#include <vehicle_controller/daf_controller.h>
#include <vehicle_controller/ackermann_pure_pursuit_controller.h>
#include <vehicle_controller/differential_pure_pursuit_controller.h>
#include <vehicle_controller/controller.h>

#include <vehicle_controller/lqr_controller.h>

#include <dynamic_reconfigure/server.h>
#include <vehicle_controller/ControllerTypeConfig.h>

class ControllerNode
{
public:
  typedef enum { CARROT, DAF, ACKERM_PP, DIFF_PP , LQR} Control_Type_Enum;

  explicit ControllerNode(const ros::NodeHandle& nh);

protected:
  void reset();
  void controllerTypeCallback(vehicle_controller::ControllerTypeConfig & config, uint32_t level);

  ros::NodeHandle nh_;
  std::shared_ptr<dynamic_reconfigure::Server<vehicle_controller::ControllerTypeConfig>> controller_type_reconfigure_server_;

  std::shared_ptr<Controller> controller_;
  std::string controller_type_;
};

#endif // VEHICLE_CONTROLLER_CONTROLLER_NODE_H
