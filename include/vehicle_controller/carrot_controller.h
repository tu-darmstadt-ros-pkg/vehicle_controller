#ifndef CARROT_CONTROLLER_H
#define CARROT_CONTROLLER_H

#include <vehicle_controller/controller.h>
#include <vehicle_controller/CarrotControllerParamsConfig.h>


class Carrot_Controller : public Controller
{
public:

  Carrot_Controller(ros::NodeHandle& nh_);
  virtual ~Carrot_Controller();

  bool configure() override;

  inline std::string getName() override
  {
    return "Carrot controller";
  }

protected:
  void computeMoveCmd() override;

  virtual void controllerParamsCallback(vehicle_controller::CarrotControllerParamsConfig & config, uint32_t level);

protected:

  ros::NodeHandle nh_dr_params;
  dynamic_reconfigure::Server<vehicle_controller::CarrotControllerParamsConfig> * dr_controller_params_server;


};

#endif // CARROT_CONTROLLER_H
