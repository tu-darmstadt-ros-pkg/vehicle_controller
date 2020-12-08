#ifndef DIFFERENTIAL_PURE_PURSUIT_H
#define DIFFERENTIAL_PURE_PURSUIT_H

#include <vehicle_controller/controller.h>

#include <vehicle_controller/PurePursuitControllerParamsConfig.h>

#include <vehicle_controller/ekf.h>

class Differential_Pure_Pursuit_Controller : public Controller
{
public:
  Differential_Pure_Pursuit_Controller(ros::NodeHandle& nh_);
  virtual ~Differential_Pure_Pursuit_Controller();

  virtual bool configure() override;

  inline virtual std::string getName()
  {
    return "Differential Pure Pursuit Controller";
  }

protected:

  virtual void computeMoveCmd() override;

  //Pure pursuit specific functions

  virtual void controllerParamsCallback(vehicle_controller::PurePursuitControllerParamsConfig & config, uint32_t level);


  ros::NodeHandle nh_dr_params;
  dynamic_reconfigure::Server<vehicle_controller::PurePursuitControllerParamsConfig> * dr_controller_params_server;


};


#endif // DIFFERENTIAL_PURE_PURSUIT_H
