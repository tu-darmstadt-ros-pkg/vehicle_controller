#ifndef PURE_PURSUIT_ACKERMANN_H
#define PURE_PURSUIT_ACKERMANN_H

#include <vehicle_controller/controller.h>

#include <vehicle_controller/PurePursuitControllerParamsConfig.h>

class Ackermann_Pure_Pursuit_Controller : public Controller
{
public:
  Ackermann_Pure_Pursuit_Controller(ros::NodeHandle& nh_);
  virtual ~Ackermann_Pure_Pursuit_Controller();
  virtual bool configure() override;

  inline virtual std::string getName()
  {
    return "Ackermann Pure Pursuit Controller";
  }

protected:
  virtual void computeMoveCmd() override;

  //Pure pursuit specific functions
  virtual void controllerParamsCallback(vehicle_controller::PurePursuitControllerParamsConfig & config, uint32_t level);


  double vehicle_length;

  ros::NodeHandle nh_dr_params;
  dynamic_reconfigure::Server<vehicle_controller::PurePursuitControllerParamsConfig> * dr_controller_params_server;

};

#endif // PURE_PURSUIT_ACKERMANN_H
