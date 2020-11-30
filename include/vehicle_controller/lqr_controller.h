#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <vehicle_controller/controller.h>
#include <vehicle_controller/LqrControllerParamsConfig.h>

#include <vehicle_controller/ekf.h>

class Lqr_Controller : public Controller
{
public:

  Lqr_Controller(ros::NodeHandle& nh_);
  virtual ~Lqr_Controller();

  virtual bool configure() override;

  inline virtual std::string getName()
  {
    return "LQR Controller";
  }

protected:
  virtual void update() override;
  virtual void reset() override;

  virtual void controllerParamsCallback(vehicle_controller::LqrControllerParamsConfig & config, uint32_t level);

  //lqr control specific functions
  void calc_local_path();
  int calcClosestPoint();
  void calcLqr();
  void solveDare();


  ros::NodeHandle nh_dr_params;
  dynamic_reconfigure::Server<vehicle_controller::LqrControllerParamsConfig> * dr_controller_params_server;

  // LQR specific variables
  geometry_msgs::PointStamped closest_point;
  double rot_vel_dir, lin_vel_dir;
  double local_path_radius;
  double alignment_angle;

  double lqr_y_error, lqr_x_error;
  double lqr_angle_error;

  geometry_msgs::Twist lqr_last_cmd;
  double lqr_last_y_error;
  double lqr_last_angle_error;


  double lqr_q11;
  double lqr_q22;
  double lqr_r;

  double lqr_p11, lqr_p12, lqr_p22;
  double lqr_k1, lqr_k2, lqr_k3;

  ros::Time lqr_time;
  Eigen::Matrix<double, 1, 2> K;

};


#endif // LQR_CONTROLLER_H
