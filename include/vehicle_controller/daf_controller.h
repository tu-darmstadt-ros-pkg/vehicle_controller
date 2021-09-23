#ifndef DAF_CONTROLLER_H
#define DAF_CONTROLLER_H

#include <sensor_msgs/Imu.h>

#include <vehicle_controller/controller.h>
#include <vehicle_controller/DafControllerParamsConfig.h>

#include <visualization_msgs/Marker.h>

#define PI 3.14159265

class Daf_Controller : public Controller{
public:
  typedef enum { INACTIVE, VELOCITY, DRIVETO, DRIVEPATH } State;

  Daf_Controller(ros::NodeHandle &nh_);
  virtual ~Daf_Controller();

  virtual bool configure() override;

  inline virtual std::string getName()
  {
    return "Dynamic Arc Fitting Controller";
  }

protected:
  virtual void reset() override;

  virtual void computeMoveCmd() override;

  virtual void stateCallback(const nav_msgs::OdometryConstPtr& odom_state) override;

  virtual void followPathGoalCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle goal) override;

  //Daf specific methods:
  void calc_local_path();
  void calc_ground_compensation();
  void check_robot_stability();
  void velocity_increase();
  void calc_angel_compensation();
  void calculate_al_rot();

  virtual void controllerParamsCallback(vehicle_controller::DafControllerParamsConfig & config, uint32_t level);


  ros::Publisher local_path_pub;
  ros::Publisher marker_pub;

  //Daf specific variables
  geometry_msgs::Twist cmd;

  nav_msgs::Path curr_path;
  nav_msgs::Path calc_path;
  nav_msgs::Path local_calc_path;

  ros::NodeHandle nh_dr_params;
  dynamic_reconfigure::Server<vehicle_controller::DafControllerParamsConfig> * dr_controller_params_server;

  bool alignment_finished;
  bool show_trajectory_planing;
  bool move_robot;
  bool enable_angle_compensation;
  bool enable_ground_compensation;
  bool enable_velocity_increase;

  double angle_diff;
  double pub_cmd_hz;
  double max_lin_speed, min_lin_speed;
  double max_rot_speed, min_rot_speed;
  double k_p_rotation;
  double alignment_angle;
  double update_skip;
  double curr_dist;
  double rot_dir_opti, rot_vel_dir;
  double lin_vel_dir;
  double lin_vel, rot_vel, lin_vel_ref;
  double points[50][2];
  double max_H, Wid, rad;
  double global_goal_tolerance;
  double th_po_x, th_po_y, fi_po_x, fi_po_y, se_po_x, se_po_y;
  double dirx, diry;
  double sideA, sideB, sideC;
  double ss, area, tmp_H;
  double al_an_diff;
  double midX, midY;
  double dx, dy;
  double distt, pdist;
  double mDx, mDy;
  double old_pos_x;
  double old_pos_y;
  double glo_pos_diff_x, glo_pos_diff_y;
  double rot_correction_factor;
  double lower_al_angle, upper_al_angle;
  double stability_angle;

  int co_unchanged, co_points;
  int psize, st_point, path_po_lenght;
  int err_cont;
  int oscilation_rotation;

};

#endif // DAF_CONTROLLER_H
