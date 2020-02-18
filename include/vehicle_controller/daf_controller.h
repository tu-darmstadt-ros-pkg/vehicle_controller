#ifndef DAF_CONTROLLER_H
#define DAF_CONTROLLER_H

#include <vehicle_controller/controller.h>
#include <vehicle_controller/quaternions.h>
#include <vehicle_controller/utility.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <limits>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <vehicle_controller/four_wheel_steer_controller.h>
#include <vehicle_controller/differential_drive_controller.h>

#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <sstream>
#include <functional>

#define PI 3.14159265

class Daf_Controller{
public:
  typedef enum { INACTIVE, VELOCITY, DRIVETO, DRIVEPATH } State;

  Daf_Controller(const std::string &ns = std::string());
  virtual ~Daf_Controller();

  friend int main(int, char**);

protected:
  virtual bool configure();
  virtual void reset();
  virtual void stop();

  virtual bool driveto(const geometry_msgs::PoseStamped&, double speed);
  virtual bool drivepath(const nav_msgs::Path& path);

  virtual bool updateRobotState(const nav_msgs::Odometry& odom_state);
  virtual void stateCallback(const nav_msgs::OdometryConstPtr& odom_state);
  virtual void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  virtual void drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>&);
  virtual void drivepathCallback(const ros::MessageEvent<nav_msgs::Path>&);
  virtual void cmd_velCallback(const geometry_msgs::Twist&);
  virtual void cmd_velTeleopCallback(const geometry_msgs::Twist&);
  virtual void speedCallback(const std_msgs::Float32&);
  virtual void poseCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>&);

  void stopVehicle();

  void followPathGoalCallback();
  void followPathPreemptCallback();

  /**
   * @brief addLeg to current tracking path
   * @param pose to be added
   * @param speed 0 indicates that the speed from the configuration is used as
   *        desired linear speed in the lower controllers, otherwise the given
   *        value is used for this purpose
   */
  void addLeg(const geometry_msgs::PoseStamped &pose, double speed = 0.0);

  bool reverseAllowed();
  bool reverseForced();
  bool pathToBeSmoothed(const std::deque<geometry_msgs::PoseStamped> &transformed_path, bool fixed_path);
  bool createDrivepath2MapTransform(tf::StampedTransform  & transform, const nav_msgs::Path& path);
  geometry_msgs::PoseStamped createPoseFromQuatAndPosition(vec3 const & position, quat const & orientation);

  //Daf specific methods:
  void calc_local_path();
  void calc_ground_compensation();
  void check_robot_stability();
  void velocity_increase();
  void calculate_cmd();
  void calc_angel_compensation();
  void calculate_al_rot();

private:
  ros::NodeHandle nh;
  tf::TransformListener listener;

  ros::Subscriber stateSubscriber;
  ros::Subscriber imuSubscriber;
  ros::Subscriber drivetoSubscriber;
  ros::Subscriber drivepathSubscriber;
  ros::Subscriber cmd_velSubscriber;
  ros::Subscriber cmd_velTeleopSubscriber;
  ros::Subscriber speedSubscriber;
  ros::Subscriber poseSubscriber;


  ros::Publisher endPosePoublisher;
  ros::Publisher carrotPosePublisher;
  ros::Publisher lookatPublisher;
  ros::Publisher cameraOrientationPublisher;
  ros::Publisher drivepathPublisher;
  ros::Publisher diagnosticsPublisher;

  ros::Publisher pathPosePublisher;
  ros::Publisher autonomy_level_pub_;

  ros::Publisher cmd_vel_pub;
  ros::Publisher local_path_pub;
  ros::Publisher marker_pub;

  // action interface
  boost::shared_ptr<actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction> > follow_path_server_;
  actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>::GoalConstPtr follow_path_goal_;

  //Service Provider
  ros::ServiceServer alternative_tolerances_service;

  State state;
  std_msgs::Header  robot_state_header;
  RobotControlState robot_control_state;


  //monstertruck_msgs::MotionCommand drive;
  geometry_msgs::PoseStamped carrotPose;
  actionlib_msgs::GoalIDPtr goalID;

  nav_msgs::Path empty_path;

  unsigned int current;
  geometry_msgs::PoseStamped start;
  Legs legs;

  double flipper_state;

  // motion parameters (set at launch)
  MotionParameters mp_;
  // path-specific settings
  move_base_lite_msgs::FollowPathOptions default_path_options_;

  std::string map_frame_id;
  std::string base_frame_id;

  bool camera_control;
  double camera_lookat_distance;
  double camera_lookat_height;
  geometry_msgs::QuaternionStamped cameraDefaultOrientation;

  bool check_stuck;
  double dt;

  double velocity_error;

  geometry_msgs::PoseStamped current_pose;


  boost::shared_ptr<VehicleControlInterface> vehicle_control_interface_;

  std::string vehicle_control_type;
  int final_twist_trials;

  nav_msgs::OdometryConstPtr latest_odom_;

  inline bool isDtInvalid()
  {
      return dt <= 0.0;
  }

  std::unique_ptr<StuckDetector> stuck;

  //Daf specific variables
  nav_msgs::Odometry odom;
  geometry_msgs::Twist cmd;

  nav_msgs::Path curr_path;
  nav_msgs::Path calc_path;
  nav_msgs::Path local_calc_path;

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
  double execution_period;
  double k_p_rotation;
  double alignment_angle;
  double roll, pitch, yaw;
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
  double imu_roll, imu_pitch, imu_yaw;
  double lower_al_angle, upper_al_angle;
  double stability_angle;

  int co_unchanged, co_points;
  int psize, st_point, path_po_lenght;
  int err_cont;
  int oscilation_rotation;

};

#endif // DAF_CONTROLLER_H
