#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <vehicle_controller/controller.h>
#include <vehicle_controller/LqrControllerParamsConfig.h>

#include <vehicle_controller/ekf.h>

class Lqr_Controller : public Controller
{
public:
  typedef enum { INACTIVE, VELOCITY, DRIVETO, DRIVEPATH } State;

  Lqr_Controller(ros::NodeHandle& nh_);
  virtual ~Lqr_Controller();
  virtual bool configure();

  inline virtual std::string getName()
  {
    return "LQR Controller";
  }

protected:
  virtual void update();
  virtual void reset();
  virtual void stop();

  virtual bool driveto(const geometry_msgs::PoseStamped&, double speed);
  virtual bool drivepath(const nav_msgs::Path& path);

  virtual bool updateRobotState(const nav_msgs::Odometry& odom_state);
  virtual void stateCallback(const nav_msgs::OdometryConstPtr& odom_state);
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

  virtual void controllerParamsCallback(vehicle_controller::LqrControllerParamsConfig & config, uint32_t level);

  //lqr control specific functions
  void calc_local_path();
  int calcClosestPoint();
  void calcLqr();
  void solveDare();
  void limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate) const;

private:
  ros::NodeHandle nh;
  tf::TransformListener listener;

  ros::Subscriber stateSubscriber;
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

  ros::NodeHandle nh_dr_params;
  dynamic_reconfigure::Server<vehicle_controller::LqrControllerParamsConfig> * dr_controller_params_server;

  inline bool isDtInvalid()
  {
      return dt <= 0.0;
  }

  std::unique_ptr<StuckDetector> stuck;

  // LQR specific variables
  nav_msgs::Path current_path;
  geometry_msgs::PointStamped closest_point;
  double rot_vel_dir, lin_vel_dir;
  double local_path_radius;
  double alignment_angle;

  double lqr_y_error, lqr_x_error;
  double lqr_angle_error;

  geometry_msgs::Twist lqr_last_cmd;
  double lqr_last_y_error;
  double lqr_last_angle_error;
  double lqr_expected_dy;
  double lqr_real_dy;

  double lqr_speed_reduction_gain;

  double lqr_q11;
  double lqr_q22;
  double lqr_r;
  bool lqr_aligning = false;
  bool lqr_aligning_2 = false;

  double lqr_p11, lqr_p12, lqr_p22;
  double lqr_k1, lqr_k2, lqr_k3;

  ros::Time lqr_time;
  double lqr_y_error_integrate;

  double roll, pitch, yaw;

  Eigen::Matrix<double, 1, 2> K;


  ros::Time ekf_lastTime;
  geometry_msgs::Twist ekf_lastCmd;
  EKF ekf;
  bool ekf_setInitialPose = false;
  bool ekf_useEkf = false;
  double ekf_last_yaw, ekf_last_roll, ekf_last_pitch;

  ros::Publisher cmd_vel_pub;


  //observer
  double obs_error_1;
  double obs_error_2;
  double obs_r = 100;
  double obs_q11 = 100;
  double obs_q22 = 0.1;
  double obs_b1;
  double obs_b2;
  ros::Time obs_last_time = ros::Time::now();
};


#endif // LQR_CONTROLLER_H
