#ifndef PURE_PURSUIT_ACKERMANN_H
#define PURE_PURSUIT_ACKERMANN_H

#include <vehicle_controller/controller.h>

#include <vehicle_controller/PurePursuitControllerParamsConfig.h>

class Ackermann_Pure_Pursuit_Controller : public Controller
{
public:
  Ackermann_Pure_Pursuit_Controller(ros::NodeHandle& nh_);
  virtual ~Ackermann_Pure_Pursuit_Controller();
  virtual bool configure();

  inline virtual std::string getName()
  {
    return "Ackermann Pure Pursuit Controller";
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

  virtual void computeMoveCmd(RobotControlState control_state);
  virtual double exponentialSpeedControll();


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

  virtual void controllerParamsCallback(vehicle_controller::PurePursuitControllerParamsConfig & config, uint32_t level);

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
  ros::Publisher cmd_vel_pub;

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

  double yaw, roll, pitch;

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

  double vehicle_length;

  geometry_msgs::PoseStamped current_pose;


  boost::shared_ptr<VehicleControlInterface> vehicle_control_interface_;

  std::string vehicle_control_type;
  int final_twist_trials;

  nav_msgs::OdometryConstPtr latest_odom_;

  ros::NodeHandle nh_dr_params;
  dynamic_reconfigure::Server<vehicle_controller::PurePursuitControllerParamsConfig> * dr_controller_params_server;

  inline bool isDtInvalid()
  {
    return dt <= 0.0;
  }

  std::unique_ptr<StuckDetector> stuck;


};

#endif // PURE_PURSUIT_ACKERMANN_H
