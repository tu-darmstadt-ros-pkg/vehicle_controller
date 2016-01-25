#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <monstertruck_msgs/MotionCommand.h>
#include <monstertruck_msgs/SetAlternativeTolerance.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <hector_move_base_msgs/MoveBaseActionGeneric.h>
#include <hector_move_base_msgs/MoveBaseActionGoal.h>
#include <hector_move_base_msgs/MoveBaseActionPath.h>
#include <hector_move_base_msgs/MoveBaseActionResult.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <vehicle_controller/vehicle_control_interface.h>
#include <vehicle_controller/motion_parameters.h>
#include <vehicle_controller/ps3d.h>
#include <vehicle_controller/stuck_detector.h>

#include <memory>
#include <vehicle_controller/ps3d.h>
#include <vehicle_controller/utility.h>

class Controller
{
public:
  typedef enum { INACTIVE, VELOCITY, DRIVETO, DRIVEPATH } State;

  Controller(const std::string &ns = std::string());
  virtual ~Controller();

  friend int main(int, char**);

protected:
  virtual bool configure();
  virtual void update();
  virtual void reset();
  virtual void stop();
  virtual void cleanup();

  virtual bool driveto(const geometry_msgs::PoseStamped&);
  virtual bool drivepath(const nav_msgs::Path&, bool fixed_path = true);

  virtual void stateCallback(const nav_msgs::Odometry&);
  virtual void drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>&);
  virtual void drivepathCallback(const ros::MessageEvent<nav_msgs::Path>&);
  virtual void cmd_velCallback(const geometry_msgs::Twist&);
  virtual void cmd_velTeleopCallback(const geometry_msgs::Twist&);
  virtual void speedCallback(const std_msgs::Float32&);
  virtual bool alternativeTolerancesService(monstertruck_msgs::SetAlternativeTolerance::Request& req, monstertruck_msgs::SetAlternativeTolerance::Response& res);

  void joint_statesCallback(sensor_msgs::JointStateConstPtr msg);
  void cmd_flipper_toggleCallback(const std_msgs::Empty&);

  virtual void actionCallback(const hector_move_base_msgs::MoveBaseActionGeneric&);
  virtual void actionGoalCallback(const hector_move_base_msgs::MoveBaseActionGoal&);
  virtual void actionPathCallback(const hector_move_base_msgs::MoveBaseActionPath&);
  virtual void publishActionResult(actionlib_msgs::GoalStatus::_status_type, const std::string& text = std::string());

  void addLeg(geometry_msgs::Pose const&);
  void setDriveCommand(float speed, float kappa, float tan_gamma);

  bool pathToBeSmoothed(const std::deque<geometry_msgs::Pose> &transformed_path, bool fixed_path);
  bool createDrivepath2MapTransform(tf::StampedTransform  & transform, const nav_msgs::Path& path);
  geometry_msgs::Pose createPoseFromQuatAndPosition(vec3 const & position, quat const & orientation);

private:
  ros::NodeHandle nh;
  tf::TransformListener listener;

  ros::Subscriber stateSubscriber;
  ros::Subscriber drivetoSubscriber;
  ros::Subscriber drivepathSubscriber;
  ros::Subscriber cmd_velSubscriber;
  ros::Subscriber cmd_velTeleopSubscriber;
  ros::Subscriber speedSubscriber;
  ros::Subscriber cmd_flipper_toggle_sub_;
  ros::Subscriber joint_states_sub_;

  ros::Publisher endPosePoublisher;
  ros::Publisher carrotPosePublisher;
  ros::Publisher lookatPublisher;
  ros::Publisher cameraOrientationPublisher;
  ros::Publisher drivepathPublisher;
  ros::Publisher diagnosticsPublisher;

  ros::Publisher pathPosePublisher;
  ros::Publisher autonomy_level_pub_;
  ros::Publisher jointstate_cmd_pub_;

  // action interface
  ros::Subscriber actionSubscriber;
  ros::Subscriber actionGoalSubscriber;
  ros::Subscriber actionPathSubscriber;
  ros::Publisher actionResultPublisher;

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
  geometry_msgs::Pose start;
  Legs legs;

  double flipper_state;

  // motion parameters (set at launch)
  MotionParameters mp_;

  std::string map_frame_id;
  std::string base_frame_id;

  bool camera_control;
  double camera_lookat_distance;
  double camera_lookat_height;
  geometry_msgs::QuaternionStamped cameraDefaultOrientation;

  bool check_stuck;
  double dt;

  double velocity_error;

  double goal_position_tolerance;
  double goal_angle_tolerance;

  actionlib_msgs::GoalIDPtr alternative_tolerance_goalID;
  double alternative_goal_position_tolerance;
  double alternative_angle_tolerance;

  boost::shared_ptr<VehicleControlInterface> vehicle_control_interface_;

  std::string vehicle_control_type;
  int final_twist_trials;

  inline void invalidateDt()
  {
      dt = 0.0;
  }

  inline bool isDtInvalid()
  {
      return dt <= 0.0;
  }

  std::unique_ptr<StuckDetector> stuck;
};

#endif // VEHICLE_CONTROLLER_H
