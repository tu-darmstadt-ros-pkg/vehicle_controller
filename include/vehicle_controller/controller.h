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

#include <hector_move_base_msgs/MoveBaseActionGeneric.h>
#include <hector_move_base_msgs/MoveBaseActionGoal.h>
#include <hector_move_base_msgs/MoveBaseActionPath.h>
#include <hector_move_base_msgs/MoveBaseActionResult.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <vehicle_controller/vehicle_control_interface.h>
#include <vehicle_controller/motion_parameters.h>

#include <vehicle_controller/ps3d.h>

class Controller {
public:
  typedef enum { INACTIVE, VELOCITY, DRIVETO, DRIVEPATH } State;
  typedef struct {
    float x;
    float y;
    float orientation;
  } Point;

  typedef struct {
    Point p1;
    Point p2;
    float course;

    bool backward;
    float speed;
    float length2;
    float length;
    float percent;
  } Leg;
  typedef std::vector<Leg> Legs;

  Controller(const std::string &ns = std::string());
  virtual ~Controller();

  friend int main(int, char**);

  bool pathToBeSmoothed(const std::deque<geometry_msgs::Pose> &transformed_path);
  bool createDrivepath2MapTransform(tf::StampedTransform  & transform, const nav_msgs::Path& path);
  geometry_msgs::Pose createPoseFromQuatAndPosition(vec3 const & position, quat const & orientation);

protected:
  virtual bool configure();
  virtual void update();
  virtual void reset();
  virtual void stop();
  virtual void cleanup();

  virtual bool driveto(const geometry_msgs::PoseStamped&);
  virtual bool drivepath(const nav_msgs::Path&);

  virtual void stateCallback(const nav_msgs::Odometry&);
  virtual void drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>&);
  virtual void drivepathCallback(const ros::MessageEvent<nav_msgs::Path>&);
  virtual void cmd_velCallback(const geometry_msgs::Twist&);
  virtual void cmd_velTeleopCallback(const geometry_msgs::Twist&);
  virtual void speedCallback(const std_msgs::Float32&);
  virtual bool alternativeTolerancesService(monstertruck_msgs::SetAlternativeTolerance::Request& req, monstertruck_msgs::SetAlternativeTolerance::Response& res);

  virtual void actionCallback(const hector_move_base_msgs::MoveBaseActionGeneric&);
  virtual void actionGoalCallback(const hector_move_base_msgs::MoveBaseActionGoal&);
  virtual void actionPathCallback(const hector_move_base_msgs::MoveBaseActionPath&);
  virtual void publishActionResult(actionlib_msgs::GoalStatus::_status_type, const std::string& text = std::string());

  void addLeg(geometry_msgs::Pose const&);
  void setDriveCommand(float speed, float kappa, float tan_gamma);

private:
  ros::NodeHandle nh;
  tf::TransformListener listener;

  ros::Subscriber stateSubscriber;
  ros::Subscriber drivetoSubscriber;
  ros::Subscriber drivepathSubscriber;
  ros::Subscriber cmd_velSubscriber;
  ros::Subscriber cmd_velTeleopSubscriber;
  ros::Subscriber speedSubscriber;

  ros::Publisher carrotPosePublisher;
  ros::Publisher lookatPublisher;
  ros::Publisher cameraOrientationPublisher;
  ros::Publisher drivepathPublisher;
  ros::Publisher diagnosticsPublisher;

  ros::Publisher pathPosePublisher;

  // action interface
  ros::Subscriber actionSubscriber;
  ros::Subscriber actionGoalSubscriber;
  ros::Subscriber actionPathSubscriber;
  ros::Publisher actionResultPublisher;

  //Service Provider
  ros::ServiceServer alternative_tolerances_service;

  State state;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Vector3Stamped velocity;
  //monstertruck_msgs::MotionCommand drive;
  geometry_msgs::PoseStamped carrotPose;
  actionlib_msgs::GoalIDPtr goalID;

  nav_msgs::Path empty_path;

  unsigned int current;
  geometry_msgs::Pose start;
  Legs legs;

  // parameters
  MotionParameters motion_control_setup;

  std::string map_frame_id;
  std::string base_frame_id;

  bool camera_control;
  double camera_lookat_distance;
  double camera_lookat_height;
  geometry_msgs::QuaternionStamped cameraDefaultOrientation;

  bool check_if_blocked;
  double dt;
  //double current_velocity;
  //double current_inclination;
  double velocity_error;
  double velocity_blocked_time;
  double linear_speed_blocked_;
  double angular_speed_blocked_;

  double goal_position_tolerance;
  double goal_angle_tolerance;

  actionlib_msgs::GoalIDPtr alternative_tolerance_goalID;
  double alternative_goal_position_tolerance;
  double alternative_angle_tolerance;

  boost::shared_ptr<VehicleControlInterface> vehicle_control_interface_;

  std::string vehicle_type;

  inline void invalidateDt()
  {
      dt = 0.0;
  }

  inline bool isDtInvalid()
  {
      return dt == 0.0;
  }

  std::deque< geometry_msgs::PoseStamped > pose_history_;
};

#endif // VEHICLE_CONTROLLER_H
