#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

#include <monstertruck_msgs/MotionCommand.h>
#include <monstertruck_msgs/SetAlternativeTolerance.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <move_base_lite_msgs/FollowPathAction.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vehicle_controller/vehicle_control_interface.h>
#include <vehicle_controller/motion_parameters.h>
#include <vehicle_controller/ps3d.h>
#include <vehicle_controller/stuck_detector.h>
#include <vehicle_controller/four_wheel_steer_controller.h>
#include <vehicle_controller/differential_drive_controller.h>
#include <vehicle_controller/quaternions.h>
#include <vehicle_controller/utility.h>

#include <memory>
#include <limits>

#include <algorithm>
#include <sstream>
#include <functional>

class Controller {
public:
  typedef enum { INACTIVE, VELOCITY, DRIVETO, DRIVEPATH } State;

  virtual bool configure() = 0;

  virtual std::string getName() = 0;

protected:
  virtual void update() = 0;
  virtual void reset() = 0;
  virtual void stop() = 0;

  virtual bool driveto(const geometry_msgs::PoseStamped&, double speed) = 0;
  virtual bool drivepath(const nav_msgs::Path& path) = 0;

  virtual bool updateRobotState(const nav_msgs::Odometry& odom_state) = 0;
  virtual void stateCallback(const nav_msgs::OdometryConstPtr& odom_state) = 0;
  virtual void drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>&) = 0;
  virtual void drivepathCallback(const ros::MessageEvent<nav_msgs::Path>&) = 0;
  virtual void cmd_velCallback(const geometry_msgs::Twist&) = 0;
  virtual void cmd_velTeleopCallback(const geometry_msgs::Twist&) = 0;
  virtual void speedCallback(const std_msgs::Float32&) = 0;
  virtual void poseCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>&) = 0;

  virtual void stopVehicle() = 0;

  virtual void followPathGoalCallback() = 0;
  virtual void followPathPreemptCallback() = 0;

  /**
   * @brief addLeg to current tracking path
   * @param pose to be added
   * @param speed 0 indicates that the speed from the configuration is used as
   *        desired linear speed in the lower controllers, otherwise the given
   *        value is used for this purpose
   */
  //void addLeg(const geometry_msgs::PoseStamped &pose, double speed = 0.0);

  virtual bool reverseAllowed() = 0;
  virtual bool reverseForced() = 0;
  virtual bool pathToBeSmoothed(const std::deque<geometry_msgs::PoseStamped> &transformed_path, bool fixed_path) = 0;
  virtual bool createDrivepath2MapTransform(tf::StampedTransform  & transform, const nav_msgs::Path& path) = 0;

public:

  geometry_msgs::PoseStamped createPoseFromQuatAndPosition(vec3 const & position, quat const & orientation){
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = position(0);
    pose.pose.position.y = position(1);
    pose.pose.position.z = position(2);
    pose.pose.orientation.w = orientation.w();
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    return pose;
  }

};

#endif // CONTROLLER_H
