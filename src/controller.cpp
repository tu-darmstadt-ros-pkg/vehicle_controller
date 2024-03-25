#include <vehicle_controller/controller.h>

#include <std_msgs/Float64.h>
#include <eigen_conversions/eigen_msg.h>

Controller::Controller(ros::NodeHandle& nh_)
  : state(INACTIVE), stuck(new StuckDetector(nh_))
{
  nh = nh_;

  mp_.carrot_distance = 1.0;
  mp_.min_speed       = 0.0;
  mp_.commanded_speed = 0.0;
  mp_.max_controller_speed = 0.25;
  mp_.max_unlimited_speed = 2.0;
  mp_.max_unlimited_angular_rate = 1.0;
  mp_.max_controller_angular_rate =  0.4;
  mp_.inclination_speed_reduction_factor = 0.5 / (30 * M_PI/180.0); // 0.5 per 30 degrees
  mp_.inclination_speed_reduction_time_constant = 0.3;

  map_frame_id = "nav";
  base_frame_id = "base_link";

  final_twist_trials = 0;

  camera_control = false;
  camera_lookat_distance = 1.0;
  camera_lookat_height = -0.2;

  check_stuck = true;

  mp_.current_inclination = 0.0;
  velocity_error = 0.0;

  cameraDefaultOrientation.header.frame_id = "base_stabilized";
  tf::Quaternion cameraOrientationQuaternion;
  cameraOrientationQuaternion.setEuler(0.0, 25.0*M_PI/180.0, 0.0);
  tf::quaternionTFToMsg(cameraOrientationQuaternion, cameraDefaultOrientation.quaternion);

  follow_path_server_.reset(new actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>(nh, "/controller/follow_path",
                                                                                               boost::bind(&Controller::followPathGoalCallback, this, _1),
                                                                                               boost::bind(&Controller::followPathPreemptCallback, this, _1),
                                                                                               false));
  follow_path_server_->start();
}

Controller::~Controller()
{
}

bool Controller::configure()
{
  ros::NodeHandle params("~");
  params.getParam("carrot_distance", mp_.carrot_distance);
  params.getParam("min_speed", mp_.min_speed);
  params.getParam("frame_id", map_frame_id);
  params.getParam("base_frame_id", base_frame_id);
  params.getParam("camera_control", camera_control);
  params.getParam("camera_lookat_distance", camera_lookat_distance);
  params.getParam("camera_lookat_height", camera_lookat_height);
  params.getParam("check_stuck", check_stuck);
  params.getParam("inclination_speed_reduction_factor", mp_.inclination_speed_reduction_factor);
  params.getParam("inclination_speed_reduction_time_constant", mp_.inclination_speed_reduction_time_constant);
  params.param("goal_position_tolerance", default_path_options_.goal_pose_position_tolerance, 0.0);
  params.param("goal_angle_tolerance", default_path_options_.goal_pose_angle_tolerance, 0.0);
  params.getParam("speed",   mp_.commanded_speed);
  params.param("y_symmetry", mp_.y_symmetry, false);
  default_path_options_.reverse_allowed = params.param<bool>("reverse_allowed", true);
  default_path_options_.rotate_front_to_goal_pose_orientation = params.param<bool>("rotate_front_to_goal_pose_orientation", false);
  params.param<std::string>("vehicle_control_type", vehicle_control_type, "differential_steering");
  double stuck_detection_window;
  params.param("stuck_detection_window", stuck_detection_window, StuckDetector::DEFAULT_DETECTION_WINDOW);
  stuck = std::make_unique<StuckDetector>(nh, stuck_detection_window);

  params.param("max_controller_speed", mp_.max_controller_speed, 0.6);
  params.getParam("max_controller_angular_rate", mp_.max_controller_angular_rate);

  if (vehicle_control_type == "differential_steering")
    vehicle_control_interface_.reset(new DifferentialDriveController());
  else
    vehicle_control_interface_.reset(new FourWheelSteerController());
  vehicle_control_interface_->configure(params, &mp_);

  ROS_INFO("[vehicle_controller] Low level vehicle motion controller is %s", this->vehicle_control_interface_->getName().c_str());

  stateSubscriber     = nh.subscribe("state", 10, &Controller::stateCallback, this, ros::TransportHints().tcpNoDelay(true));
  drivetoSubscriber   = nh.subscribe("driveto", 10, &Controller::drivetoCallback, this);
  drivepathSubscriber = nh.subscribe("drivepath", 10, &Controller::drivepathCallback, this);
  cmd_velSubscriber   = nh.subscribe("cmd_vel", 10, &Controller::cmd_velCallback, this, ros::TransportHints().tcpNoDelay(true));
  cmd_velTeleopSubscriber = nh.subscribe("cmd_vel_teleop", 10, &Controller::cmd_velTeleopCallback, this, ros::TransportHints().tcpNoDelay(true));
  speedSubscriber     = nh.subscribe("speed", 10, &Controller::speedCallback, this);
  poseSubscriber      = nh.subscribe("robot_pose", 10, &Controller::poseCallback, this);

  carrotPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("carrot", 1, true);
  endPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("end_pose", 1, true);
  drivepathPublisher  = nh.advertise<nav_msgs::Path>("drivepath", 1, true);
  smoothPathPublisher   = nh.advertise<nav_msgs::Path>("smooth_path", 1, true);
  speedPublisher = params.advertise<std_msgs::Float64>("desired_speed", 10, false);

  diagnosticsPublisher = params.advertise<std_msgs::Float32>("velocity_error", 1, true);
  autonomy_level_pub_ = nh.advertise<std_msgs::String>("/autonomy_level", 30);

  if (camera_control)
  {
    cameraOrientationPublisher = nh.advertise<geometry_msgs::QuaternionStamped>("camera/command", 1);
    lookatPublisher = nh.advertise<geometry_msgs::PointStamped>("camera/look_at", 1);
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
  }
  empty_path.header.frame_id = map_frame_id;

  return true;
}

bool Controller::updateRobotState(const nav_msgs::Odometry& odom_state)
{
  dt = (odom_state.header.stamp - robot_state_header.stamp).toSec();

  if (dt < 0.0 || dt > 0.2){
    ROS_INFO("[vehicle_controller] dt between old robot state and new is %f seconds, setting to 0.0 for this iteration", dt);
    dt = 0.0;
  }

  geometry_msgs::PoseStamped pose;
  geometry_msgs::Vector3Stamped velocity_linear;
  geometry_msgs::Vector3Stamped velocity_angular;
  pose.header = odom_state.header;
  pose.pose = odom_state.pose.pose;
  velocity_linear.header = odom_state.header;
  velocity_linear.vector = odom_state.twist.twist.linear;
  velocity_angular.header = odom_state.header;
  velocity_angular.vector = odom_state.twist.twist.angular;

  try
  {
    listener.waitForTransform(map_frame_id, odom_state.header.frame_id, odom_state.header.stamp, ros::Duration(3.0));
    listener.waitForTransform(base_frame_id, odom_state.header.frame_id, odom_state.header.stamp, ros::Duration(3.0));
    listener.transformPose(map_frame_id, pose, pose);
    listener.transformVector(base_frame_id, velocity_linear, velocity_linear);
    listener.transformVector(base_frame_id, velocity_angular, velocity_angular);

    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    robot_state_header = odom_state.header;
    robot_control_state.setRobotState(velocity_linear.vector, velocity_angular.vector, pose.pose, dt);
    robot_control_state.clearControlState();
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  double inclination = acos(pose.pose.orientation.w * pose.pose.orientation.w - pose.pose.orientation.x * pose.pose.orientation.x
                            - pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z);
  if (inclination >= mp_.current_inclination)
    mp_.current_inclination = inclination;
  else
    mp_.current_inclination = (mp_.inclination_speed_reduction_time_constant * mp_.current_inclination + dt * inclination)
        / (mp_.inclination_speed_reduction_time_constant + dt);

  return true;

}

void Controller::poseCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{

  geometry_msgs::PoseStampedConstPtr pose = event.getConstMessage();
  current_pose = *pose;

}

void Controller::stateCallback(const nav_msgs::OdometryConstPtr& odom_state)
{
  latest_odom_ = odom_state;

  if (state < DRIVETO) return;

  this->updateRobotState(*latest_odom_);

  update();
}

void Controller::drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{
  const geometry_msgs::PoseStampedConstPtr& goal = event.getConstMessage();

  if (followPathServerIsActive()){
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_goal_.setCanceled(result, "Cancelled by drivetoCallback");
  }
  driveto(*goal, 0.0);
}

bool Controller::driveto(const geometry_msgs::PoseStamped& goal, double speed)
{
  reset();

  geometry_msgs::PoseStamped goal_transformed;
  try
  {
    listener.waitForTransform(map_frame_id, goal.header.frame_id, goal.header.stamp, ros::Duration(3.0));
    listener.transformPose(map_frame_id, goal, goal_transformed);
    goal_transformed.header.stamp = goal.header.stamp; // Assign original stamp
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("[vehicle_controller] %s", ex.what());
    stop();
    if (followPathServerIsActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::TF_LOOKUP_FAILURE;
      follow_path_goal_.setCanceled(result, ex.what());
    }
    return false;
  }

  start = geometry_msgs::PoseStamped();
  start.pose = robot_control_state.pose;
  addLeg(goal_transformed, speed);
  removeLegsInThePast();
  state = DRIVETO;

  ROS_INFO("[vehicle_controller] Received new goal point (x = %.2f, y = %.2f), backward = %d.",
           goal_transformed.pose.position.x, goal_transformed.pose.position.y, legs.back().backward);

  nav_msgs::Path map_path_msg;
  map_path_msg.header.frame_id = map_frame_id;
  map_path_msg.header.stamp = ros::Time::now();
  map_path_msg.poses = {start, goal};
  smoothPathPublisher.publish(map_path_msg);

  final_twist_trials = 0;
  return true;
}

void Controller::drivepathCallback(const ros::MessageEvent<nav_msgs::Path>& event)
{
  if (event.getPublisherName() == ros::this_node::getName()) return;
  const nav_msgs::PathConstPtr& path = event.getConstMessage();

  //publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new path");
  if (followPathServerIsActive()){
    ROS_INFO("Received new path while Action running, preempted.");
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_goal_.setCanceled(result, "drive path callback");
    ROS_INFO("Preempted, new path. desired vel: %f", robot_control_state.desired_velocity_linear);
  }
  drivepath(*path);
}

bool Controller::pathToBeSmoothed(const std::deque<geometry_msgs::PoseStamped>& transformed_path, bool fixed_path)
{
  // Check if this path shall be smoothed
  // <=> path has length >= 2 and path is output of exploration planner
  //     and hence, the points lie in the centers of neighbouring grid cells.
  // Note: This function not robust to changes in the exploration planner

  //    if(transformed_path.size() < 2 || fixed_path)
  //        return false;

  //    bool path_to_be_smoothed = transformed_path.size() > 2;

  //    return path_to_be_smoothed;

  return transformed_path.size() > 2 && ! fixed_path;

  //    double lu = 0.05 - 1e-5;
  //    double lo = std::sqrt(2.0) * 0.05 + 1e-5;
  //    std::stringstream sstr;
  //    for(unsigned i = 1; path_to_be_smoothed && i < transformed_path.size() - 1; ++i)
  //    {
  //        double d = euclideanDistance(transformed_path[i].position,transformed_path[i - 1].position);
  //        path_to_be_smoothed = path_to_be_smoothed && lu < d && d < lo;
  //        sstr << " " << d << ":" << (lu < d && d < lo ? "T" : "F");
  //    }
  //    return path_to_be_smoothed;
}

bool Controller::drivepath(const nav_msgs::Path& path)
{
  reset();

  double desired_speed = default_path_options_.desired_speed;
  bool is_fixed = default_path_options_.is_fixed;

  if (followPathServerIsActive()){
    const move_base_lite_msgs::FollowPathOptions& options = follow_path_goal_.getGoal()->follow_path_options;
    desired_speed = options.desired_speed;
    is_fixed = options.is_fixed;
  }

  if (!latest_odom_.get()){
    ROS_ERROR("No latest odom message received, aborting path planning in drivepath!");
    stop();
    if (followPathServerIsActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::FAILURE;
      follow_path_goal_.setCanceled(result, "no odom message received");
    }
    return false;
  }

  this->updateRobotState(*latest_odom_);

  if (path.poses.empty())
  {
    ROS_WARN("[vehicle_controller] Received empty path");
    stop();
    if (followPathServerIsActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
      follow_path_goal_.setSucceeded(result, "empty path received, automatic success.");
    }
    return false;
  }

  tf::StampedTransform transform;
  if(!createDrivepath2MapTransform(transform, path))
    return false;

  std::vector<geometry_msgs::PoseStamped> map_path(path.poses.size());
  std::transform(path.poses.begin(), path.poses.end(), map_path.begin(),
                 [&transform, this](geometry_msgs::PoseStamped const & ps)
  {
    geometry_msgs::PoseStamped transformed_waypoint;
    transformed_waypoint.header.stamp = ps.header.stamp;
    transformed_waypoint.header.frame_id = this->map_frame_id;

    tf::Pose tf_waypoint;
    tf::poseMsgToTF(ps.pose, tf_waypoint);
    tf_waypoint = transform * tf_waypoint;
    tf::poseTFToMsg(tf_waypoint, transformed_waypoint.pose);

    return transformed_waypoint;
  });

  // Publish end pose
  geometry_msgs::PoseStamped ptbp;
  ptbp.header.stamp = ros::Time::now();
  ptbp.header.frame_id = map_frame_id;
  ptbp.pose = map_path.back().pose;
  endPosePublisher.publish(ptbp);

  // If path is too short, drive directly to last point
  if (map_path.size() <= 2) {
    return driveto(map_path.back(), desired_speed);
  }

  start = map_path[0];
  start.pose.orientation = robot_control_state.pose.orientation;

  if(!is_fixed)
  {
    ROS_DEBUG("[vehicle_controller] Using PathSmoother.");
    Pathsmoother3D ps3d(reverseAllowed(), false);

    Eigen::Quaterniond in_start_orientation;
    Eigen::Quaterniond in_end_orientation;

    std::deque<Eigen::Vector3d> in_path;
    std::transform(map_path.begin(), map_path.end(), std::back_inserter(in_path),
                   [](geometry_msgs::PoseStamped const & pose_)
                   { return Eigen::Vector3d(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z); });

    in_start_orientation = geomQuat2EigenQuat(robot_control_state.pose.orientation);
    in_end_orientation   = geomQuat2EigenQuat(map_path.back().pose.orientation);

    std::vector<Eigen::Vector3d> out_smoothed_positions;
    std::vector<Eigen::Quaterniond> out_smoothed_orientations;
    ps3d.smooth(in_path, in_start_orientation, in_end_orientation,
                out_smoothed_positions, out_smoothed_orientations, reverseAllowed());

    std::vector<geometry_msgs::PoseStamped> smooth_path;
    std::transform(out_smoothed_positions.begin(), out_smoothed_positions.end(),
                   out_smoothed_orientations.begin(), std::back_inserter(smooth_path),
                   boost::bind(&createPoseFromQuatAndPosition, _1, _2));

    std::for_each(smooth_path.begin() + 1, smooth_path.end(), boost::bind(&Controller::addLeg, this, _1, desired_speed));
    nav_msgs::Path smooth_path_msg;
    smooth_path_msg.header.frame_id = map_frame_id;
    smooth_path_msg.header.stamp = ros::Time::now();
    std::transform(smooth_path.begin(), smooth_path.end(), std::back_inserter(smooth_path_msg.poses),
                   [smooth_path_msg](geometry_msgs::PoseStamped const & pose)
                   {
                     geometry_msgs::PoseStamped ps;
                     ps.header = smooth_path_msg.header;
                     ps.pose = pose.pose;
                     return ps;
                   });
    smoothPathPublisher.publish(smooth_path_msg);

    current_path = smooth_path_msg;
  }
  else
  {
    for(auto it = map_path.begin()+1; it != map_path.end(); ++it)
    {
      const geometry_msgs::PoseStamped& waypoint = *it;
      addLeg(waypoint, desired_speed);
    }
    nav_msgs::Path map_path_msg;
    map_path_msg.header.frame_id = map_frame_id;
    map_path_msg.header.stamp = ros::Time::now();
    map_path_msg.poses = map_path;
    smoothPathPublisher.publish(map_path_msg);
  }

  removeLegsInThePast();
  if(!legs.empty()) {
    ROS_INFO("[vehicle_controller] Received new path to goal point (x = %.2f, y = %.2f)", legs.back().p2.x, legs.back().p2.y);
    state = DRIVEPATH;
  }
  else {
    ROS_WARN("Controller::drivepath produced empty legs array.");
    state = INACTIVE;
    stop();
    if (followPathServerIsActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
      follow_path_goal_.setSucceeded(result, "Received empty path"); // trivially succeeded
    }
  }

  return true;
}


bool Controller::createDrivepath2MapTransform(tf::StampedTransform & transform, const nav_msgs::Path& path)
{
  if (!path.header.frame_id.empty())
  {
    try
    {
      listener.waitForTransform(this->map_frame_id, path.header.frame_id, path.header.stamp, ros::Duration(3.0));
      listener.lookupTransform(this->map_frame_id, path.header.frame_id, path.header.stamp, transform);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("[vehicle_controller] Drivepath transformation to map frame failed: "
                "%s", ex.what());
      stop();
      //publishActionResult(actionlib_msgs::GoalStatus::REJECTED);
      if (followPathServerIsActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::TF_LOOKUP_FAILURE;
        follow_path_goal_.setCanceled(result, ex.what());
      }
      return false;
    }
  }
  else
  {
    ROS_WARN("[vehicle_controller] Received a path with empty frame_id. Assuming frame = %s.", map_frame_id.c_str());
    transform.setIdentity();
  }
  return true;
}

void Controller::cmd_velCallback(const geometry_msgs::Twist& velocity)
{
  //publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a velocity command");
  if (followPathServerIsActive()){
    ROS_INFO("Direct cmd_vel received, preempting running Action!");
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_goal_.setCanceled(result, "cmd_vel callback");
  }
  reset();
  state = ((velocity.linear.x == 0.0) && (velocity.angular.z == 0.0)) ? INACTIVE : VELOCITY;
  vehicle_control_interface_->executeTwist(velocity);
}

void Controller::cmd_velTeleopCallback(const geometry_msgs::Twist& velocity)
{
  if (followPathServerIsActive()){
    ROS_INFO("Direct teleop cmd_vel received, preempting running Action!");
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_goal_.setCanceled(result, "cmd_vel teleop callback");
  }
  reset();
  state = ((velocity.linear.x == 0.0) && (velocity.angular.z == 0.0)) ? INACTIVE : VELOCITY;

  std_msgs::String autonomy_level;
  autonomy_level.data = "teleop";
  autonomy_level_pub_.publish(autonomy_level);

  vehicle_control_interface_->executeUnlimitedTwist(velocity);
}

void Controller::speedCallback(const std_msgs::Float32& speed)
{
  mp_.commanded_speed = speed.data;
}

void Controller::stopVehicle()
{
  geometry_msgs::Vector3 p;
  robot_control_state.setControlState(0.0, p, 0, 0, mp_.carrot_distance, 0.0, false, true);
  vehicle_control_interface_->stop();
}

bool Controller::followPathServerIsActive() {
  if (!follow_path_goal_.getGoal()) {
    return false;
  }
  unsigned int status = follow_path_goal_.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE ||
         status == actionlib_msgs::GoalStatus::PREEMPTING;
}

void Controller::followPathGoalCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle goal)
{
  // Check if another goal exists
  bool goal_updated = false;
  if (followPathServerIsActive()) {
    // Check if new one is newer
    if (goal.getGoalID().stamp >= follow_path_goal_.getGoalID().stamp) {
      // Abort previous goal
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      follow_path_goal_.setCanceled(result, "This goal has been preempted by a newer goal.");
      goal_updated = true;
    } else {
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      goal.setCanceled(result, "This goal has been preempted by a newer goal.");
      return;
    }
  }

  // Accept new goal
  goal.setAccepted();
  follow_path_goal_ = goal;
  // Reset if requested or if we received a new goal (not an updated goal)
  if (!goal_updated || follow_path_goal_.getGoal()->follow_path_options.reset_stuck_history)
  {
    stuck->reset();
  }
  current_path = follow_path_goal_.getGoal()->target_path;
  drivepath(follow_path_goal_.getGoal()->target_path);
  drivepathPublisher.publish(follow_path_goal_.getGoal()->target_path);
}

void Controller::followPathPreemptCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle preempt)
{
  if (preempt == follow_path_goal_) {
    stopVehicle();
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_goal_.setCanceled(result, "preempt from incoming message to server");
    reset();
    ROS_INFO_STREAM("Goal preempted, stopping vehicle");
  } else {
    ROS_WARN_STREAM("Received preempt for unknown goal");
  }

}

void Controller::addLeg(const geometry_msgs::PoseStamped& pose, double speed)
{
  Leg leg;
  leg.finish_time = pose.header.stamp;
  double angles[3];

  leg.p2.x = pose.pose.position.x;
  leg.p2.y = pose.pose.position.y;

  if (legs.empty())
  {
    leg.start_time = start.header.stamp; // start time is goal time of start state
    leg.p1.x = start.pose.position.x;
    leg.p1.y = start.pose.position.y;
    leg.course = atan2(leg.p2.y - leg.p1.y, leg.p2.x - leg.p1.x);

    if (start.pose.orientation.w == 0.0 && start.pose.orientation.x == 0.0
        && start.pose.orientation.y == 0.0 && start.pose.orientation.z == 0.0)
    {
      leg.p1.orientation = leg.course;
    }
    else
    {
      quaternion2angles(start.pose.orientation, angles);
      leg.p1.orientation = angles[0];
    }
  }
  else
  {
    const Leg& last = legs.back();
    leg.start_time = last.finish_time; // start after last leg finished
    leg.p1.x = last.p2.x;
    leg.p1.y = last.p2.y;
    leg.p1.orientation = last.p2.orientation;
    leg.course = atan2(leg.p2.y - leg.p1.y, leg.p2.x - leg.p1.x);
  }

  if (reverseAllowed()) {
    leg.backward = fabs(constrainAngle_mpi_pi(leg.course - leg.p1.orientation)) > M_PI_2;
  } else {
    leg.backward = false;
  }
  if (pose.pose.orientation.w == 0.0 && pose.pose.orientation.x == 0.0
      && pose.pose.orientation.y == 0.0 && pose.pose.orientation.z == 0.0)
  {
    leg.p2.orientation = leg.backward ? angularNorm(leg.course + M_PI) : leg.course;
  }
  else
  {
    quaternion2angles(pose.pose.orientation, angles);
    leg.p2.orientation = angles[0];
  }

  leg.length2 = std::pow(leg.p2.x - leg.p1.x, 2) + std::pow(leg.p2.y - leg.p1.y, 2);
  leg.length  = std::sqrt(leg.length2);

  if (leg.finish_time != ros::Time(0)) {
    ros::Duration time_diff = leg.finish_time - leg.start_time;
    double time_diff_s = time_diff.toSec();
    if (time_diff_s > 0) {
      leg.speed = leg.length / time_diff_s;
    } else {
      ROS_WARN_STREAM("Waypoint time is not monotonic (-" << time_diff_s << " s). Can't compute speed.");
      leg.speed = mp_.commanded_speed;
    }

  } else {
    leg.speed = (speed != 0)? speed : mp_.commanded_speed;
  }

  leg.percent = 0.0f;

  ROS_DEBUG_STREAM("Leg " << legs.size() << ": [" << leg.p1.x << ", " << leg.p1.y << "] -> [" << leg.p2.x << ", " << leg.p2.y << "], Length: "
                   << leg.length << ", Speed: " << leg.speed << ", Backward: " << leg.backward);

  if (leg.length2 == 0.0f) return;

  legs.push_back(leg);
}

void Controller::removeLegsInThePast() {
  ros::Time now = ros::Time::now();
  Legs legs_copy = legs;
  legs.clear();
  legs.reserve(legs_copy.size());
  std::copy_if (legs_copy.begin(), legs_copy.end(), std::back_inserter(legs),
               [now](const Leg& leg) {
                 return leg.finish_time.toSec() == 0.0 || leg.finish_time > now;
               }
               );
  size_t discarded_legs = legs_copy.size() - legs.size();
  if (discarded_legs > 0)
    ROS_WARN_STREAM("Dropping the first " << discarded_legs << " legs out of " << legs_copy.size() << " because their finish time is in the past.");
}


bool Controller::reverseAllowed()
{
  // Driving backward is always allowed if vehicle is symmetric
  if (mp_.isYSymmetric()) {
    return true;
  }
  // If not, check for path specific settings
  if (followPathServerIsActive()) {
    return follow_path_goal_.getGoal()->follow_path_options.reverse_allowed;
  } else {
    // Return default
    return default_path_options_.reverse_allowed;
  }
}

bool Controller::usePathOrientation() {
  if (followPathServerIsActive()) {
    return follow_path_goal_.getGoal()->follow_path_options.use_path_orientation;
  } else {
    // Return default
    return default_path_options_.use_path_orientation;
  }
}

bool Controller::reverseForced()
{
  return false;
  if (followPathServerIsActive()) {
    return follow_path_goal_.getGoal()->follow_path_options.reverse_forced;
  } else {
    return false;
  }


}

void Controller::reset()
{
  state = INACTIVE;
  current = 0;
  final_twist_trials = 0;
  dt = 0.0;
  legs.clear();
}


void Controller::stop()
{
  this->vehicle_control_interface_->stop();
  stuck->reset();
  drivepathPublisher.publish(empty_path);
  if (camera_control)
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
}

void Controller::update()
{
  if (state < DRIVETO) return;

  // get current orientation
  double angles[3];
  quaternion2angles(robot_control_state.pose.orientation, angles);

  double linear_tolerance_for_current_path = default_path_options_.goal_pose_position_tolerance;
  double angular_tolerance_for_current_path = default_path_options_.goal_pose_angle_tolerance;
  bool rotate_front_to_goal_pose_orientation = default_path_options_.rotate_front_to_goal_pose_orientation;

  if (followPathServerIsActive()){
    if (follow_path_goal_.getGoal()->follow_path_options.goal_pose_position_tolerance > 0.0){
      linear_tolerance_for_current_path = follow_path_goal_.getGoal()->follow_path_options.goal_pose_position_tolerance;
    }

    if (follow_path_goal_.getGoal()->follow_path_options.goal_pose_angle_tolerance > 0.0){
      angular_tolerance_for_current_path = follow_path_goal_.getGoal()->follow_path_options.goal_pose_angle_tolerance;
    }
    rotate_front_to_goal_pose_orientation = follow_path_goal_.getGoal()->follow_path_options.rotate_front_to_goal_pose_orientation;
  }

  // Check if goal has been reached based on goal_position_tolerance/goal_angle_tolerance
  double goal_position_error =
      std::sqrt(
        std::pow(legs.back().p2.x - robot_control_state.pose.position.x, 2)
        + std::pow(legs.back().p2.y - robot_control_state.pose.position.y, 2));
  double goal_angle_error   = angularNorm(legs.back().p2.orientation - angles[0]);

  if (goal_position_error < linear_tolerance_for_current_path)
  {   // Reached goal point. This task is handled in the following loop
    ROS_INFO_THROTTLE(1.0, "[vehicle_controller] Current position is within goal tolerance.");
    current = legs.size();
  }

  while(true)
  {
    if (current == legs.size())
    {
      goal_angle_error = constrainAngle_mpi_pi(goal_angle_error);
      // ROS_INFO("[vehicle_controller] Reached goal point position. Angular error = %f, tol = %f", goal_angle_error_ * 180.0 / M_PI, angular_tolerance_for_current_path * 180.0 / M_PI);
      if (final_twist_trials > mp_.final_twist_trials_max
          || vehicle_control_interface_->hasReachedFinalOrientation(goal_angle_error,
                                                                    angular_tolerance_for_current_path, !rotate_front_to_goal_pose_orientation)
          || !mp_.use_final_twist)
      {
        state = INACTIVE;
        ROS_INFO("[vehicle_controller] Finished orientation correction!"
                 " error = %f, tol = %f, trials = [%d, %d]",
                 goal_angle_error,
                 angular_tolerance_for_current_path,
                 final_twist_trials, mp_.final_twist_trials_max);
        final_twist_trials = 0;
        stop();
        if (followPathServerIsActive()){
          move_base_lite_msgs::FollowPathResult result;
          result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
          follow_path_goal_.setSucceeded(result, "Reached goal");
        }
        return;
      }
      else // Perform twist_ at end of path to obtain a desired orientation
      {
        final_twist_trials++;
        ROS_DEBUG("[vehicle_controller] Performing final twist_.");

        geometry_msgs::Vector3 desired_position;
        desired_position.x = legs.back().p2.x;
        desired_position.y = legs.back().p2.y;

        if(!rotate_front_to_goal_pose_orientation)
        {
          if(goal_angle_error > M_PI_2)
            goal_angle_error = goal_angle_error - M_PI;
          if(goal_angle_error < -M_PI_2)
            goal_angle_error = M_PI + goal_angle_error;
        }

        robot_control_state.setControlState(0.0,
                                            desired_position,
                                            goal_angle_error,
                                            goal_angle_error,
                                            mp_.carrot_distance,
                                            0.0,
                                            true,
                                            !rotate_front_to_goal_pose_orientation);

        vehicle_control_interface_->executeMotionCommand(robot_control_state);
        return;
      }
    }

    if (legs[current].start_time != ros::Time(0) && legs[current].finish_time != ros::Time(0)) {
      double leg_time = (legs[current].finish_time - legs[current].start_time).toSec();
      double time_in_leg;
      if (latest_odom_->header.stamp >= legs[current].start_time) {
        time_in_leg = (latest_odom_->header.stamp - legs[current].start_time).toSec();
      } else {
        time_in_leg = -1.0 * (legs[current].start_time - latest_odom_->header.stamp).toSec();
      }
      legs[current].percent = time_in_leg / leg_time;
//      ROS_INFO_STREAM("Start time: " << legs[current].start_time.toSec() << ", finish time: " << legs[current].finish_time.toSec() << ", odom time: " << latest_odom_->header.stamp.toSec() << ", percent: " << legs[current].percent);
    } else {
      legs[current].percent =
          (  (robot_control_state.pose.position.x - legs[current].p1.x)
               * (legs[current].p2.x - legs[current].p1.x)
           + (robot_control_state.pose.position.y - legs[current].p1.y)
                 * (legs[current].p2.y - legs[current].p1.y))
          / legs[current].length2;
    }

    ROS_DEBUG("[vehicle_controller] Robot has passed %.1f percent of leg %u.", legs[current].percent, current);
    if (legs[current].percent < 1.0) break;

    ++current;
    ROS_DEBUG("[vehicle_controller] Robot reached waypoint %d", current);
  }

  // calculate carrot
  Point carrot{};
  unsigned int carrot_waypoint = current;
  double carrot_percent = legs[current].percent;
  double carrot_remaining = mp_.carrot_distance;

  while(carrot_waypoint < legs.size())
  {
    if (carrot_remaining <= (1.0f - carrot_percent) * legs[carrot_waypoint].length)
    {
      carrot_percent += carrot_remaining / legs[carrot_waypoint].length;
      ROS_DEBUG_STREAM("Current waypoint contains carrot at " << carrot_percent);
      break;
    }

    carrot_remaining -= (1.0f - carrot_percent) * legs[carrot_waypoint].length;
    if (carrot_waypoint + 1 < legs.size()
        && legs[carrot_waypoint].backward == legs[carrot_waypoint + 1].backward)
    {
      ROS_DEBUG("Carrot reached waypoint %d", carrot_waypoint);
      carrot_percent = 0.0f;
      carrot_waypoint++;
    }
    else
    {
      ROS_DEBUG("Carrot reached last waypoint or change of direction");
      carrot_percent = 1.0f + carrot_remaining / legs[carrot_waypoint].length;
      break;
    }
  }

  carrot.x           = (1.0 - carrot_percent) * legs[carrot_waypoint].p1.x + carrot_percent * legs[carrot_waypoint].p2.x;
  carrot.y           = (1.0 - carrot_percent) * legs[carrot_waypoint].p1.y + carrot_percent * legs[carrot_waypoint].p2.y;
  // carrot.orientation = legs[carrot_waypoint].p1.orientation + std::min(carrot_percent, 1.0f) * angular_norm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);

  if (carrot_waypoint == legs.size() - 1)
  {
    carrot.orientation = legs[carrot_waypoint].p1.orientation + std::min(carrot_percent, 1.0) * angularNorm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);
  }
  else
  {
    carrot.orientation = legs[carrot_waypoint].p1.orientation + /* carrot_percent * */ 1.0 * angularNorm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);
  }

  carrotPose.header = robot_state_header;
  carrotPose.pose.position.x = carrot.x;
  carrotPose.pose.position.y = carrot.y;
  double ypr[3] = { carrot.orientation, 0.0, 0.0 };
  angles2quaternion(ypr, carrotPose.pose.orientation);
  if (carrotPosePublisher)
    carrotPosePublisher.publish(carrotPose);

  // --------------------------------------------------------------------------------------------------- //
  // alpha = robot orientation                      # robot orientation in global cosy                   //
  // beta  = angle(carrot position, robot position) # path orientation in global cosy                    //
  // error_2_path   = beta - alpha                  # error of robot orientation to path orientation     //
  // error_2_carrot = carrot orientation - alpha    # error of robot orientation to carrot orientation   //
  // --------------------------------------------------------------------------------------------------- //

  geometry_msgs::Vector3 desired_position;
  desired_position.x = carrot.x;
  desired_position.y = carrot.y;
  desired_position.z = 0.0;

  double alpha = angles[0];
  double beta  = atan2(carrot.y - robot_control_state.pose.position.y,
                       carrot.x - robot_control_state.pose.position.x);

  double error_2_path   = constrainAngle_mpi_pi( beta - alpha );
  double error_2_carrot = constrainAngle_mpi_pi( carrot.orientation - alpha);
  double sign  = legs[current].backward ? -1.0 : 1.0;

  if (reverseAllowed()) {
    Eigen::Vector3d rdp(desired_position.x - robot_control_state.pose.position.x, desired_position.y - robot_control_state.pose.position.y, 0.0);
    rdp.normalize();
    Eigen::Quaterniond rq(robot_control_state.pose.orientation.w, robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y, robot_control_state.pose.orientation.z);

    Eigen::Vector3d rpath = /*rq **/ rdp;
    Eigen::Vector3d rpos = rq * Eigen::Vector3d::UnitX();
    if (reverseForced()) {
      rpos.dot(rpath);
      sign = -1.0;
    } else {
      sign = rpos.dot(rpath) >= 0.0 ? 1.0 : -1.0;
    }
  }

  // Compute speed
  ros::Time current_time = latest_odom_->header.stamp;
  double speed = sign * legs[current].speed;
  publishDouble(speedPublisher, speed);
  if (legs[current].start_time != ros::Time(0) && legs[current].finish_time != ros::Time(0)) {
    Eigen::Isometry3d robot_pose(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ()));
    robot_pose.translation().x() = robot_control_state.pose.position.x;
    robot_pose.translation().y() = robot_control_state.pose.position.y;
    robot_pose.translation().z() = 0;
//    tf::poseMsgToEigen(robot_control_state.pose, robot_pose);

    Eigen::Vector3d dpos;
    dpos.x() = legs[current].percent * legs[current].p2.x + (1.0 - legs[current].percent) * legs[current].p1.x;
    dpos.y() = legs[current].percent * legs[current].p2.y + (1.0 - legs[current].percent) * legs[current].p1.y;
    dpos.z() = 0.0;

    Eigen::Vector3d dpos_base = robot_pose.inverse() * dpos;

//    ROS_INFO_STREAM("Leg " << current << ": [" << legs[current].p1.x << ", " << legs[current].p1.y << "] -> [" << legs[current].p2.x << ", " << legs[current].p2.y << "], Length: "
//                            << legs[current].length << ", Speed: " << legs[current].speed << ", Backward: " << legs[current].backward);

    double error = dpos_base.x();
    speed += mp_.speed_p_gain * error;

//    ROS_INFO_STREAM("Current percent: " << legs[current].percent <<
//                    ", current pos: [" << robot_pose.translation().x() << ", " << robot_pose.translation().y() <<
//                    "], desired pos: [" << dpos.x() << ", " << dpos.y() <<
//                    "], base frame: [" << dpos_base.x() << ", " << dpos_base.y() <<
//                    "], error: " << error << ", org. speed: " << legs[current].speed << ", new speed: " << speed);

  }

  // Account for driving direction
//  speed *= sign;

  double signed_carrot_distance_2_robot =
      sign * euclideanDistance2D(carrotPose.pose.position,
                                 robot_control_state.pose.position);
  bool approaching_goal_point = goal_position_error < 0.4;

  if(std::abs(signed_carrot_distance_2_robot) > (mp_.carrot_distance * 1.5))
  {
    ROS_WARN("[vehicle_controller] Control failed, distance to carrot is %f (allowed: %f)", signed_carrot_distance_2_robot, (mp_.carrot_distance * 1.5));
    state = INACTIVE;
    stop();

    if (followPathServerIsActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::CONTROL_FAILED;
      follow_path_goal_.setCanceled(result, std::string("Control failed, distance between trajectory and robot too large."));
    }
    return;
  }

  //    if (state == DRIVETO && goal_position_error < 0.6 /* && mp_.isYSymmetric() */)
  //    { // TODO: Does mp_.isYSymmetric() really make sense here?
  //        if(error_2_path > M_PI_2)
  //            error_2_path = error_2_path - M_PI;
  //        if(error_2_path < -M_PI_2)
  //            error_2_path = M_PI + error_2_path;
  //    }

  if(reverseAllowed()) {
    if (!usePathOrientation()) {
      // We do not use the path orientation, check if we can drive backwards
      // If angle error to path is outside of [-pi/2; pi/2], we drive backwards
      if (error_2_path > M_PI_2)
        error_2_path = error_2_path - M_PI;
      if (error_2_path < -M_PI_2)
        error_2_path = M_PI + error_2_path;
    } else {
      // We use the driving direction encoded in the path
      // If this leg is backwards, flip angle error with pi to force backwards driving
      if (legs[current].backward) {
        if (error_2_path > 0) {
          error_2_path -= M_PI;
        } else {
          error_2_path += M_PI;
        }
      }
    }
  }

  robot_control_state.setControlState(speed,
                                      desired_position,
                                      error_2_path,
                                      error_2_carrot,
                                      mp_.carrot_distance,
                                      signed_carrot_distance_2_robot,
                                      approaching_goal_point,
                                      reverseAllowed());

  //vehicle_control_interface_->executeMotionCommand(robot_control_state);

  //Compute the move command for the robot and send it to the control interface
  computeMoveCmd();


  if (check_stuck)
  {
    //        geometry_msgs::PoseStamped ps;
    //        ps.header = robot_state_header;
    //        ps.pose   = robot_control_state.pose;
    stuck->update(current_pose, vehicle_control_interface_->getCommandedSpeed(), vehicle_control_interface_->getCommandedRotationalRate());
    if(stuck->isStuck())
    {
      ROS_WARN("[vehicle_controller] I think I am blocked! Terminating current drive goal.");
      state = INACTIVE;
      stop();
      stuck->reset();
      //publishActionResult(actionlib_msgs::GoalStatus::ABORTED,
      //                    vehicle_control_type == "differential_steering" ? "blocked_tracked" : "blocked");
      if (followPathServerIsActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
        follow_path_goal_.setCanceled(result, "I think I am blocked! Terminating current drive goal.");
      }
    }
  }

  // publish feedback
  if (followPathServerIsActive()) {
    //      ROS_INFO_STREAM_THROTTLE(1, "Current lagg: " << lagg.toSec());
    move_base_lite_msgs::FollowPathFeedback feedback;
    feedback.current_waypoint = current;
    feedback.current_percent_complete = legs[current].percent;
    //      feedback.total_percent_completer

    if (legs[current].finish_time != ros::Time(0)) {
      ros::Duration total_dt = legs[current].finish_time - legs[current].start_time;
      ros::Duration time_in_path = total_dt * legs[current].percent;
      feedback.actual_duration_since_start = time_in_path;
      ros::Duration current_time_since_start = current_time - legs[current].start_time;
      ros::Duration lagg =  current_time_since_start - time_in_path;
      feedback.current_lagg = lagg;
    }

    follow_path_goal_.publishFeedback(feedback);
  }

  // camera control
  if (camera_control)
  {
    // calculate lookat position
    Point lookat;
    unsigned int lookat_waypoint = current;
    bool found_lookat_position = false;

    while(lookat_waypoint < legs.size())
    {
      lookat.x           = legs[lookat_waypoint].p2.x;
      lookat.y           = legs[lookat_waypoint].p2.y;
      lookat.orientation = legs[lookat_waypoint].p2.orientation;

      double distance =
          std::sqrt(
            (robot_control_state.pose.position.x - lookat.x)
            * (robot_control_state.pose.position.x - lookat.x)
            +  (robot_control_state.pose.position.y - lookat.y)
            * (robot_control_state.pose.position.y - lookat.y));
      double relative_angle =
          angularNorm(atan2(lookat.y - robot_control_state.pose.position.y,
                            lookat.x - robot_control_state.pose.position.x)
                      - angles[0]);

      if (distance >= camera_lookat_distance && relative_angle >= -M_PI/2 && relative_angle <= M_PI/2) {
        found_lookat_position = true;
        break;
      }

      if (lookat_waypoint+1 < legs.size()) {
        ROS_DEBUG("lookat reached waypoint %d", lookat_waypoint);
        lookat_waypoint++;
      } else {
        ROS_DEBUG("lookat reached last waypoint");
        break;
      }
    }

    if (found_lookat_position) {
      geometry_msgs::PointStamped lookat_msg;
      lookat_msg.header  = robot_state_header;
      lookat_msg.point.x = lookat.x;
      lookat_msg.point.y = lookat.y;
      lookat_msg.point.z =   robot_control_state.pose.position.z
          + camera_lookat_height;
      lookatPublisher.publish(lookat_msg);
    } else {
      cameraOrientationPublisher.publish(cameraDefaultOrientation);
      //      lookat_msg.header.frame_id = "base_stabilized";
      //      lookat_msg.point.x = 1.0;
      //      lookat_msg.point.y = 0.0;
      //      lookat_msg.point.z = camera_lookat_height;

    }
  }
}
