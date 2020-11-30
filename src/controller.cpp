#include <vehicle_controller/controller.h>

Controller::Controller(ros::NodeHandle& nh_)
  : state(INACTIVE), stuck(new StuckDetector)
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
  mp_.pd_params = "PdParams";

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

  follow_path_server_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>(nh, "/controller/follow_path", 0, false));

  follow_path_server_->registerGoalCallback(boost::bind(&Controller::followPathGoalCallback, this));
  follow_path_server_->registerPreemptCallback(boost::bind(&Controller::followPathPreemptCallback, this));

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
  params.param("pd_params",  mp_.pd_params, std::string("PdParams"));
  params.param("y_symmetry", mp_.y_symmetry, false);
  default_path_options_.reverse_allowed = params.param<bool>("reverse_allowed", true);
  default_path_options_.rotate_front_to_goal_pose_orientation = params.param<bool>("rotate_front_to_goal_pose_orientation", false);
  params.param<std::string>("vehicle_control_type", vehicle_control_type, "differential_steering");
  double stuck_detection_window;
  params.param("stuck_detection_window", stuck_detection_window, StuckDetector::DEFAULT_DETECTION_WINDOW);
  stuck.reset(new StuckDetector(stuck_detection_window));

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
  endPosePoublisher   = nh.advertise<geometry_msgs::PoseStamped>("end_pose", 1, true);
  drivepathPublisher  = nh.advertise<nav_msgs::Path>("drivepath", 1, true);
  pathPosePublisher   = nh.advertise<nav_msgs::Path>("smooth_path", 1, true);

  diagnosticsPublisher = params.advertise<std_msgs::Float32>("velocity_error", 1, true);
  autonomy_level_pub_ = nh.advertise<std_msgs::String>("/autonomy_level", 30);

  if (camera_control)
  {
    cameraOrientationPublisher = nh.advertise<geometry_msgs::QuaternionStamped>("camera/command", 1);
    lookatPublisher = nh.advertise<geometry_msgs::PointStamped>("camera/look_at", 1);
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
  }
  empty_path.header.frame_id = map_frame_id;


  ROS_INFO("configure");

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
  catch (tf::TransformException ex)
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
  ROS_INFO("base state callback");
  latest_odom_ = odom_state;

  if (state < DRIVETO) return;

  this->updateRobotState(*latest_odom_);

  update();
}

void Controller::drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{
  geometry_msgs::PoseStampedConstPtr goal = event.getConstMessage();

  if (follow_path_server_->isActive()){
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_server_->setPreempted(result, "drive to callback");
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
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("[vehicle_controller] %s", ex.what());
    stop();
    if (follow_path_server_->isActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::TF_LOOKUP_FAILURE;
      follow_path_server_->setAborted(result, ex.what());
    }
    return false;
  }

  start = geometry_msgs::PoseStamped();
  start.pose = robot_control_state.pose;
  addLeg(goal_transformed, speed);
  state = DRIVETO;

  ROS_INFO("[vehicle_controller] Received new goal point (x = %.2f, y = %.2f), backward = %d.",
           goal_transformed.pose.position.x, goal_transformed.pose.position.y, legs.back().backward);

  final_twist_trials = 0;
  return true;
}

void Controller::drivepathCallback(const ros::MessageEvent<nav_msgs::Path>& event)
{
  if (event.getPublisherName() == ros::this_node::getName()) return;
  nav_msgs::PathConstPtr path = event.getConstMessage();

  //publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new path");
  if (follow_path_server_->isActive()){
    ROS_INFO("Received new path while Action running, preempted.");
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_server_->setPreempted(result, "drive path callback");
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
  const move_base_lite_msgs::FollowPathOptions& options = follow_path_goal_->follow_path_options;

  if (!latest_odom_.get()){
    ROS_ERROR("No latest odom message received, aborting path planning in drivepath!");
    stop();
    if (follow_path_server_->isActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::FAILURE;
      follow_path_server_->setAborted(result, "no odom message received");
    }
    return false;
  }

  this->updateRobotState(*latest_odom_);

  if (path.poses.size() == 0)
  {
    ROS_WARN("[vehicle_controller] Received empty path");
    stop();
    if (follow_path_server_->isActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
      follow_path_server_->setSucceeded(result, "empty path received, automatic success.");
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
  endPosePoublisher.publish(ptbp);

  // If path is too short, drive directly to last point
  if (map_path.size() <= 2) {
    driveto(map_path.back(), options.desired_speed);
  }

  start = map_path[0];
  start.pose.orientation = robot_control_state.pose.orientation;

  if(!options.is_fixed)
  {
    ROS_DEBUG("[vehicle_controller] Using PathSmoother.");
    Pathsmoother3D ps3d(reverseAllowed(), &mp_);

    quat in_start_orientation;
    quat in_end_orientation;

    deque_vec3 in_path;
    std::transform(map_path.begin(), map_path.end(), std::back_inserter(in_path),
                   [](geometry_msgs::PoseStamped const & pose_)
    { return vec3(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z); });

    in_start_orientation = geomQuat2EigenQuat(robot_control_state.pose.orientation);
    in_end_orientation   = geomQuat2EigenQuat(map_path.back().pose.orientation);

    vector_vec3 out_smoothed_positions;
    vector_quat out_smoothed_orientations;
    ps3d.smooth(in_path, in_start_orientation, in_end_orientation,
                out_smoothed_positions, out_smoothed_orientations, reverseAllowed());

    std::vector<geometry_msgs::PoseStamped> smooth_path;
    std::transform(out_smoothed_positions.begin(), out_smoothed_positions.end(),
                   out_smoothed_orientations.begin(), std::back_inserter(smooth_path),
                   boost::bind(&Controller::createPoseFromQuatAndPosition, this, _1, _2));

    std::for_each(smooth_path.begin() + 1, smooth_path.end(), boost::bind(&Controller::addLeg, this, _1, options.desired_speed));

    nav_msgs::Path path2publish;
    path2publish.header.frame_id = map_frame_id;
    path2publish.header.stamp = ros::Time::now();
    std::transform(smooth_path.begin(), smooth_path.end(), std::back_inserter(path2publish.poses),
                   [path2publish](geometry_msgs::PoseStamped const & pose)
    {
      geometry_msgs::PoseStamped ps;
      ps.header = path2publish.header;
      ps.pose = pose.pose;
      return ps;
    });
    pathPosePublisher.publish(path2publish);

    current_path = path2publish;
  }
  else
  {
    for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = map_path.begin()+1; it != map_path.end(); ++it)
    {
      const geometry_msgs::PoseStamped& waypoint = *it;
      addLeg(waypoint, options.desired_speed);
    }
    nav_msgs::Path path;
    path.header.frame_id = map_frame_id;
    path.header.stamp = ros::Time::now();
    path.poses = map_path;
    pathPosePublisher.publish(path);
  }

  state = DRIVEPATH;
  if(!legs.empty())
    ROS_INFO("[vehicle_controller] Received new path to goal point (x = %.2f, y = %.2f)", legs.back().p2.x, legs.back().p2.y);
  else
    ROS_WARN("[vehicle_controller] Controller::drivepath produced empty legs array.");

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
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[vehicle_controller] Drivepath transformation to map frame failed: "
                "%s", ex.what());
      stop();
      //publishActionResult(actionlib_msgs::GoalStatus::REJECTED);
      if (follow_path_server_->isActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::TF_LOOKUP_FAILURE;
        follow_path_server_->setAborted(result, ex.what());
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
  if (follow_path_server_->isActive()){
    ROS_INFO("Direct cmd_vel received, preempting running Action!");
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_server_->setPreempted(result, "cmd_vel callback");
  }
  reset();
  state = ((velocity.linear.x == 0.0) && (velocity.angular.z == 0.0)) ? INACTIVE : VELOCITY;
  vehicle_control_interface_->executeTwist(velocity);
}

void Controller::cmd_velTeleopCallback(const geometry_msgs::Twist& velocity)
{
  if (follow_path_server_->isActive()){
    ROS_INFO("Direct teleop cmd_vel received, preempting running Action!");
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_server_->setPreempted(result, "cmd_vel teleop callback");
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
  vehicle_control_interface_->executeMotionCommand(robot_control_state);
}

void Controller::followPathGoalCallback()
{
  ROS_INFO("Received Goal");
  ROS_INFO("%f", follow_path_server_->isActive());
  follow_path_goal_ = follow_path_server_->acceptNewGoal();
  if (follow_path_goal_->follow_path_options.reset_stuck_history)
  {
    stuck->reset();
  }
  current_path = follow_path_goal_->target_path;
  drivepath(follow_path_goal_->target_path);
  drivepathPublisher.publish(follow_path_goal_->target_path);
}

void Controller::followPathPreemptCallback()
{
  stopVehicle();
  move_base_lite_msgs::FollowPathResult result;
  result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
  follow_path_server_->setPreempted(result, "preempt from incoming message to server");
  reset();
}

void Controller::addLeg(const geometry_msgs::PoseStamped& pose, double speed)
{
  Leg leg;
  leg.finish_time = pose.header.stamp;
  double angles[3];

  leg.p2.x = pose.pose.position.x;
  leg.p2.y = pose.pose.position.y;

  if (legs.size() == 0)
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
    ros::Duration dt = leg.finish_time - leg.start_time;
    double dt_s = dt.toSec();
    if (dt_s > 0) {
      leg.speed = leg.length / dt_s;
    } else {
      ROS_WARN_STREAM("Waypoint time is not monotonic. Can't compute speed.");
      leg.speed = mp_.commanded_speed;
    }

  } else {
    leg.speed = (speed != 0)? speed : mp_.commanded_speed;
  }

  leg.percent = 0.0f;

  ROS_DEBUG_STREAM("Leg " << legs.size() << ": [" << leg.p1.x << ", " << leg.p1.y << "] -> [" << leg.p2.x << ", " << leg.p2.y << "], Length: "
                   << leg.length << ", Backward: " << leg.backward);

  if (leg.length2 == 0.0f) return;
  legs.push_back(leg);
}

bool Controller::reverseAllowed()
{
  // Driving backward is always allowed if vehicle is symmetric
  if (mp_.isYSymmetric()) {
    return true;
  }
  // If not, check for path specific settings
  if (follow_path_server_->isActive()) {
    return follow_path_goal_->follow_path_options.reverse_allowed;
  } else {
    // Return default
    return default_path_options_.reverse_allowed;
  }
}

bool Controller::reverseForced()
{
  return false;
  if (follow_path_server_->isActive()) {
    return follow_path_goal_->follow_path_options.reverse_forced;
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
