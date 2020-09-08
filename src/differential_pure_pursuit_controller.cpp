#include <vehicle_controller/differential_pure_pursuit_controller.h>

Differential_Pure_Pursuit_Controller::Differential_Pure_Pursuit_Controller(ros::NodeHandle& nh_)
  : state(INACTIVE), stuck(new StuckDetector), nh_dr_params("~/controller_params")
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

  follow_path_server_->registerGoalCallback(boost::bind(&Differential_Pure_Pursuit_Controller::followPathGoalCallback, this));
  follow_path_server_->registerPreemptCallback(boost::bind(&Differential_Pure_Pursuit_Controller::followPathPreemptCallback, this));

  follow_path_server_->start();
}

Differential_Pure_Pursuit_Controller::~Differential_Pure_Pursuit_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    delete dr_controller_params_server;
  }
}

bool Differential_Pure_Pursuit_Controller::configure()
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
  params.param("vehicle_length", vehicle_length, 0.5);

  if (vehicle_control_type == "differential_steering")
    vehicle_control_interface_.reset(new DifferentialDriveController());
  else
    vehicle_control_interface_.reset(new FourWheelSteerController());
  vehicle_control_interface_->configure(params, &mp_);

  ROS_INFO("[vehicle_controller] Low level vehicle motion controller is %s", this->vehicle_control_interface_->getName().c_str());

  stateSubscriber     = nh.subscribe("state", 10, &Differential_Pure_Pursuit_Controller::stateCallback, this, ros::TransportHints().tcpNoDelay(true));
  drivetoSubscriber   = nh.subscribe("driveto", 10, &Differential_Pure_Pursuit_Controller::drivetoCallback, this);
  drivepathSubscriber = nh.subscribe("drivepath", 10, &Differential_Pure_Pursuit_Controller::drivepathCallback, this);
  cmd_velSubscriber   = nh.subscribe("cmd_vel", 10, &Differential_Pure_Pursuit_Controller::cmd_velCallback, this, ros::TransportHints().tcpNoDelay(true));
  cmd_velTeleopSubscriber = nh.subscribe("cmd_vel_teleop", 10, &Differential_Pure_Pursuit_Controller::cmd_velTeleopCallback, this, ros::TransportHints().tcpNoDelay(true));
  speedSubscriber     = nh.subscribe("speed", 10, &Differential_Pure_Pursuit_Controller::speedCallback, this);
  poseSubscriber      = nh.subscribe("robot_pose", 10, &Differential_Pure_Pursuit_Controller::poseCallback, this);

  carrotPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("carrot", 1, true);
  endPosePoublisher   = nh.advertise<geometry_msgs::PoseStamped>("end_pose", 1, true);
  drivepathPublisher  = nh.advertise<nav_msgs::Path>("drivepath", 1, true);
  pathPosePublisher   = nh.advertise<nav_msgs::Path>("smooth_path", 1, true);
  cmd_vel_pub         = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1, true);
  //cmd_vel_pub         = nh.advertise<geometry_msgs::Twist>("/affw_ctrl/target_vel", 1, true);

  diagnosticsPublisher = params.advertise<std_msgs::Float32>("velocity_error", 1, true);
  autonomy_level_pub_ = nh.advertise<std_msgs::String>("/autonomy_level", 30);

  if (camera_control)
  {
    cameraOrientationPublisher = nh.advertise<geometry_msgs::QuaternionStamped>("camera/command", 1);
    lookatPublisher = nh.advertise<geometry_msgs::PointStamped>("camera/look_at", 1);
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
  }
  empty_path.header.frame_id = map_frame_id;

  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::PurePursuitControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Differential_Pure_Pursuit_Controller::controllerParamsCallback, this, _1, _2));

  return true;
}

bool Differential_Pure_Pursuit_Controller::updateRobotState(const nav_msgs::Odometry& odom_state)
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

void Differential_Pure_Pursuit_Controller::poseCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{

  geometry_msgs::PoseStampedConstPtr pose = event.getConstMessage();
  current_pose = *pose;

}

void Differential_Pure_Pursuit_Controller::stateCallback(const nav_msgs::OdometryConstPtr& odom_state)
{
  latest_odom_ = odom_state;

  if (state < DRIVETO) return;

  this->updateRobotState(*latest_odom_);

  update();
}

void Differential_Pure_Pursuit_Controller::drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{
  geometry_msgs::PoseStampedConstPtr goal = event.getConstMessage();

  if (follow_path_server_->isActive()){
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_server_->setPreempted(result, "drive to callback");
  }
  driveto(*goal, 0.0);
}

bool Differential_Pure_Pursuit_Controller::driveto(const geometry_msgs::PoseStamped& goal, double speed)
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

void Differential_Pure_Pursuit_Controller::drivepathCallback(const ros::MessageEvent<nav_msgs::Path>& event)
{
  if (event.getPublisherName() == ros::this_node::getName()) return;
  nav_msgs::PathConstPtr path = event.getConstMessage();

  //publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new path");
  if (follow_path_server_->isActive()){
    ROS_INFO("Received new path while Action running, preempted.");
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_server_->setPreempted(result, "drive path callback");
  }
  drivepath(*path);
}

bool Differential_Pure_Pursuit_Controller::pathToBeSmoothed(const std::deque<geometry_msgs::PoseStamped>& transformed_path, bool fixed_path)
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

bool Differential_Pure_Pursuit_Controller::drivepath(const nav_msgs::Path& path)
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

    std::for_each(smooth_path.begin() + 1, smooth_path.end(), boost::bind(&Differential_Pure_Pursuit_Controller::addLeg, this, _1, options.desired_speed));

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
    ROS_WARN("[vehicle_controller] Differential_Pure_Pursuit_Controller::drivepath produced empty legs array.");

  return true;
}


bool Differential_Pure_Pursuit_Controller::createDrivepath2MapTransform(tf::StampedTransform & transform, const nav_msgs::Path& path)
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

void Differential_Pure_Pursuit_Controller::cmd_velCallback(const geometry_msgs::Twist& velocity)
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

void Differential_Pure_Pursuit_Controller::cmd_velTeleopCallback(const geometry_msgs::Twist& velocity)
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

void Differential_Pure_Pursuit_Controller::speedCallback(const std_msgs::Float32& speed)
{
  mp_.commanded_speed = speed.data;
}

void Differential_Pure_Pursuit_Controller::stopVehicle()
{
  geometry_msgs::Vector3 p;
  robot_control_state.setControlState(0.0, p, 0, 0, mp_.carrot_distance, 0.0, false, true);
  vehicle_control_interface_->executeMotionCommand(robot_control_state);
}

void Differential_Pure_Pursuit_Controller::followPathGoalCallback()
{
  ROS_INFO("Received Goal");
  ROS_INFO("%f", follow_path_server_->isActive());
  follow_path_goal_ = follow_path_server_->acceptNewGoal();
  if (follow_path_goal_->follow_path_options.reset_stuck_history)
  {
    stuck->reset();
  }
  drivepath(follow_path_goal_->target_path);
  drivepathPublisher.publish(follow_path_goal_->target_path);
}

void Differential_Pure_Pursuit_Controller::followPathPreemptCallback()
{
  stopVehicle();
  move_base_lite_msgs::FollowPathResult result;
  result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
  follow_path_server_->setPreempted(result, "preempt from incoming message to server");
  reset();
}

void Differential_Pure_Pursuit_Controller::addLeg(const geometry_msgs::PoseStamped& pose, double speed)
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

bool Differential_Pure_Pursuit_Controller::reverseAllowed()
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

bool Differential_Pure_Pursuit_Controller::reverseForced()
{
  return false;
  if (follow_path_server_->isActive()) {
    return follow_path_goal_->follow_path_options.reverse_forced;
  } else {
    return false;
  }


}

void Differential_Pure_Pursuit_Controller::reset()
{
  state = INACTIVE;
  current = 0;
  final_twist_trials = 0;
  dt = 0.0;
  legs.clear();
}

void Differential_Pure_Pursuit_Controller::update()
{
  if (state < DRIVETO) return;

  // get current orientation
  double angles[3];
  quaternion2angles(robot_control_state.pose.orientation, angles);

  double linear_tolerance_for_current_path = default_path_options_.goal_pose_position_tolerance;
  double angular_tolerance_for_current_path = default_path_options_.goal_pose_angle_tolerance;
  bool rotate_front_to_goal_pose_orientation = default_path_options_.rotate_front_to_goal_pose_orientation;

  if (follow_path_server_->isActive()){
    if (follow_path_goal_->follow_path_options.goal_pose_position_tolerance > 0.0){
      linear_tolerance_for_current_path = follow_path_goal_->follow_path_options.goal_pose_position_tolerance;
    }

    if (follow_path_goal_->follow_path_options.goal_pose_angle_tolerance > 0.0){
      angular_tolerance_for_current_path = follow_path_goal_->follow_path_options.goal_pose_angle_tolerance;
    }
    rotate_front_to_goal_pose_orientation = follow_path_goal_->follow_path_options.rotate_front_to_goal_pose_orientation;
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

  while(1)
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
                 goal_angle_error * 180.0 / M_PI,
                 angular_tolerance_for_current_path * 180.0 / M_PI,
                 final_twist_trials, mp_.final_twist_trials_max);
        final_twist_trials = 0;
        stop();
        if (follow_path_server_->isActive()){
          move_base_lite_msgs::FollowPathResult result;
          result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
          follow_path_server_->setSucceeded(result, "reached goal");
        }
        return;
      }
      else // Perform twist at end of path to obtain a desired orientation
      {
        final_twist_trials++;
        ROS_DEBUG("[vehicle_controller] Performing final twist.");

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

    legs[current].percent =
        (  (robot_control_state.pose.position.x - legs[current].p1.x)
           * (legs[current].p2.x - legs[current].p1.x)
           + (robot_control_state.pose.position.y - legs[current].p1.y)
           * (legs[current].p2.y - legs[current].p1.y))
        / legs[current].length2;

    ROS_DEBUG("[vehicle_controller] Robot has passed %.1f percent of leg %u.", legs[current].percent, current);
    if (legs[current].percent < 1.0) break;

    ++current;
    ROS_DEBUG("[vehicle_controller] Robot reached waypoint %d", current);
  }

  // calculate carrot
  Point carrot;
  unsigned int carrot_waypoint = current;
  float carrot_percent = legs[current].percent;
  float carrot_remaining = mp_.carrot_distance;

  while(carrot_waypoint < legs.size())
  {
    if (carrot_remaining <= (1.0f - carrot_percent) * legs[carrot_waypoint].length)
    {
      carrot_percent += carrot_remaining / legs[carrot_waypoint].length;
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

  carrot.x           = (1.0f - carrot_percent) * legs[carrot_waypoint].p1.x + carrot_percent * legs[carrot_waypoint].p2.x;
  carrot.y           = (1.0f - carrot_percent) * legs[carrot_waypoint].p1.y + carrot_percent * legs[carrot_waypoint].p2.y;
  // carrot.orientation = legs[carrot_waypoint].p1.orientation + std::min(carrot_percent, 1.0f) * angular_norm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);

  if (carrot_waypoint == legs.size() - 1)
  {
    carrot.orientation = legs[carrot_waypoint].p1.orientation + std::min(carrot_percent, 1.0f) * angularNorm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);
  }
  else
  {
    carrot.orientation = legs[carrot_waypoint].p1.orientation + /* carrot_percent * */ 1.0f * angularNorm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);
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
    vec3 rdp(desired_position.x - robot_control_state.pose.position.x, desired_position.y - robot_control_state.pose.position.y, 0.0);
    rdp.normalize();
    quat rq(robot_control_state.pose.orientation.w, robot_control_state.pose.orientation.x, robot_control_state.pose.orientation.y, robot_control_state.pose.orientation.z);

    vec3 rpath = /*rq **/ rdp;
    vec3 rpos = rq * vec3(1,0,0);
    if (reverseForced()) {
      rpos.dot(rpath);
      sign = -1.0;
    } else {
      sign = rpos.dot(rpath) >= 0.0 ? 1.0 : -1.0;
    }
  }

  // Compute speed
  // Check if we need to wait
  ros::Time current_time = ros::Time::now();
  double speed;
  if (current_time < legs[current].start_time) {
    ROS_DEBUG_STREAM("[vehicle_controller] Start time of waypoint " << current << " not reached yet, waiting..");
    speed = 0;
  } else {
    // if we are lagging behind the trajectory, increase speed accordingly
    if (legs[current].finish_time != ros::Time(0)) {
      ros::Duration dt = current_time - legs[current].start_time;
      double dx_des = dt.toSec() * legs[current].speed;
      double dx_cur = std::sqrt(std::pow(robot_control_state.pose.position.x - legs[current].p1.x, 2)
                                + std::pow(robot_control_state.pose.position.y - legs[current].p1.y, 2));
      double error = dx_des - dx_cur;
      speed = legs[current].speed + 1.5 * error;

    } else {
      speed = legs[current].speed;
    }
    speed = sign * speed;
  }
  //    ROS_INFO_STREAM("Speed: " << speed);

  double signed_carrot_distance_2_robot =
      sign * euclideanDistance2D(carrotPose.pose.position,
                                 robot_control_state.pose.position);
  bool approaching_goal_point = goal_position_error < 0.4;

  if(std::abs(signed_carrot_distance_2_robot) > (mp_.carrot_distance * 5))  //  CHANGED FROM 1.5 to 5
  {
    ROS_WARN("[vehicle_controller] Control failed, distance to carrot is %f (allowed: %f)", signed_carrot_distance_2_robot, (mp_.carrot_distance * 1.5));
    state = INACTIVE;
    stop();

    if (follow_path_server_->isActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::CONTROL_FAILED;
      follow_path_server_->setAborted(result, std::string("Control failed, distance between trajectory and robot too large."));
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

  if(reverseAllowed())
  {
    if(error_2_path > M_PI_2)
      error_2_path = error_2_path - M_PI;
    if(error_2_path < -M_PI_2)
      error_2_path = M_PI + error_2_path;
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

  Differential_Pure_Pursuit_Controller::computeMoveCmd(robot_control_state);


  if (check_stuck)
  {
    //        geometry_msgs::PoseStamped ps;
    //        ps.header = robot_state_header;
    //        ps.pose   = robot_control_state.pose;
    stuck->update(current_pose);
    if((*stuck)(robot_control_state.desired_velocity_linear))
    {
      ROS_WARN("[vehicle_controller] I think I am blocked! Terminating current drive goal.");
      state = INACTIVE;
      stop();
      stuck->reset();
      //publishActionResult(actionlib_msgs::GoalStatus::ABORTED,
      //                    vehicle_control_type == "differential_steering" ? "blocked_tracked" : "blocked");
      if (follow_path_server_->isActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
        follow_path_server_->setAborted(result, std::string("I think I am blocked! Terminating current drive goal."));
      }
    }
  }

  // publish feedback
  if (follow_path_server_->isActive()) {
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

    follow_path_server_->publishFeedback(feedback);
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

void Differential_Pure_Pursuit_Controller::stop()
{
  this->vehicle_control_interface_->stop();
  stuck->reset();
  drivepathPublisher.publish(empty_path);
  if (camera_control)
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
}

void Differential_Pure_Pursuit_Controller::computeMoveCmd(RobotControlState control_state){

  double dist_to_carrot = control_state.signed_carrot_distance_2_robot;

//  geometry_msgs::PoseStamped carrotPose_baseframe;
//  try
//  {
//    carrotPose.pose.position.z = robot_control_state.pose.position.z;
//    listener.waitForTransform(base_frame_id, carrotPose.header.frame_id, carrotPose.header.stamp, ros::Duration(3.0));
//    listener.transformPose(base_frame_id, carrotPose, carrotPose_baseframe);
//  }
//  catch (tf::TransformException ex)
//  {
//    ROS_ERROR("%s", ex.what());
//    return;
//  }

  double carrot_distance_y = std::sin(robot_control_state.error_2_path_angular) * dist_to_carrot;

  double curv =  2 * carrot_distance_y / (dist_to_carrot*dist_to_carrot);

  geometry_msgs::Twist cmd;
  cmd.linear.x = control_state.desired_velocity_linear;
  cmd.linear.y = 0.0;
  cmd.angular.z = curv * cmd.linear.x;

  //if(ekf_useEkf){
//    if (!ekf_setInitialPose){
//      ekf.x_(0,0) = robot_control_state.pose.position.x;
//      ekf.x_(1,0) = robot_control_state.pose.position.y;
//      ekf.x_(2,0) = yaw;

//      ekf_setInitialPose = true;
//      ekf_lastTime = ros::Time::now();
//      //ekf_lastCmd = cmd;

//      ekf_last_pitch = pitch;
//      ekf_last_roll = roll;
//      ekf_last_yaw = yaw;

//      //cmd_vel_pub.publish(cmd);
//      //vehicle_control_interface_->executeTwist(cmd);
//      ROS_INFO("initial SET");
//    }
//    else{
//      double dt = (ros::Time::now().toSec() - ekf_lastTime.toSec());

//      double l;
//      nh.getParam("/vehicle_controller/wheel_separation", l);

//      double v_lin = ekf_lastCmd.linear.x;
//      double v_ang = ekf_lastCmd.angular.z;

//      double Vl_ = v_lin - l/2 * v_ang;
//      double Vr_ = v_lin + l/2 * v_ang;

//      if(dt > 0){
//        ekf.predict(Vl_, Vr_, ekf_last_pitch, ekf_last_roll, dt);

//        Eigen::Vector3d delta;
//        delta(0) = robot_control_state.pose.position.x;
//        delta(1) = robot_control_state.pose.position.y;
//        delta(2) = yaw;
//        ekf.correct(delta);

//        double omega = -(Vl_ - Vr_)/fabs(ekf.x_(4) - ekf.x_(3))  * std::cos(ekf_last_roll) * std::cos(ekf_last_pitch);
//        //ROS_INFO("omega: %f, twist: %f, pose.twist: %f, yaw_diff: %f", omega, cmd.angular.z, robot_control_state.velocity_angular.z, (yaw-ekf_last_yaw)/dt);

//        double y_ICRr = ekf.x_(3,0);
//        double y_ICRl = ekf.x_(4,0);

////        double vl_corrected = cmd.linear.x - y_ICRl * cmd.angular.z;
////        double vr_corrected = cmd.linear.x - y_ICRr * cmd.angular.z;

////        ROS_INFO("vl: %f, vl_corrected: %f", Vl_, vl_corrected);
////        ROS_INFO("vr: %f, vr_corrected: %f", Vr_, vr_corrected);
////        ROS_INFO("vlin: %f, vlin_corrected: %f", cmd.linear.x, (vl_corrected + vr_corrected)/2);

//        //cmd.linear.x = (vl_corrected + vr_corrected)/2;
//        //cmd.angular.z = (vr_corrected - vl_corrected)/l;

//        //cmd_vel_pub.publish(cmd);
//        //vehicle_control_interface_->executeTwist(cmd);

//        ROS_INFO("yl: %f, yr: %f, x: %f",ekf.x_(4), ekf.x_(3), ekf.x_(5) );

//        double icr = (ekf.x_(4) + ekf.x_(3));
//        ROS_INFO("ICR: %f", icr);

//        //ekf_lastCmd = cmd;
//        ekf_lastTime = ros::Time::now();
//        ekf_last_pitch = pitch;
//        ekf_last_roll = roll;
//        ekf_last_yaw = yaw;
//      }
//    }
//    geometry_msgs::Pose augmented_pose = robot_control_state.pose;
//    double r_x = ekf.x_(5);
//    double r_y = (ekf.x_(3) + ekf.x_(4))/2;

//    augmented_pose.position.x += std::cos(yaw) * r_x - std::sin(yaw) * r_y;
//    augmented_pose.position.y += std::sin(yaw) * r_x + std::cos(yaw) * r_y;

//    geometry_msgs::PoseStamped augmented_carrot = carrotPose;
//    double current_waypoint_orientation = atan2(legs[current].p2.y - legs[current].p1.y,
//                                                legs[current].p2.x - legs[current].p1.x);

//    augmented_carrot.pose.position.x += std::cos(current_waypoint_orientation) * r_x - std::sin(current_waypoint_orientation) * r_y;
//    augmented_carrot.pose.position.y += std::sin(current_waypoint_orientation) * r_x + std::cos(current_waypoint_orientation) * r_y;

//    double dist_to_aug_carrot = std::sqrt(std::pow(augmented_pose.position.x - augmented_carrot.pose.position.x, 2)
//                                            + std::pow(augmented_pose.position.y - augmented_carrot.pose.position.y, 2));

//    double beta  = atan2(augmented_carrot.pose.position.y - augmented_pose.position.y,
//                         augmented_carrot.pose.position.x - augmented_pose.position.x);

//    double error_2_path   = constrainAngle_mpi_pi( beta - yaw );

//    ROS_INFO("yaw: %f, carrot_orientation: %f", yaw, current_waypoint_orientation);

//    double carrot_distance_aug_y = std::sin(error_2_path) * dist_to_aug_carrot;

//    double curv =  2 * carrot_distance_aug_y / (dist_to_aug_carrot*dist_to_aug_carrot);

//    geometry_msgs::Twist cmd;
//    cmd.linear.x = control_state.desired_velocity_linear;
//    cmd.linear.y = 0.0;
//    cmd.angular.z = curv * cmd.linear.x;

//    ekf_lastCmd = cmd;

//    }
//  }
//  else{
//    //cmd_vel_pub.publish(cmd);
//    vehicle_control_interface_->executeTwist(cmd);
//  }

   vehicle_control_interface_->executeTwist(cmd, robot_control_state, yaw, pitch, roll);

   ROS_INFO("lin vel: %f, ang vel: %f", cmd.linear.x, cmd.angular.z);
   ROS_INFO("curv: %f", curv);
}

double Differential_Pure_Pursuit_Controller::exponentialSpeedControll(){
  //Test: hadcoded kw and kg
  double k_w_ = 0.0;
  double k_g_ = 0.0;

  //compute the curvature, and stop when the look-ahead distance is reached (w.r.t. orthogonal projection)
//  double s_cum_sum = 0;
//  curv_sum_ = 0.0;

//  for (unsigned int i = proj_ind_ + 1; i < path_interpol.n(); i++){

//      s_cum_sum = path_interpol.s(i) - path_interpol.s(proj_ind_);
//      curv_sum_ += std::abs(path_interpol.curvature(i));

//      if(s_cum_sum - look_ahead_dist_ >= 0){
//          break;
//      }
//  }

  double goal_position_error =
      std::sqrt(
        std::pow(legs.back().p2.x - robot_control_state.pose.position.x, 2)
        + std::pow(legs.back().p2.y - robot_control_state.pose.position.y, 2));

  //get the robot's current angular velocity
  double angular_vel = robot_control_state.velocity_angular.z;

  //double obst_angle = 0.0;

//  double min_dist = std::numeric_limits<double>::infinity();
//  if(collision_avoider_->hasObstacles()) {
//      auto obstacle_cloud = collision_avoider_->getObstacles();
//      const pcl::PointCloud<pcl::PointXYZ>& cloud = *obstacle_cloud->cloud;
//      if(cloud.header.frame_id == "base_link" || cloud.header.frame_id == "/base_link") {
//          for(const pcl::PointXYZ& pt : cloud) {

//              if(std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) < min_dist){

//                  obst_angle = std::atan2(pt.y, pt.x);
//              }
//              min_dist = std::min<double>(min_dist, std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z));
//          }

//      } else {
//          tf::Transform trafo = pose_tracker_->getTransform(pose_tracker_->getRobotFrameId(), cloud.header.frame_id, ros::Time(0), ros::Duration(0));
//          for(const pcl::PointXYZ& pt : cloud) {

//              tf::Point pt_cloud(pt.x, pt.y, pt.z);
//              tf::Point pt_robot = trafo * pt_cloud;

//              if(pt_robot.length() < min_dist){
//                  obst_angle = std::atan2(pt_robot.getY(), pt_robot.getX());
//              }

//              min_dist = std::min<double>(min_dist, pt_robot.length());
//          }
//      }
//  }

//  distance_to_obstacle_ = min_dist;

  //ensure valid values
//  if (!std::isnormal(distance_to_obstacle_)) distance_to_obstacle_ = 1e5;
//  if (!std::isnormal(distance_to_goal_)) distance_to_goal_ = 1e5;

  //consider only the obstacles closer than a threshold, and slow down only when driving forward
//  double epsilon_o = 0.0;
//  if(distance_to_obstacle_ <= obst_threshold_){
//      epsilon_o = k_o_/distance_to_obstacle_;
//  }
//  //consider the obstacle orientation for backward driving
//  if(getDirSign() < 0){
//      obst_angle -= M_PI;
//  }

//  double fact_curv = k_curv_*curv_sum_;
//  if (!std::isnormal(fact_curv)) fact_curv = 0.0;
  double fact_w = k_w_*fabs(angular_vel);
  if (!std::isnormal(fact_w)) fact_w = 0.0;
//  double fact_obst = epsilon_o*std::max(0.0, cos(obst_angle));
//  if (!std::isnormal(fact_obst)) fact_obst = 0.0;
  double fact_goal = std::min(3.0, k_g_/goal_position_error); // TODO: remove this hack to avoid non-moving robot!
  if (!std::isnormal(fact_goal)) fact_goal = 0.0;

  //publish the factors of the exponential speed control
//  std_msgs::Float64MultiArray exp_control_array;
//  exp_control_array.data.resize(4);
//  exp_control_array.data[0] = fact_curv;
//  exp_control_array.data[1] = fact_w;
//  exp_control_array.data[2] = fact_obst;
//  exp_control_array.data[3] = fact_goal;
//  exp_control_pub_.publish(exp_control_array);

//    ROS_INFO("k_curv: %f, k_o: %f, k_w: %f, k_g: %f, look_ahead: %f, obst_thresh: %f",
//             k_curv_, k_o_, k_w_, k_g_, look_ahead_dist_, obst_threshold_);
  //double exponent = fact_curv + fact_w + fact_obst + fact_goal;
  double exponent = fact_w + fact_goal;
  return exp(-exponent);
}

void Differential_Pure_Pursuit_Controller::controllerParamsCallback(vehicle_controller::PurePursuitControllerParamsConfig &config, uint32_t level){
  mp_.carrot_distance = config.lookahead_distance;
}


