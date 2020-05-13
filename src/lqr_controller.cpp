#include <vehicle_controller/lqr_controller.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

Lqr_Controller::Lqr_Controller(ros::NodeHandle& nh_)
  : state(INACTIVE), stuck(new StuckDetector), nh_dr_params("~/controller_params")
{
  nh = nh_;

  //LQR parameters
  lqr_q11 = 1000;
  lqr_q22 = 0;
  lqr_r = 1;
  rot_vel_dir = 1;
  lin_vel_dir = 1;

  mp_.carrot_distance = 0.2;
  mp_.min_speed       = 0.0;
  mp_.commanded_speed = 0.0;
  mp_.max_controller_speed = 0.6;
  mp_.max_unlimited_speed = 0.6;
  mp_.max_unlimited_angular_rate = 1.2;
  mp_.max_controller_angular_rate =  1.2;
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

  follow_path_server_->registerGoalCallback(boost::bind(&Lqr_Controller::followPathGoalCallback, this));
  follow_path_server_->registerPreemptCallback(boost::bind(&Lqr_Controller::followPathPreemptCallback, this));

  follow_path_server_->start();
}

Lqr_Controller::~Lqr_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    delete dr_controller_params_server;
  }
}

bool Lqr_Controller::configure()
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

  if (vehicle_control_type == "differential_steering")
    vehicle_control_interface_.reset(new DifferentialDriveController());
  else
    vehicle_control_interface_.reset(new FourWheelSteerController());
  vehicle_control_interface_->configure(params, &mp_);

  ROS_INFO("[vehicle_controller] Low level vehicle motion controller is %s", this->vehicle_control_interface_->getName().c_str());

  stateSubscriber     = nh.subscribe("state", 10, &Lqr_Controller::stateCallback, this, ros::TransportHints().tcpNoDelay(true));
  drivetoSubscriber   = nh.subscribe("driveto", 10, &Lqr_Controller::drivetoCallback, this);
  drivepathSubscriber = nh.subscribe("drivepath", 10, &Lqr_Controller::drivepathCallback, this);
  cmd_velSubscriber   = nh.subscribe("cmd_vel", 10, &Lqr_Controller::cmd_velCallback, this, ros::TransportHints().tcpNoDelay(true));
  cmd_velTeleopSubscriber = nh.subscribe("cmd_vel_teleop", 10, &Lqr_Controller::cmd_velTeleopCallback, this, ros::TransportHints().tcpNoDelay(true));
  speedSubscriber     = nh.subscribe("speed", 10, &Lqr_Controller::speedCallback, this);
  poseSubscriber      = nh.subscribe("robot_pose", 10, &Lqr_Controller::poseCallback, this);

  carrotPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("carrot", 1, true);
  endPosePoublisher   = nh.advertise<geometry_msgs::PoseStamped>("end_pose", 1, true);
  drivepathPublisher  = nh.advertise<nav_msgs::Path>("drivepath", 1, true);
  pathPosePublisher   = nh.advertise<nav_msgs::Path>("smooth_path", 1, true);

  diagnosticsPublisher = params.advertise<std_msgs::Float32>("velocity_error", 1, true);
  autonomy_level_pub_ = nh.advertise<std_msgs::String>("/autonomy_level", 30);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);
  //cmd_vel_pub         = nh.advertise<geometry_msgs::Twist>("/affw_ctrl/target_vel", 1, true);

  if (camera_control)
  {
    cameraOrientationPublisher = nh.advertise<geometry_msgs::QuaternionStamped>("camera/command", 1);
    lookatPublisher = nh.advertise<geometry_msgs::PointStamped>("camera/look_at", 1);
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
  }
  empty_path.header.frame_id = map_frame_id;

//  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::CarrotControllerParamsConfig>(nh_dr_params);
//  dr_controller_params_server->setCallback(boost::bind(&Lqr_Controller::controllerParamsCallback, this, _1, _2));

  return true;
}

bool Lqr_Controller::updateRobotState(const nav_msgs::Odometry& odom_state)
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

void Lqr_Controller::poseCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{

  geometry_msgs::PoseStampedConstPtr pose = event.getConstMessage();
  current_pose = *pose;

}

void Lqr_Controller::stateCallback(const nav_msgs::OdometryConstPtr& odom_state)
{
  latest_odom_ = odom_state;

  if (state < DRIVETO) return;

  this->updateRobotState(*latest_odom_);

  update();
}

void Lqr_Controller::drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{
  geometry_msgs::PoseStampedConstPtr goal = event.getConstMessage();

  if (follow_path_server_->isActive()){
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_server_->setPreempted(result, "drive to callback");
  }
  driveto(*goal, 0.0);
}

bool Lqr_Controller::driveto(const geometry_msgs::PoseStamped& goal, double speed)
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

void Lqr_Controller::drivepathCallback(const ros::MessageEvent<nav_msgs::Path>& event)
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

bool Lqr_Controller::pathToBeSmoothed(const std::deque<geometry_msgs::PoseStamped>& transformed_path, bool fixed_path)
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

bool Lqr_Controller::drivepath(const nav_msgs::Path& path)
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

    std::for_each(smooth_path.begin() + 1, smooth_path.end(), boost::bind(&Lqr_Controller::addLeg, this, _1, options.desired_speed));

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
    ROS_WARN("[vehicle_controller] Lqr_Controller::drivepath produced empty legs array.");

  return true;
}


bool Lqr_Controller::createDrivepath2MapTransform(tf::StampedTransform & transform, const nav_msgs::Path& path)
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

void Lqr_Controller::cmd_velCallback(const geometry_msgs::Twist& velocity)
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

void Lqr_Controller::cmd_velTeleopCallback(const geometry_msgs::Twist& velocity)
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

void Lqr_Controller::speedCallback(const std_msgs::Float32& speed)
{
  mp_.commanded_speed = speed.data;
}

void Lqr_Controller::stopVehicle()
{
  geometry_msgs::Vector3 p;
  robot_control_state.setControlState(0.0, p, 0, 0, mp_.carrot_distance, 0.0, false, true);
  vehicle_control_interface_->executeMotionCommand(robot_control_state);
}

void Lqr_Controller::followPathGoalCallback()
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

void Lqr_Controller::followPathPreemptCallback()
{
  stopVehicle();
  move_base_lite_msgs::FollowPathResult result;
  result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
  follow_path_server_->setPreempted(result, "preempt from incoming message to server");
  reset();
}

void Lqr_Controller::addLeg(const geometry_msgs::PoseStamped& pose, double speed)
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

bool Lqr_Controller::reverseAllowed()
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

bool Lqr_Controller::reverseForced()
{
  return false;
  if (follow_path_server_->isActive()) {
    return follow_path_goal_->follow_path_options.reverse_forced;
  } else {
    return false;
  }


}

void Lqr_Controller::reset()
{
  lqr_y_error_integrate = 0;
  lqr_y_error = 0;
  lqr_angle_error = 0;
  state = INACTIVE;
  current = 0;
  final_twist_trials = 0;
  dt = 0.0;
  legs.clear();
}

void Lqr_Controller::update()
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

//  if(std::abs(signed_carrot_distance_2_robot) > (mp_.carrot_distance * 5))  //  CHANGED FROM 1.5 to 5
//  {
//    ROS_WARN("[vehicle_controller] Control failed, distance to carrot is %f (allowed: %f)", signed_carrot_distance_2_robot, (mp_.carrot_distance * 1.5));
//    state = INACTIVE;
//    stop();

//    if (follow_path_server_->isActive()){
//      move_base_lite_msgs::FollowPathResult result;
//      result.result.val = move_base_lite_msgs::ErrorCodes::CONTROL_FAILED;
//      follow_path_server_->setAborted(result, std::string("Control failed, distance between trajectory and robot too large."));
//    }
//    return;
//  }

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


  robot_control_state.desired_velocity_linear = 0.3;

  //robot_control_state.desired_velocity_linear *= std::max(0.0, 1.0 - (fabs(robot_control_state.velocity_linear.y) / 0.3));

  //solveDare();

  calc_local_path();
  calcLqr();
  ROS_INFO("radius: %f", local_path_radius);

  ROS_INFO ("angle error: %f", lqr_angle_error);

  if(reverseAllowed()){
    if (lqr_angle_error > M_PI/2){
      lqr_angle_error = lqr_angle_error - M_PI;
      lin_vel_dir = -1;
    }
    else if(lqr_angle_error < -M_PI/2){
      lqr_angle_error = lqr_angle_error + M_PI;
      lin_vel_dir = -1;
    }
    else{
      lin_vel_dir = 1;
    }
  }
  else{
    lin_vel_dir = 1;
  }

  //double omega_ff = (lin_vel_dir * rot_vel_dir * robot_control_state.velocity_linear.x / local_path_radius);
  double omega_ff = (lin_vel_dir * rot_vel_dir * robot_control_state.desired_velocity_linear / local_path_radius);
  //omega_ff = omega_ff * std::cos(roll) * std::cos(pitch);

  double omega_fb = - lin_vel_dir *lqr_k1 * lqr_y_error - lqr_k2 * lqr_angle_error;
  //double omega_fb = - lin_vel_dir *K(0,0) * lqr_y_error - K(0,1) * lqr_angle_error;

  //omega_fb = omega_fb * robot_control_state.velocity_linear.x/robot_control_state.desired_velocity_linear;

  double dt_test = (ros::Time::now() - lqr_time).toSec();
//  if(dt_test > 0.0){
//    lqr_expected_dy = lqr_last_cmd.linear.x * lqr_last_angle_error * dt_test;
//    lqr_real_dy = lqr_y_error - lqr_last_y_error;
//  }

//  double angular_vel;
//  if (lqr_expected_dy * lqr_real_dy < 0.0){
//    ROS_INFO("Just FF");
//    angular_vel = omega_ff ;
//  }
//  else{
//    angular_vel = omega_ff + omega_fb;
//  }

  double angular_vel= omega_ff + omega_fb;

  //_________________________________________________________________________--
//  exakte LInearisierung
//  double v = robot_control_state.desired_velocity_linear;
//  double a1 = 2;
//  double a0 = 10;

//   double angular_vel = - (-v*std::cos(lqr_angle_error)*omega_ff + a1 * v *std::sin(lqr_angle_error) + a0 * lqr_y_error) / (v*std::cos(lqr_angle_error));


  //____________________________________________________________________________

//  double k_w = 0.4;
//  double k_curv = 0.01;

//  double fact_curv = k_curv*1/local_path_radius;
//  if (!std::isnormal(fact_curv)) fact_curv = 0.0;
//  double fact_w = k_w*fabs(angular_vel);
//  if (!std::isnormal(fact_w)) fact_w = 0.0;

//  ROS_INFO("fact curv: %f, fact w: %f", fact_curv, fact_w);

//  //double exponent = fact_curv + fact_w + fact_obst + fact_goal;
//  double exponent = fact_w + fact_curv;

  geometry_msgs::Twist cmd;
  cmd.linear.x = lin_vel_dir * fabs(robot_control_state.desired_velocity_linear) ;//* exp(-exponent);
  cmd.angular.z = angular_vel ;//* (1 - fabs(robot_control_state.desired_velocity_linear - robot_control_state.velocity_linear.x));
  ROS_INFO("velocity x: %f, y: %f", robot_control_state.velocity_linear.x, robot_control_state.velocity_linear.y);

  ROS_INFO("unlimited: lin: %f, ang: %f", cmd.linear.x, cmd.angular.z);
  this->limitTwist(cmd, mp_.max_controller_speed, mp_.max_controller_angular_rate);
  ROS_INFO("limited: lin: %f, ang: %f", cmd.linear.x, cmd.angular.z);


  //cmd.angular.z = cmd.angular.z / (std::cos(roll) * std::cos(pitch));

  ROS_INFO ("ff: %f , fb: %f", omega_ff, omega_fb);
  ROS_INFO ("k1: %f , k2: %f", lqr_k1, lqr_k2);
  ROS_INFO ("y_error: %f, w_error: %f, x_error: %f", lqr_y_error, lqr_angle_error, lqr_x_error);
  //cmd_vel_pub.publish(cmd);

    if(ekf_useEkf){
      if (!ekf_setInitialPose){
        ekf.x_(0,0) = robot_control_state.pose.position.x;
        ekf.x_(1,0) = robot_control_state.pose.position.y;
        ekf.x_(2,0) = yaw;

        ekf_setInitialPose = true;
        ekf_lastTime = ros::Time::now();
        ekf_lastCmd = cmd;

        ekf_last_pitch = pitch;
        ekf_last_roll = roll;
        ekf_last_yaw = yaw;

        cmd_vel_pub.publish(cmd);
        ROS_INFO("initial SET");
      }
      else{
        double dt = (ros::Time::now().toSec() - ekf_lastTime.toSec());

        double l;
        nh.getParam("/vehicle_controller/wheel_separation", l);

        double v_lin = ekf_lastCmd.linear.x;
        double v_ang = ekf_lastCmd.angular.z;

        double Vl_ = v_lin - l/2 * v_ang;
        double Vr_ = v_lin + l/2 * v_ang;

        if(dt > 0){
          ekf.predict(Vl_, Vr_, ekf_last_pitch, ekf_last_roll, dt);

          Eigen::Vector3d delta;
          delta(0) = robot_control_state.pose.position.x;
          delta(1) = robot_control_state.pose.position.y;
          delta(2) = yaw;
          ekf.correct(delta);

          double omega = -(Vl_ - Vr_)/fabs(ekf.x_(4) - ekf.x_(3))  * std::cos(ekf_last_roll) * std::cos(ekf_last_pitch);
          ROS_INFO("omega: %f, twist: %f, pose.twist: %f, yaw_diff: %f", omega, cmd.angular.z, robot_control_state.velocity_angular.z, (yaw-ekf_last_yaw)/dt);

          double y_ICRr = ekf.x_(3,0);
          double y_ICRl = ekf.x_(4,0);

          double vl_corrected = cmd.linear.x - y_ICRl * cmd.angular.z;
          double vr_corrected = cmd.linear.x - y_ICRr * cmd.angular.z;

          ROS_INFO("vl: %f, vl_corrected: %f", Vl_, vl_corrected);
          ROS_INFO("vr: %f, vr_corrected: %f", Vr_, vr_corrected);
          ROS_INFO("vlin: %f, vlin_corrected: %f", cmd.linear.x, (vl_corrected + vr_corrected)/2);

          cmd_vel_pub.publish(cmd);

          ROS_INFO("yl: %f, yr: %f, x: %f",ekf.x_(4), ekf.x_(3), ekf.x_(5) );

          double icr = (ekf.x_(4) + ekf.x_(3));
          ROS_INFO("ICR: %f", icr);

          ekf_lastCmd = cmd;
          ekf_lastTime = ros::Time::now();
          ekf_last_pitch = pitch;
          ekf_last_roll = roll;
          ekf_last_yaw = yaw;
        }

      }
    }
    else{
//      if(!lqr_aligning_2){
//        if(fabs(lqr_y_error) > 0.03){
//          lqr_aligning = true;
//          lqr_aligning_2 = true;
//        }
//      }
//      else{
//        if(fabs(lqr_y_error) < 0.015){
//          lqr_aligning = false;
//          lqr_aligning_2 = false;
//        }
//      }
//      if (lqr_aligning){
//        if (robot_control_state.error_2_path_angular < 0.1){
//          lqr_aligning = false;
//        }
//        else{
//          cmd.linear.x = 0.0;
//          cmd.angular.z = robot_control_state.error_2_path_angular;
//        }
//      }
      cmd_vel_pub.publish(cmd);
    }

  ROS_INFO("expected dy: %f, real dy: %f, diff: %f, dt: %f", lqr_expected_dy, lqr_real_dy, lqr_expected_dy - lqr_real_dy, dt_test);
  ROS_INFO("lin-out: %f ang-out: %f",cmd.linear.x, cmd.angular.z);
  ROS_INFO("lin-ist: %f ang-ist: %f",robot_control_state.velocity_linear.x, robot_control_state.velocity_angular.z);

  lqr_last_cmd = cmd;
  lqr_time = ros::Time::now();

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

void Lqr_Controller::stop()
{
  this->vehicle_control_interface_->stop();
  stuck->reset();
  drivepathPublisher.publish(empty_path);
  if (camera_control)
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
}

//void Lqr_Controller::controllerParamsCallback(){
//  mp_.carrot_distance = config.carrot_distance;
//}

void Lqr_Controller::calc_local_path(){
  int next_point = calcClosestPoint();

  mp_.carrot_distance = 0.3;

  double path_po_lenght = 0;
  //calculate path_po_lenght
  int psize = current_path.poses.size();

  for(int i=0; i < psize; i++)
  {
    double curr_dist_x =fabs(closest_point.point.x - current_path.poses[i].pose.position.x);
    double curr_dist_y =fabs(closest_point.point.y - current_path.poses[i].pose.position.y);
      double curr_dist = sqrt(curr_dist_x*curr_dist_x + curr_dist_y*curr_dist_y);


      if(fabs(curr_dist) > mp_.carrot_distance) //search for points
      {
          continue;
      }
      path_po_lenght = path_po_lenght + 1;
  }

  double min_dif = 10.0;

  double points[50][2];

  int st_point, co_points;
  double th_po_x, th_po_y, fi_po_x, fi_po_y, se_po_x, se_po_y;
  double max_H;

  double dirx, diry;
  double sideA, sideB, sideC;
  double ss, area, tmp_H;
  double Wid;

  double midX, midY;
  double dx, dy;
  double distt, pdist;
  double mDx, mDy;

  //start point from robot current pose
  points[0][0] = closest_point.point.x;
  points[0][1] = closest_point.point.y;

  //search for closest point to path
  for(int i=0; i < psize; i++)
  {
      double po_dist = sqrt((current_path.poses[i].pose.position.x - points[0][0])*(current_path.poses[i].pose.position.x - points[0][0]) + (current_path.poses[i].pose.position.y - points[0][1])*(current_path.poses[i].pose.position.y - points[0][1]));
      if(fabs(po_dist) < min_dif)
      {
          min_dif = fabs(po_dist);
          st_point = i;
      }
  }
  //ROS_INFO("Founded closest point on path: x: %f y: %f at postition: %i", curr_path.poses[st_point].pose.position.x, curr_path.poses[st_point].pose.position.x, st_point);

  //calculate execution path distance
  co_points = 0;
  for(int i=st_point; i < (st_point+path_po_lenght); i++)
  {
      if(i > (psize-2))
      {
          co_points = co_points +1;
          points[co_points][0] = current_path.poses[i].pose.position.x;
          points[co_points][1] = current_path.poses[i].pose.position.y;
          i = (st_point+path_po_lenght)+1;
      }else
      {
          co_points = co_points +1;
          points[co_points][0] = current_path.poses[i].pose.position.x;
          points[co_points][1] = current_path.poses[i].pose.position.y;
      }
  }

  th_po_x = 0; th_po_y = 0; fi_po_x = 0; fi_po_y = 0;
  se_po_x = 0; se_po_y = 0; dirx = 1; diry = -1; max_H = 0;

  //calculate triangle height height
  for(int i=0; i < co_points; i++)
  {
                                     //p1            p2              p3
      //ROS_INFO("Points X: %f %f %f", points[0][0], points[i][0], points[co_points][0]);
      //ROS_INFO("Points Y: %f %f %f", points[0][1], points[i][1], points[co_points][1]);
      sideA = sqrt(((points[0][0] - points[i][0])*(points[0][0] - points[i][0])) + (points[0][1] - points[i][1])*(points[0][1] - points[i][1]));
      sideB = sqrt(((points[i][0] - points[co_points][0])*(points[i][0] - points[co_points][0])) + (points[i][1] - points[co_points][1])*(points[i][1] - points[co_points][1]));
      sideC = sqrt(((points[co_points][0] - points[0][0])*(points[co_points][0] - points[0][0])) + (points[co_points][1] - points[0][1])*(points[co_points][1] - points[0][1]));
      //ROS_INFO("triangle sides: %f %f %f", sideA, sideB, sideC);
      ss = (sideA + sideB + sideC)/2;
      area = sqrt(ss*(ss-sideA)*(ss-sideB)*(ss-sideC));
      //determine params for radius calculation
      tmp_H = (area*2)/sideC;

      if(tmp_H > max_H)
      {
          max_H = tmp_H;
          //float det_dir = (points[co_points][0] - points[1][0])*(points[i][1] - points[0][1]) - (points[co_points][1] - points[0][1])*(points[i][0]- points[0][0]);
          float det_dir = (points[co_points][0] - points[0][0])*(points[i][1] - points[0][1]) - (points[co_points][1] - points[0][1])*(points[i][0]- points[0][0]);
          se_po_x = points[i][0];
          se_po_y = points[i][1];

          if(det_dir > 0)
          {
              dirx = -1;
              diry = 1;
              rot_vel_dir = -1;
          }else
          {
              dirx = 1;
              diry = -1;
              rot_vel_dir = 1;
          }
      }
      Wid = sideC;
  }

  //if local path is too short
  if(co_points < 3)
  {
      max_H = 0.001;
  }
  //smooth local path
  //max_H = max_H/2;

  //calculate ground compensation, which modifiy max_H and W
  //calc_ground_compensation();

  fi_po_x = points[0][0];
  fi_po_y = points[0][1];
  th_po_x = points[co_points][0];
  th_po_y = points[co_points][1];

  //calculate radious
  local_path_radius = max_H/2 + (Wid*Wid)/(8*max_H);
  ROS_INFO("Fitted circle radius: %f", local_path_radius);

  //calculating circle center
  midX = (points[0][0] + points[co_points][0])/2;
  midY = (points[0][1] + points[co_points][1])/2;
  dx = (points[0][0] - points[co_points][0])/2;
  dy = (points[0][1] - points[co_points][1])/2;
  distt = sqrt(dx*dx + dy*dy);
  pdist = sqrt(local_path_radius*local_path_radius - distt*distt);
  mDx = dirx*dy*pdist/distt;
  mDy = diry*dx*pdist/distt;

  //calculate alignemnt angle
  double curr_dist_x = points[0][0] -  (midX + mDx);
  double curr_dist_y = points[0][1] - (midY + mDy);

  if(isinf(local_path_radius)){
    alignment_angle = atan2(current_path.poses[co_points].pose.position.y - closest_point.point.y,
                            current_path.poses[co_points].pose.position.x - closest_point.point.x);
  }
  else{
    //correct angle directions
    if((curr_dist_x < 0)&&(curr_dist_y < 0))
    {
      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
    }
    else if((curr_dist_x > 0)&&(curr_dist_y > 0))
    {
      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
    }
    else if((curr_dist_x < 0)&&(curr_dist_y > 0))
    {
      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
    }
    else if((curr_dist_x > 0)&&(curr_dist_y < 0))
    {
      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
    }
  }

  //reduce angle on -PI to +PI
  if(alignment_angle > M_PI)
  {
      alignment_angle = alignment_angle - 2*M_PI;
  }
  if(alignment_angle < -M_PI)
  {
      alignment_angle = alignment_angle + 2*M_PI;
  }

  if(isnan(alignment_angle))
  {
      ROS_WARN("Alignment Angle can not be computed!");
  }

  //ROS_INFO("Alignment angle is: %f", alignment_angle);
  if(isnan(alignment_angle))
  {
      ROS_INFO("Alignment angle is nan - return to calc_local_path");
      calc_local_path();
  }

}

//void Lqr_Controller::calc_local_path(){
//  int next_point = calcClosestPoint();

//  double co_point = 0;
//  double triangle_height = 0.001;
//  double det_dir = 1;
//  double sideA;
//  double sideB;
//  double sideC;

//  if((next_point + 2) < current_path.poses.size()){
//    co_point = next_point + 2;

//    sideA = sqrt(std::pow(closest_point.point.x - current_path.poses[next_point + 1].pose.position.x, 2) +
//        std::pow(closest_point.point.y - current_path.poses[next_point + 1].pose.position.y, 2));
//    sideB = sqrt(std::pow(current_path.poses[next_point + 1].pose.position.x - current_path.poses[next_point + 2].pose.position.x, 2) +
//        std::pow(current_path.poses[next_point + 1].pose.position.y - current_path.poses[next_point + 2].pose.position.y, 2));
//    sideC = sqrt(std::pow(closest_point.point.x - current_path.poses[next_point + 2].pose.position.x, 2) +
//        std::pow(closest_point.point.y - current_path.poses[next_point + 2].pose.position.y, 2));

//    //Calulate area with rule of heron
//    double ss = (sideA + sideB + sideC)/2;
//    double area = sqrt(ss*(ss-sideA)*(ss-sideB)*(ss-sideC));

//    if(isnan(area)){
//      ROS_INFO ("area is nan, setting to 0.0");
//      area = 0.0;
//    }

//    triangle_height = (area*2)/sideC;

//    ROS_INFO("height: %f, sideC: %f, sideA: %f, sideB: %f, area: %f", triangle_height, sideC, sideA, sideB, area);
//    ROS_INFO("next_point: %i", next_point);
//    ROS_INFO("closestPointX: %f, Y: %f", closest_point.point.x, closest_point.point.y);

//    det_dir = (current_path.poses[next_point + 2].pose.position.x -  closest_point.point.x)*(current_path.poses[next_point + 1].pose.position.y - closest_point.point.y)
//              - (current_path.poses[next_point + 1].pose.position.x - closest_point.point.x)*(current_path.poses[next_point + 2].pose.position.y - closest_point.point.y);

//  }
//  else{
//    co_point = current_path.poses.size() - 1;
//    triangle_height = 0.001;

//    sideC = sqrt(std::pow(closest_point.point.x - current_path.poses[co_point].pose.position.x, 2) +
//        std::pow(closest_point.point.y - current_path.poses[co_point].pose.position.y, 2));
//  }

////  double dh = fabs((triangle_height/cos(roll))-triangle_height);
////  double dw = fabs((sideC/cos(pitch))-sideC);

////  triangle_height = triangle_height + dh;
////  sideC = sideC + dw;

//  double dirx;
//  double diry;

//  if(det_dir > 0)
//  {
//      dirx = -1;
//      diry = 1;
//      rot_vel_dir = -1;
//  }else
//  {
//      dirx = 1;
//      diry = -1;
//      rot_vel_dir = 1;
//  }


//  local_path_radius = triangle_height/2 + sideC*sideC / (8*triangle_height);

//  //calculating circle center
//  double midX = (closest_point.point.x + current_path.poses[co_point].pose.position.x)/2;
//  double midY = (closest_point.point.y + current_path.poses[co_point].pose.position.y)/2;
//  double dx = (closest_point.point.x - current_path.poses[co_point].pose.position.x)/2;
//  double dy = (closest_point.point.y - current_path.poses[co_point].pose.position.y)/2;
//  double distt = sqrt(dx*dx + dy*dy);
//  double pdist = sqrt(local_path_radius*local_path_radius - distt*distt);
//  double mDx = dirx*dy*pdist/distt;
//  double mDy = diry*dx*pdist/distt;

//  //calculate alignemnt angle
//  double curr_dist_x = closest_point.point.x -  (midX + mDx);
//  double curr_dist_y = closest_point.point.y - (midY + mDy);

//  if(isinf(local_path_radius)){
//    alignment_angle = atan2(current_path.poses[co_point].pose.position.y - closest_point.point.y,
//                            current_path.poses[co_point].pose.position.x - closest_point.point.x);
//  }
//  else{
//    //correct angle directions
//    if((curr_dist_x < 0)&&(curr_dist_y < 0))
//    {
//      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
//    }
//    else if((curr_dist_x > 0)&&(curr_dist_y > 0))
//    {
//      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
//    }
//    else if((curr_dist_x < 0)&&(curr_dist_y > 0))
//    {
//      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
//    }
//    else if((curr_dist_x > 0)&&(curr_dist_y < 0))
//    {
//      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
//    }
//  }

//  //reduce angle on -PI to +PI
//  if(alignment_angle > M_PI)
//  {
//      alignment_angle = alignment_angle - 2*M_PI;
//  }
//  if(alignment_angle < -M_PI)
//  {
//      alignment_angle = alignment_angle + 2*M_PI;
//  }

//  if(isnan(alignment_angle))
//  {
//      ROS_INFO("Alignment angle is nan - return to calc_local_path");
//      calc_local_path();
//  }

//}

//Calculate the closest Point on the linear interpolated path, returns index of next point on path
int Lqr_Controller::calcClosestPoint(){
  //Calculate the two closest Points on the path
  int closest = 1;
  int second_closest = 0;
  double shortest_dist = 999999;
  for(int i = 0; i < current_path.poses.size(); i++){
    double dist = std::sqrt(std::pow(robot_control_state.pose.position.x - current_path.poses[i].pose.position.x, 2)
                           + std::pow(robot_control_state.pose.position.y - current_path.poses[i].pose.position.y, 2));
    if (dist < shortest_dist){
      shortest_dist = dist;
      second_closest = closest;
      closest = i;
    }
  }

//  if(abs(closest - second_closest) > 1){
//    if (closest == 0){
//      second_closest = 1;
//    }
//    else{
//      if ((closest + 1) < current_path.poses.size()){
//        double dist_next = std::sqrt(std::pow(robot_control_state.pose.position.x - current_path.poses[closest + 1].pose.position.x, 2)
//                               + std::pow(robot_control_state.pose.position.y - current_path.poses[closest + 1].pose.position.y, 2));
//        double dist_prev = std::sqrt(std::pow(robot_control_state.pose.position.x - current_path.poses[closest - 1].pose.position.x, 2)
//                               + std::pow(robot_control_state.pose.position.y - current_path.poses[closest - 1].pose.position.y, 2));

//        if(dist_next < dist_prev){
//          second_closest = closest + 1;
//        }
//        else{
//          second_closest = closest - 1;
//        }
//      }
//      else{
//        second_closest = closest - 1;
//      }
//    }
//  }

  if (closest == 0){
    second_closest = 1;
  }
  else{
    if ((closest + 1) < current_path.poses.size()){
      double prev_dx = (current_path.poses[closest - 1].pose.position.x - current_path.poses[closest].pose.position.x);
      double prev_dy = (current_path.poses[closest - 1].pose.position.y - current_path.poses[closest].pose.position.y);
      double next_dx = (current_path.poses[closest + 1].pose.position.x - current_path.poses[closest].pose.position.x);
      double next_dy = (current_path.poses[closest + 1].pose.position.y - current_path.poses[closest].pose.position.y);
      double robot_dx = (robot_control_state.pose.position.x - current_path.poses[closest].pose.position.x);
      double robot_dy = (robot_control_state.pose.position.y - current_path.poses[closest].pose.position.y);

      double angle_prev = std::acos((prev_dx*robot_dx + prev_dy*robot_dy) / (sqrt(prev_dx*prev_dx + prev_dy*prev_dy) + sqrt(robot_dx*robot_dx + robot_dy*robot_dy) ));
      double angle_next = std::acos((next_dx*robot_dx + next_dy*robot_dy) / (sqrt(next_dx*next_dx + next_dy*next_dy) + sqrt(robot_dx*robot_dx + robot_dy*robot_dy) ));

      if (fabs(angle_prev) < fabs(angle_next)){
        second_closest = closest - 1;
      }
      else{
        second_closest = closest + 1;
      }
    }
    else{
      second_closest = closest - 1;
    }
  }

  //Calculate the closest Point on the connection line of the two closest points on the path
  double l1 = current_path.poses[second_closest].pose.position.x - current_path.poses[closest].pose.position.x;
  double l2 = current_path.poses[second_closest].pose.position.y - current_path.poses[closest].pose.position.y;

  double r1 = -l2;
  double r2 = l1;

  double lambda = (l2 * (current_path.poses[closest].pose.position.x - robot_control_state.pose.position.x)
                   + l1 * (robot_control_state.pose.position.y - current_path.poses[closest].pose.position.y) ) / (r1*l2 - r2*l1);

  closest_point.point.x = robot_control_state.pose.position.x + lambda * r1;
  closest_point.point.y = robot_control_state.pose.position.y + lambda * r2;
  closest_point.header = robot_state_header;

  ROS_INFO("closest: %i, second: %i", closest, second_closest);
  ROS_INFO("closest: x: %f, y: %f", current_path.poses[closest].pose.position.x, current_path.poses[closest].pose.position.y);
  ROS_INFO("second: x: %f, y: %f", current_path.poses[second_closest].pose.position.x, current_path.poses[second_closest].pose.position.y);

  if (closest > second_closest){
    return closest;
  }
  else{
    return second_closest;
  }

}


void Lqr_Controller::calcLqr(){
  geometry_msgs::PointStamped closest_point_baseframe;

  lqr_last_y_error = lqr_y_error;
  lqr_last_angle_error = lqr_angle_error;
  //compute errors

  if(fabs((ros::Time::now() - lqr_time).toSec()) < 0.2){
    double dt = (ros::Time::now() - lqr_time).toSec();
    lqr_y_error_integrate = lqr_y_error_integrate + dt*lqr_y_error;
  }
  //lqr_time = ros::Time::now();

  try
  {
    closest_point.point.z = robot_control_state.pose.position.z;
    listener.waitForTransform(base_frame_id, robot_state_header.frame_id, robot_state_header.stamp, ros::Duration(3.0));
    listener.transformPoint(base_frame_id, closest_point, closest_point_baseframe);

    lqr_x_error = -closest_point_baseframe.point.x;
    lqr_y_error = -closest_point_baseframe.point.y;
//    std::cout << "robot pose odom: " << robot_control_state.pose.position << std::endl;
//    std::cout << "robot pose robotpose: " << current_pose << std::endl;
//    std::cout << "closest point: " << closest_point << std::endl;
//    std::cout << "closest point baseframe: " << closest_point_baseframe << std::endl;
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s", ex.what());
      return;
  }

  double angles[3];
  quaternion2angles(robot_control_state.pose.orientation, angles);

  lqr_angle_error =  constrainAngle_mpi_pi(angles[0] - alignment_angle);
  ROS_INFO("yaw: %f, al_angle: %f", angles[0] , alignment_angle);

  //compute control gains
  double v = fabs(robot_control_state.desired_velocity_linear);
  //double v = fabs(robot_control_state.velocity_linear.x);

  ROS_INFO ("speed: %f", v);

  if (v != 0.0){
    lqr_p12 = sqrt(lqr_q11 * lqr_r);
    lqr_p11 = sqrt(lqr_q11*(2 * lqr_p12 * v + lqr_q22)/(v*v));
    lqr_p22 = sqrt(lqr_r) * v * lqr_p11 / sqrt(lqr_q11);

    lqr_k1 = 1/lqr_r * lqr_p12;
    lqr_k2 = 1/lqr_r * lqr_p22;
  }
  else{
    lqr_k1 = 0.0;
    lqr_k2 = 0.0;
  }

}

//void Lqr_Controller::solveDare(){
//  double epsilon = 0.001;

//  Eigen::Matrix<double, 2, 2> Q =  Eigen::Matrix<double, 2, 2>::Zero();
//  Q(0,0) = lqr_q11;
//  Q(1,1) = lqr_q22;

//  Eigen::Matrix<double, 2, 2> P = Q;

//  Eigen::Matrix<double, 2, 2> P_new;

//  Eigen::Matrix<double, 2, 2> A = Eigen::Matrix<double, 2, 2>::Zero();
//  A(0,1) = fabs(robot_control_state.desired_velocity_linear);
//  Eigen::Matrix<double, 2, 2> A_t = A.transpose();

//  Eigen::Matrix<double, 2, 1> B = Eigen::Matrix<double, 2, 1>::Zero();
//  B(1,0) = 1;
//  Eigen::Matrix<double, 1, 2> B_t = B.transpose();

//  Eigen::Matrix<double, 1, 2> K = B.transpose();

//  double help_var = 1/(B_t*P*B+lqr_r);

//  double diff;
//  int max_iter = 1000;
//  for(int i = 0; i< max_iter;i++){
//    help_var = 1/(B_t*P*B+lqr_r);
//    P_new = Q + A_t * P * A - A_t * P * B * help_var * B_t * P * A;
//    std::cout << "Matrix P: " << P <<std::endl;
//    std::cout << "Matrix Pnew: " << P_new <<std::endl;

//    Eigen::Matrix2d P_test = P_new - P;
//    diff = fabs(P_test.maxCoeff());

//    ROS_INFO ("diff: %f", diff);
//    ROS_INFO("helpvar: %f", help_var);
//    std::cout << "Matrix P: " << P <<std::endl;
//    std::cout << "Matrix Pnew: " << P_new <<std::endl;
//    P = P_new;
//    if(diff < epsilon){
//      ROS_INFO("iterations: %i, p11: %f, p12: %f, p22: %f",i, P(0,0), P(0,1), P(1,1));
//      K = help_var * B_t * P * A;
//      ROS_INFO("k1: %f, k2:%f", K(0,0), K(0,1));
//      return;
//    }

//  }
//  ROS_INFO("max iterations, diff: %f, p11: %f, p12: %f, p22: %f",diff, P(0,0), P(0,1), P(1,1));
//  K = help_var * B_t * P * A;
//  ROS_INFO("k1: %f, k2:%f", K(0,0), K(0,1));

//}

void Lqr_Controller::solveDare(){
  double epsilon = 0.001;

  double xicr = - robot_control_state.velocity_linear.y /robot_control_state.velocity_angular.z;
  if(isnan(xicr)){
    xicr = 0.0;
  }

  ROS_INFO("xicr: %f", xicr);

  Eigen::Matrix<double, 2, 2> Q =  Eigen::Matrix<double, 2, 2>::Zero();
  Q(0,0) = 1000;
  Q(1,1) = 0.0;

  Eigen::Matrix<double, 2, 2> P = Eigen::Matrix<double, 2, 2>::Zero();

  Eigen::Matrix<double, 2, 2> P_new;

  Eigen::Matrix<double, 2, 2> A = Eigen::Matrix<double, 2, 2>::Zero();
  A(0,1) = fabs(robot_control_state.desired_velocity_linear);
  Eigen::Matrix<double, 2, 2> A_t = A.transpose();

  Eigen::Matrix<double, 2, 1> B = Eigen::Matrix<double, 2, 1>::Zero();
  B(0,0) =  ekf.x_(5,0);
  B(1,0) = 1;
  Eigen::Matrix<double, 1, 2> B_t = B.transpose();


  double dt = 0.01;

  double diff;
  int max_iter = 1000;
  for(int i = 0; i< max_iter;i++){
    P_new = P - dt*(-Q - A_t*P-P*A+P*B*B_t*P*1/lqr_r);
    Eigen::Matrix2d P_test = P_new - P;
    diff = fabs(P_test.cwiseAbs().maxCoeff());

//    ROS_INFO ("diff: %f", diff);
//    std::cout << "Matrix P: " << P <<std::endl;
//    std::cout << "Matrix Pnew: " << P_new <<std::endl;
    P = P_new;
    if(diff < epsilon){
      ROS_INFO("iterations: %i, p11: %f, p12: %f, p22: %f",i, P(0,0), P(0,1), P(1,1));
      K = 1/lqr_r * B_t*P;
      ROS_INFO("k1: %f, k2:%f, k3: %f", K(0,0), K(0,1), K(0,2));
       ROS_INFO("ekf_x: %f, stanni xicr: %f", ekf.x_(5,0), xicr);
      return;
    }

  }
  ROS_INFO("max iterations, diff: %f, p11: %f, p12: %f, p22: %f",diff, P(0,0), P(0,1), P(1,1));
  K = 1/lqr_r * B_t*P;
  ROS_INFO("k1: %f, k2:%f, k3: %f", K(0,0), K(0,1), K(0,2));

  ROS_INFO("ekf_x: %f, stanni xicr: %f", ekf.x_(5,0), xicr);

}

void Lqr_Controller::limitTwist(geometry_msgs::Twist& twist, double max_speed, double max_angular_rate) const
{
  double SPEED_REDUCTION_GAIN_ = 0.5;

  double speed = twist.linear.x;
  double angular_rate = twist.angular.z;

  speed        = std::max(-mp_.max_unlimited_speed, std::min(mp_.max_unlimited_speed, speed));
  angular_rate = std::max(-mp_.max_unlimited_angular_rate, std::min(mp_.max_unlimited_angular_rate, angular_rate));

  double m = -mp_.max_controller_speed / mp_.max_controller_angular_rate;
  double t = mp_.max_controller_speed;
  double speedAbsUL = std::min(std::max(0.05, m * std::abs(angular_rate) * SPEED_REDUCTION_GAIN_ + t), max_speed);

  twist.linear.x = std::max(-speedAbsUL, std::min(speed, speedAbsUL));
  twist.angular.z = std::max(-max_angular_rate, std::min(max_angular_rate, angular_rate));
}
