#include <vehicle_controller/daf_controller.h>


Daf_Controller::Daf_Controller(ros::NodeHandle& nh_)
  : state(INACTIVE), stuck(new StuckDetector), nh_dr_params("~/controller_params")
{
  nh = nh_;

  //General controller params
  mp_.carrot_distance = 0.4;
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

  follow_path_server_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>(nh, "/controller/follow_path", 0, false));

  follow_path_server_->registerGoalCallback(boost::bind(&Daf_Controller::followPathGoalCallback, this));
  follow_path_server_->registerPreemptCallback(boost::bind(&Daf_Controller::followPathPreemptCallback, this));

  follow_path_server_->start();
}

Daf_Controller::~Daf_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    delete dr_controller_params_server;
  }
}

bool Daf_Controller::configure()
{
    ros::NodeHandle params("~");
    params.param<double>("pub_cmd_hz", pub_cmd_hz, 30);
    params.param<double>("rot_correction_factor", rot_correction_factor, 1);
    params.param<double>("execution_period", execution_period, 1.0);
    params.param<double>("k_p_rotation", k_p_rotation, 5);

    params.param<double>("update_skip_until_vel_increase", update_skip, 5);
    params.param<double>("lower_al_angle", lower_al_angle, 0.2);              //this angle determine when angle correction is executed
    params.param<double>("upper_al_angle", upper_al_angle, 1.0);              //this angle determine middle robot correction wich is compensate in linear movment
                                                                    //smaller correction angle means better fiting of trajectories
    params.param("enable_angle_compensation", enable_angle_compensation, true);
    params.param("enable_ground_compensation", enable_ground_compensation, true);
    params.param("enable_velocity_increase", enable_velocity_increase, true);
    params.param("show_trajectory_planing", show_trajectory_planing, false);
    params.param<double>("stability_angle", stability_angle, 1.0);

    params.param("max_controller_speed", mp_.max_controller_speed, 0.6);
    params.getParam("max_controller_angular_rate", mp_.max_controller_angular_rate);

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

    stateSubscriber     = nh.subscribe("state", 10, &Daf_Controller::stateCallback, this, ros::TransportHints().tcpNoDelay(true));
    imuSubscriber       = nh.subscribe("imu/data", 10, &Daf_Controller::imuCallback, this, ros::TransportHints().tcpNoDelay(true));
    drivetoSubscriber   = nh.subscribe("driveto", 10, &Daf_Controller::drivetoCallback, this);
    drivepathSubscriber = nh.subscribe("drivepath", 10, &Daf_Controller::drivepathCallback, this);
    cmd_velSubscriber   = nh.subscribe("cmd_vel", 10, &Daf_Controller::cmd_velCallback, this, ros::TransportHints().tcpNoDelay(true));
    cmd_velTeleopSubscriber = nh.subscribe("cmd_vel_teleop", 10, &Daf_Controller::cmd_velTeleopCallback, this, ros::TransportHints().tcpNoDelay(true));
    speedSubscriber     = nh.subscribe("speed", 10, &Daf_Controller::speedCallback, this);
    poseSubscriber      = nh.subscribe("robot_pose", 10, &Daf_Controller::poseCallback, this);

    endPosePoublisher   = nh.advertise<geometry_msgs::PoseStamped>("end_pose", 1, true);
    drivepathPublisher  = nh.advertise<nav_msgs::Path>("drivepath", 1, true);
    pathPosePublisher   = nh.advertise<nav_msgs::Path>("smooth_path", 1, true);

    diagnosticsPublisher = params.advertise<std_msgs::Float32>("velocity_error", 1, true);
    autonomy_level_pub_ = nh.advertise<std_msgs::String>("/autonomy_level", 30);

    //path following visualization
    if(show_trajectory_planing)
    {
      local_path_pub = nh.advertise<nav_msgs::Path>("/local_calc_path", 1);
      marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);

    if (camera_control)
    {
        cameraOrientationPublisher = nh.advertise<geometry_msgs::QuaternionStamped>("camera/command", 1);
        lookatPublisher = nh.advertise<geometry_msgs::PointStamped>("camera/look_at", 1);
        cameraOrientationPublisher.publish(cameraDefaultOrientation);
    }
    empty_path.header.frame_id = map_frame_id;


    dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::DafControllerParamsConfig>(nh_dr_params);
    dr_controller_params_server->setCallback(boost::bind(&Daf_Controller::controllerParamsCallback, this, _1, _2));

    return true;
}

bool Daf_Controller::updateRobotState(const nav_msgs::Odometry& odom_state)
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

//      odom.header = odom_state.header;
//      odom.pose.pose= pose.pose;

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

void Daf_Controller::poseCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{

    geometry_msgs::PoseStampedConstPtr pose = event.getConstMessage();
    current_pose = *pose;

}

void Daf_Controller::stateCallback(const nav_msgs::OdometryConstPtr& odom_state)
{
  latest_odom_ = odom_state;

  if (state < DRIVETO) return;

  this->updateRobotState(*latest_odom_);

  if(move_robot)
  {
  //calculate local path

  calc_local_path();

  //increase velocity if robot dose not move
  velocity_increase();

  if(follow_path_server_->isPreemptRequested() || !move_robot)
  {
    move_robot = false;
    reset();
    follow_path_server_->setPreempted();
    ROS_INFO("path execution is preempted");
    return;
  }
  update();

  }
}

void Daf_Controller::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg){
  tf::Quaternion imu_quaternion;
  tf::quaternionMsgToTF(imu_msg->orientation, imu_quaternion);
  tf::Transform imu_orientation(imu_quaternion, tf::Vector3(0,0,0));
  imu_orientation.getBasis().getEulerYPR(imu_yaw, imu_pitch, imu_roll);
}

void Daf_Controller::drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{
    geometry_msgs::PoseStampedConstPtr goal = event.getConstMessage();

    if (follow_path_server_->isActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      follow_path_server_->setPreempted(result, "drive to callback");
    }
    driveto(*goal, 0.0);
}

bool Daf_Controller::driveto(const geometry_msgs::PoseStamped& goal, double speed)
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

void Daf_Controller::drivepathCallback(const ros::MessageEvent<nav_msgs::Path>& event)
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

bool Daf_Controller::pathToBeSmoothed(const std::deque<geometry_msgs::PoseStamped>& transformed_path, bool fixed_path)
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

bool Daf_Controller::drivepath(const nav_msgs::Path& path)
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

        std::for_each(smooth_path.begin() + 1, smooth_path.end(), boost::bind(&Daf_Controller::addLeg, this, _1, options.desired_speed));

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

        curr_path = path2publish;
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


bool Daf_Controller::createDrivepath2MapTransform(tf::StampedTransform & transform, const nav_msgs::Path& path)
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

void Daf_Controller::cmd_velCallback(const geometry_msgs::Twist& velocity)
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

void Daf_Controller::cmd_velTeleopCallback(const geometry_msgs::Twist& velocity)
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

void Daf_Controller::speedCallback(const std_msgs::Float32& speed)
{
  mp_.commanded_speed = speed.data;
}

void Daf_Controller::stopVehicle()
{
  geometry_msgs::Vector3 p;
  robot_control_state.setControlState(0.0, p, 0, 0, mp_.carrot_distance, 0.0, false, true);
  vehicle_control_interface_->executeMotionCommand(robot_control_state);
}

void Daf_Controller::followPathGoalCallback()
{
  follow_path_goal_ = follow_path_server_->acceptNewGoal();
  if (follow_path_goal_->follow_path_options.reset_stuck_history)
  {
      stuck->reset();
  }

  curr_path = follow_path_goal_->target_path;

  drivepath(follow_path_goal_->target_path);
  drivepathPublisher.publish(follow_path_goal_->target_path);

  if(follow_path_goal_->follow_path_options.desired_speed != 0){
    lin_vel = follow_path_goal_->follow_path_options.desired_speed;
  }
  else{
    lin_vel = mp_.commanded_speed;
  }

  lin_vel_ref = lin_vel;

  psize = (int)curr_path.poses.size();

  if(psize > 0)
  {
      ROS_INFO("New Path Received from Action. Path seq: %i size: %i distance to plan path: %f", curr_path.header.seq, psize, mp_.carrot_distance);
      move_robot = true;

  }else{
      move_robot = false;

      //sends aborted feedback
      if (follow_path_server_->isActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::INVALID_MOTION_PLAN;
        follow_path_server_->setAborted(result, "Path size is 0");
      }
  }
}

void Daf_Controller::followPathPreemptCallback()
{
  stopVehicle();
  move_base_lite_msgs::FollowPathResult result;
  result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
  follow_path_server_->setPreempted(result, "preempt from incoming message to server");
  reset();
}

void Daf_Controller::addLeg(const geometry_msgs::PoseStamped& pose, double speed)
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

bool Daf_Controller::reverseAllowed()
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

bool Daf_Controller::reverseForced()
{
  return false;
  if (follow_path_server_->isActive()) {
      return follow_path_goal_->follow_path_options.reverse_forced;
  } else {
      return false;
  }


}

void Daf_Controller::reset()
{
  //ROS_INFO("RESET");
  state = INACTIVE;
  current = 0;
  final_twist_trials = 0;
  dt = 0.0;
  legs.clear();

  alignment_finished = true;
  co_unchanged = 0;
  st_point = 0;
  path_po_lenght = 0;
  //lin_vel = mp_.max_controller_speed/2;

  //save reference for linar velocity
  //lin_vel_ref = lin_vel;

  //rot_vel_dir = 1;
  //rot_dir_opti = 1;
  old_pos_x = 0;
  old_pos_y = 0;
  //rad = 0;
  err_cont = 0;
  //oscilation_rotation = 1;

}

/*****************************************************************************************************************
 * calculate Fitter Circular Path
 */
void Daf_Controller::calc_local_path()
{

  //start point from robot current pose
  points[0][0] = robot_control_state.pose.position.x;
  points[0][1] = robot_control_state.pose.position.y;

  //search for closest point on path
  double min_dif = 20.0;
  for(int i=0; i < psize; i++)
  {
    double po_dist = std::sqrt(std::pow(curr_path.poses[i].pose.position.x - points[0][0] , 2) + std::pow(curr_path.poses[i].pose.position.y - points[0][1], 2));
    if(po_dist < min_dif)
    {
      min_dif = po_dist;
      st_point = i;
    }
  }

  co_points = 0;
  path_po_lenght = 0;
  //calculate path_po_lenght -> number of waypoints in the carrot distance
  for(int i=st_point + 1; i < psize; i++)
  {
    double curr_dist = std::sqrt(std::pow(robot_control_state.pose.position.x - curr_path.poses[i].pose.position.x, 2) +
                                 std::pow(robot_control_state.pose.position.y - curr_path.poses[i].pose.position.y, 2));

    if(curr_dist > mp_.carrot_distance) //search for points
    {
      break;
    }
    else{
      path_po_lenght = path_po_lenght + 1;
    }
  }

  double angle_carrot = std::atan2(curr_path.poses[st_point + path_po_lenght].pose.position.y - robot_control_state.pose.position.y,
                                      curr_path.poses[st_point + path_po_lenght].pose.position.x - robot_control_state.pose.position.x);
  double angle_to_carrot = constrainAngle_mpi_pi(angle_carrot - yaw);

  for(int i=0; i <= path_po_lenght; i++){
    double angle_waypoint = std::atan2(curr_path.poses[st_point + i].pose.position.y - robot_control_state.pose.position.y,
                            curr_path.poses[st_point + i].pose.position.x - robot_control_state.pose.position.x);
    double angle_diff_carrot2waypoint = constrainAngle_mpi_pi(angle_carrot - angle_waypoint);
    //ROS_INFO("angle diff carrot2waypoint: %f", angle_diff_carrot2waypoint);
    if (fabs(angle_diff_carrot2waypoint) < M_PI_2){
      co_points = co_points + 1;
      points[co_points][0] = curr_path.poses[st_point + i].pose.position.x;
      points[co_points][1] = curr_path.poses[st_point + i].pose.position.y;
    }
  }


  th_po_x = 0; th_po_y = 0; fi_po_x = 0; fi_po_y = 0;
  se_po_x = 0; se_po_y = 0; dirx = 1; diry = -1; max_H = 0;

  if(co_points < 2){
    rot_vel_dir = 0;
    max_H = 0.001;
    rad = 99999;
    alignment_angle = atan2(curr_path.poses[st_point + co_points].pose.position.y - robot_control_state.pose.position.y,
        curr_path.poses[st_point + co_points].pose.position.x - robot_control_state.pose.position.x);
  }
  else{

    sideC = sqrt(((points[co_points][0] - points[0][0])*(points[co_points][0] - points[0][0])) + (points[co_points][1] - points[0][1])*(points[co_points][1] - points[0][1]));
    Wid = sideC;

    //calculate triangle height height
    for(int i=1; i < co_points; i++)
    {
      //p1            p2              p3
      //ROS_INFO("Points X: %f %f %f", points[0][0], points[i][0], points[co_points][0]);
      //ROS_INFO("Points Y: %f %f %f", points[0][1], points[i][1], points[co_points][1]);
      sideA = sqrt(((points[0][0] - points[i][0])*(points[0][0] - points[i][0])) + (points[0][1] - points[i][1])*(points[0][1] - points[i][1]));
      sideB = sqrt(((points[i][0] - points[co_points][0])*(points[i][0] - points[co_points][0])) + (points[i][1] - points[co_points][1])*(points[i][1] - points[co_points][1]));
      //ROS_INFO("triangle sides: %f %f %f", sideA, sideB, sideC);
      ss = (sideA + sideB + sideC)/2;
      area = sqrt(ss*(ss-sideA)*(ss-sideB)*(ss-sideC));
      //determine params for radius calculation
      tmp_H = (area*2)/sideC;

      if(tmp_H > max_H)
      {
        max_H = tmp_H;
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
    }

    //ROS_INFO("Mid Point: x: %f, y: %f", se_po_x, se_po_y);

    //calculate ground compensation, which modifiy max_H and W
    calc_ground_compensation();

    fi_po_x = points[0][0];
    fi_po_y = points[0][1];
    th_po_x = points[co_points][0];
    th_po_y = points[co_points][1];

    //calculate radious
    rad = max_H/2 + (Wid*Wid)/(8*max_H);

    ROS_INFO("max H: %f, Wid: %f, rad: %f", max_H, Wid, rad);

    //calculating circle center
    midX = (points[0][0] + points[co_points][0])/2;
    midY = (points[0][1] + points[co_points][1])/2;
    dx = (points[0][0] - points[co_points][0])/2;
    dy = (points[0][1] - points[co_points][1])/2;
    distt = sqrt(dx*dx + dy*dy);
    pdist = sqrt(rad*rad - distt*distt);
    mDx = dirx*dy*pdist/distt;
    mDy = diry*dx*pdist/distt;

    //calculate alignemnt angle
    double curr_dist_x = points[0][0] -  (midX + mDx);
    double curr_dist_y = points[0][1] - (midY + mDy);

    if(isinf(rad)){
      alignment_angle = atan2(points[co_points][1] - robot_control_state.pose.position.y,
          points[co_points][0] - robot_control_state.pose.position.x);
    }
    else{
      //correct angle directions
      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*PI/2;
    }
  }

  ROS_INFO("co points: %i, point y: %f, point x: %f", co_points, points[co_points][1], points[co_points][0]);
  ROS_INFO("st point: %i, path_po length: %i", st_point, path_po_lenght);

  //ROS_INFO("Fitted circle radius: %f", rad);

  //reduce angle on -PI to +PI
  if(alignment_angle > PI)
  {
    alignment_angle = alignment_angle - 2*PI;
  }
  if(alignment_angle < -PI)
  {
    alignment_angle = alignment_angle + 2*PI;
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

  //check if robot should drive backwards
  lin_vel_dir = 1;
  //ROS_INFO("angle to carrot: %f", angle_to_carrot);
  if (reverseAllowed()){
    if(fabs(angle_to_carrot) > M_PI/2){
      lin_vel_dir = -1;

      if(alignment_angle < 0){
        alignment_angle = alignment_angle + M_PI;
      }
      else{
        alignment_angle = alignment_angle - M_PI;
      }
    }
  }

  //send feedback
  //feedback.feedback = (int)round((100*st_point)/psize); //calculated progress relative to given path
  //execute_path_action_server_.publishFeedback(feedback);

  //display of all lines to plan a path
  if(show_trajectory_planing == true)
  {
    uint32_t shape = visualization_msgs::Marker::CUBE;

    local_calc_path.header.frame_id = map_frame_id;
    local_calc_path.poses.resize(360);

    for(int d= 0; d < 360; d++)
    {
      double xp = sin(d*0.0174532925)*rad + (midX + mDx);
      double yp = cos(d*0.0174532925)*rad + (midY + mDy);
      local_calc_path.poses[d].pose.position.x = xp;
      local_calc_path.poses[d].pose.position.y = yp;
      local_calc_path.poses[d].pose.position.z = 0;
    }
    //publish circle
    local_path_pub.publish(local_calc_path);

    //pub triangle
    calc_path.header.frame_id = map_frame_id;
    calc_path.poses.resize(4);

    //pub path
    calc_path.poses[0].pose.position.x = points[0][0];
    calc_path.poses[0].pose.position.y = points[0][1];
    calc_path.poses[0].pose.position.z = 0;
    calc_path.poses[1].pose.position.x = se_po_x;
    calc_path.poses[1].pose.position.y = se_po_y;
    calc_path.poses[1].pose.position.z = 0;
    calc_path.poses[2].pose.position.x = points[co_points][0];
    calc_path.poses[2].pose.position.y = points[co_points][1];
    calc_path.poses[2].pose.position.z = 0;
    calc_path.poses[3].pose.position.x = points[0][0];
    calc_path.poses[3].pose.position.y = points[0][1];
    calc_path.poses[3].pose.position.z = 0;
    calc_path.poses.resize(4);
    drivepathPublisher.publish(calc_path);

    //start point
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "first_point";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = fi_po_x;
    marker.pose.position.y = fi_po_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    // Publish the marker
    marker_pub.publish(marker);

    marker.ns = "second_point";
    marker.id = 2;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = se_po_x;
    marker.pose.position.y = se_po_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    // Publish the marker
    marker_pub.publish(marker);

    //end point
    marker.ns = "third_point";
    marker.id = 4;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = th_po_x;
    marker.pose.position.y = th_po_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    // Publish the marker
    marker_pub.publish(marker);
  }
}

/*****************************************************************************************************************
 * Compensate Ground Inclinations
 */
void Daf_Controller::calc_ground_compensation()
{
  if(enable_ground_compensation == true)
  {
    //ROS_INFO("Distances before ground compensation: max_H: %f and W: %f", max_H, Wid);
    //        double dh = fabs((max_H/cos(imu_roll))-max_H);
    //        double dw = fabs((Wid/cos(imu_pitch))-Wid);

    double dh = fabs((max_H/cos(roll))-max_H);
    double dw = fabs((Wid/cos(pitch))-Wid);
    //ROS_INFO("Angle compensation diferences: dh: %f, dw: %f", dh, dw);
    max_H = max_H + dh;
    Wid = Wid + dw;
    //ROS_INFO("Distances after ground compensation: max_H: %f and W: %f", max_H, Wid);
  }
}

/*****************************************************************************************************************
 * Check robot current stability
 */
void Daf_Controller::check_robot_stability()
{
  //if robot reaches treshodl of its stability angle it responds with error
  //if((imu_roll > stability_angle)||(imu_pitch > stability_angle))
  if((roll > stability_angle)||(pitch > stability_angle))
  {
    if (follow_path_server_->isActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::INVALID_MOTION_PLAN;
      follow_path_server_->setAborted(result, "Robot has exceeded angle of stability!");
    }
    reset();
    ROS_WARN("Robot has exceeded angle of stability!");
  }
}

/*****************************************************************************************************************
 * calculate Current Alignment Rotation
 */
void Daf_Controller::calculate_al_rot()
{
    //define rotation direction
    angle_diff = alignment_angle - yaw;

    if(fabs(angle_diff) > M_PI)
    {
        if(angle_diff > 0)
        {
             angle_diff = -2 * M_PI + fabs(angle_diff);
        }
        else
        {
            angle_diff = 2 * M_PI - fabs(angle_diff);
        }
    }

    if(angle_diff > 0){rot_dir_opti = 1;}
    else{ rot_dir_opti = -1;}
}

/*****************************************************************************************************************
 * Compensate Current Angle Difreence
 */
void Daf_Controller::calc_angel_compensation()
{
    if(enable_angle_compensation == true)
    {
        //cosider space
        angle_diff = alignment_angle - yaw;

        if(fabs(angle_diff) > M_PI)
        {
            if(angle_diff > 0)
            {
                 angle_diff = -2 * M_PI + fabs(angle_diff);
            }
            else
            {
                angle_diff = 2 * M_PI - fabs(angle_diff);
            }
        }

        //double add_al_rot = fabs(angle_diff/(execution_period));

        double add_al_rot = fabs(angle_diff*lin_vel);
        //ROS_INFO("Additional Alignment Rotation: %f", add_al_rot);

        rot_vel = rot_vel_dir * lin_vel/rad*rot_correction_factor + rot_dir_opti*add_al_rot*k_p_rotation;
    }else
    {
        rot_vel = rot_vel_dir * lin_vel/rad*rot_correction_factor;
 //upper and lower angle treshods are same
       lower_al_angle = upper_al_angle;
    }
    //ROS_INFO("radius: %f", rad);
}
/*****************************************************************************************************************
 * Increase Velocity if Robot Does not Move
 */
void Daf_Controller::velocity_increase()
{

    //check if there is no change in position it increase linear speed
    double pose_diff_x = fabs(robot_control_state.pose.position.x - old_pos_x);
    double pose_diff_y = fabs(robot_control_state.pose.position.y - old_pos_y);

    if((pose_diff_x <= std::numeric_limits<double>::epsilon()) && (pose_diff_y <= std::numeric_limits<double>::epsilon()))
    {
        ++co_unchanged;

        if (co_unchanged > update_skip)
        {
            co_unchanged = 0;

            if(enable_velocity_increase == true)
            {

                lin_vel = lin_vel + mp_.max_controller_speed/10;

                if(lin_vel >= mp_.max_controller_speed)
                {
                    lin_vel = mp_.max_controller_speed;

                    if (follow_path_server_->isActive()){
                      move_base_lite_msgs::FollowPathResult result;
                      result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
                      follow_path_server_->setAborted(result, "Robot cannot move! Maximum speed reached!");
                    }
                    reset();
                    ROS_WARN("Robot cannot move! Maximum speed reached!");
                }
            }else
            {
              if (follow_path_server_->isActive()){
                move_base_lite_msgs::FollowPathResult result;
                result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
                follow_path_server_->setAborted(result, "Robot cannot move!");
              }
              reset();
              ROS_WARN("Robot cannot move!");
            }
        }
    }
    else
    {
        //else reference linear velocity is used
        old_pos_x = robot_control_state.pose.position.x;
        old_pos_y = robot_control_state.pose.position.y;
        co_unchanged = 0;
        lin_vel = lin_vel_ref;
    }
}

/*****************************************************************************************************************
 * calculate and Publish Velocity Commands
 */
void Daf_Controller::update()
{
  if(move_robot == true)
  {
    double linear_tolerance_for_current_path = default_path_options_.goal_pose_position_tolerance;
    double angular_tolerance_for_current_path = default_path_options_.goal_pose_angle_tolerance;

    if (follow_path_server_->isActive()){
      if (follow_path_goal_->follow_path_options.goal_pose_position_tolerance > 0.0){
        linear_tolerance_for_current_path = follow_path_goal_->follow_path_options.goal_pose_position_tolerance;
      }

      if (follow_path_goal_->follow_path_options.goal_pose_angle_tolerance > 0.0){
        angular_tolerance_for_current_path = follow_path_goal_->follow_path_options.goal_pose_angle_tolerance;
      }
    }
    double goal_position_error =
            std::sqrt(
                std::pow(curr_path.poses.back().pose.position.x - robot_control_state.pose.position.x, 2)
              + std::pow(curr_path.poses.back().pose.position.y - robot_control_state.pose.position.y, 2));

    //If close to the goal point drive in a straight line
    if(goal_position_error < 0.2)
    {
      alignment_angle = atan2(curr_path.poses.back().pose.position.y - robot_control_state.pose.position.y,
                              curr_path.poses.back().pose.position.x - robot_control_state.pose.position.x);
      if(lin_vel_dir < 0){
        if(alignment_angle < 0){
          alignment_angle = alignment_angle + M_PI;
        }
        else{
          alignment_angle = alignment_angle - M_PI;
        }
      }
      rad = DBL_MAX;
    }

    //check if robot has reached a treshod for inclination
    check_robot_stability();

    ROS_INFO_ONCE("Start calculating cmd velocity!");
    //do algnment if angle is too large do alignment
    al_an_diff = alignment_angle - yaw;

    if(al_an_diff > M_PI)
    {
      al_an_diff = -2 * M_PI + al_an_diff;
    }
    else if (al_an_diff < -M_PI){
      al_an_diff = 2 * M_PI + al_an_diff;
    }

//    //check if robot should drive backwards
//    lin_vel_dir = 1;

//    if (reverseAllowed()){
//      if(fabs(al_an_diff) > M_PI/2){
//        lin_vel_dir = -1;

//        if(alignment_angle < 0){
//          alignment_angle = alignment_angle + M_PI;
//        }
//        else{
//          alignment_angle = alignment_angle - M_PI;
//        }

//        al_an_diff = alignment_angle - yaw;

//        if(al_an_diff > M_PI)
//        {
//          al_an_diff = -2 * M_PI + al_an_diff;
//        }
//        else if (al_an_diff < -M_PI){
//          al_an_diff = 2 * M_PI + al_an_diff;
//        }

//      }
//    }

    calculate_al_rot();

    // if difference is larger thatn both tresholds angle_correction and middle al_offset
    if((fabs(al_an_diff) > (upper_al_angle))||(alignment_finished == false))
    {
      // turn in place
      ROS_INFO("ROBOT IS ALIGNING || yaw: %f angle: %f al_an_diff: %f", yaw, alignment_angle, al_an_diff);
      //ROS_INFO("lower_al: %f upper_al: %f",lower_al_angle, upper_al_angle );

      cmd.linear.x = 0.0;
      cmd.angular.z = rot_dir_opti * (mp_.max_controller_angular_rate/3);

      if(fabs(al_an_diff) < upper_al_angle/2)
      {
        alignment_finished = true;
        ROS_INFO("Alignment completed!");
        err_cont = 0;
      }else
      {
        alignment_finished = false;

        //if robot misses alignment angle
        ++err_cont;
        if(err_cont > pub_cmd_hz*8) //second for delay
        {
          if (follow_path_server_->isActive()){
            move_base_lite_msgs::FollowPathResult result;
            result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
            follow_path_server_->setAborted(result, "robot misses alignment angle");
          }
          reset();
          ROS_WARN("If your robot is oscilating, please increase lower_al_angle and upper_al_angle parameter (recommended: lower_al_angle=0.6, upper_al_angle=1.0)");
          ROS_WARN("Requesting new path!");

        }
      }
    }
    // else if difference is between lower treshold (angle correction) and upper treshold (middle_al_offset) the angle compensation is used
    else if((fabs(al_an_diff) > lower_al_angle)&&((fabs(al_an_diff) < (upper_al_angle))))
    {
      ROS_INFO("DRIVE ROBOT MIDDLE STAGE || yaw: %f al_angle: %f", yaw, alignment_angle);

      //add additional rotation speed based on ground
      calc_angel_compensation();

      cmd.linear.x = lin_vel_dir * lin_vel;
      cmd.angular.z = rot_vel;

      ROS_INFO("cmd: lin: %f, ang: %f", cmd.linear.x, cmd.angular.z);

    }
    //if difference is below lower treshold ()
    else
    {
      ROS_INFO("DRIVE ROBOT || yaw: %f al_angle: %f", yaw, alignment_angle);

      rot_vel = rot_vel_dir * lin_vel/rad*rot_correction_factor;  //pazi za meso je dva krat

      cmd.linear.x = lin_vel_dir * lin_vel;
      cmd.angular.z = rot_vel;

      ROS_INFO("cmd: lin: %f, ang: %f, rad: %f, rot_veldir: %f, correct_fact: %f", cmd.linear.x, cmd.angular.z, rad, rot_vel_dir, rot_correction_factor);
    }

    //check for global goal proximitiy
    if(goal_position_error < linear_tolerance_for_current_path)
    {
      ROS_INFO("GLOBAL GOAL REACHED");
      cmd.linear.x = 0;
      cmd.angular.z = 0;
      stop();
      //goal reach reporting
      if (follow_path_server_->isActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
        follow_path_server_->setSucceeded(result, "reached goal");
      }
      move_robot = false;
      return;
    }

    //publish commad
    vehicle_control_interface_->executeTwist(cmd, robot_control_state, yaw, pitch, roll);
    //ROS_INFO("goal_position_error: %f, tolerance: %f", goal_position_error, linear_tolerance_for_current_path);
  }
}


void Daf_Controller::stop()
{
  this->vehicle_control_interface_->stop();
  stuck->reset();
  drivepathPublisher.publish(empty_path);
  if (camera_control)
    cameraOrientationPublisher.publish(cameraDefaultOrientation);
}

void Daf_Controller::controllerParamsCallback(vehicle_controller::DafControllerParamsConfig &config, uint32_t level){
  mp_.carrot_distance = config.lookahead_distance;
  k_p_rotation = config.kp_rot;
  lower_al_angle = config.lower_al_angle;
  upper_al_angle = config.upper_al_angle;

  show_trajectory_planing = config.show_trajectory_planning;
  if(show_trajectory_planing)
  {
    local_path_pub = nh.advertise<nav_msgs::Path>("/local_calc_path", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  }
}


