#include <vehicle_controller/carrot_controller.h>

Carrot_Controller::Carrot_Controller(ros::NodeHandle& nh_)
  : Controller(nh_), nh_dr_params("~/carrot_controller_params")
{
  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::CarrotControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Carrot_Controller::controllerParamsCallback, this, _1, _2));
}

Carrot_Controller::~Carrot_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    dr_controller_params_server->clearCallback();
  }
}



void Carrot_Controller::update()
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

  vehicle_control_interface_->executeMotionCommand(robot_control_state);

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


void Carrot_Controller::controllerParamsCallback(vehicle_controller::CarrotControllerParamsConfig &config, uint32_t level){
  ROS_INFO_STREAM("HALLO" << mp_.carrot_distance << " " << config.carrot_distance);
  mp_.carrot_distance = config.carrot_distance;
  ROS_INFO_STREAM("HALLO2" << mp_.carrot_distance);
}
