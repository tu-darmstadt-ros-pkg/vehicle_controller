#include <vehicle_controller/lqr_controller.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

Lqr_Controller::Lqr_Controller(ros::NodeHandle& nh_)
  : Controller(nh_), nh_dr_params("~/lqr_controller_params")
{
  nh = nh_;

  //LQR parameters
  lqr_q11 = 1000;
  lqr_q22 = 0;
  lqr_r = 1;
  rot_vel_dir = 1;
  lin_vel_dir = 1;

  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::LqrControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Lqr_Controller::controllerParamsCallback, this, _1, _2));

}

Lqr_Controller::~Lqr_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    dr_controller_params_server->clearCallback();
  }
}

void Lqr_Controller::reset()
{
  Controller::reset();
  lqr_y_error = 0;
  lqr_angle_error = 0;
  state = INACTIVE;
  current = 0;
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



  calc_local_path();
  calcLqr();
  //ROS_INFO("radius: %f", local_path_radius);

  double omega_ff = (lin_vel_dir * rot_vel_dir * robot_control_state.desired_velocity_linear / local_path_radius);

  double omega_fb = - lin_vel_dir *lqr_k1 * lqr_y_error - lqr_k2 * lqr_angle_error;

  double angular_vel= omega_ff + omega_fb;

  geometry_msgs::Twist cmd;
  cmd.linear.x = lin_vel_dir * fabs(robot_control_state.desired_velocity_linear) ;
  cmd.angular.z = angular_vel ;

  //send cmd to control interface
  vehicle_control_interface_->executeTwist(cmd, robot_control_state, yaw, pitch, roll);
  //ROS_INFO("lin vel: %f, ang vel: %f", cmd.linear.x, cmd.angular.z);

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

void Lqr_Controller::calc_local_path(){
  int next_point = calcClosestPoint();

  int psize = current_path.poses.size();

  double points[50][2];

  int st_point, co_points, path_po_lenght;
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

  //start point from closest point
  points[0][0] = closest_point.point.x;
  points[0][1] = closest_point.point.y;

  //search for closest point on path
  double min_dif = 20.0;
  for(int i=0; i < psize; i++)
  {
    double po_dist = std::sqrt(std::pow(current_path.poses[i].pose.position.x - points[0][0] , 2) + std::pow(current_path.poses[i].pose.position.y - points[0][1], 2));
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
    double curr_dist = std::sqrt(std::pow(closest_point.point.x - current_path.poses[i].pose.position.x, 2) +
                                 std::pow(closest_point.point.y - current_path.poses[i].pose.position.y, 2));

    if(curr_dist > mp_.carrot_distance) //search for points
    {
      break;
    }
    else{
      path_po_lenght = path_po_lenght + 1;
//      co_points = co_points + 1;
//      points[co_points][0] = current_path.poses[i].pose.position.x;
//      points[co_points][1] = current_path.poses[i].pose.position.y;
    }
  }

  double angle_carrot = std::atan2(current_path.poses[st_point + path_po_lenght].pose.position.y - closest_point.point.y,
                                      current_path.poses[st_point + path_po_lenght].pose.position.x - closest_point.point.x);

  for(int i=0; i <= path_po_lenght; i++){
    double angle_waypoint = std::atan2(current_path.poses[st_point + i].pose.position.y - closest_point.point.y,
                            current_path.poses[st_point + i].pose.position.x - closest_point.point.x);
    double angle_diff_carrot2waypoint = constrainAngle_mpi_pi(angle_carrot - angle_waypoint);
    //ROS_INFO("angle diff carrot2waypoint: %f", angle_diff_carrot2waypoint);
    if (fabs(angle_diff_carrot2waypoint) < M_PI_2){
      co_points = co_points + 1;
      points[co_points][0] = current_path.poses[st_point + i].pose.position.x;
      points[co_points][1] = current_path.poses[st_point + i].pose.position.y;
    }
  }

  th_po_x = 0; th_po_y = 0; fi_po_x = 0; fi_po_y = 0;
  se_po_x = 0; se_po_y = 0; dirx = 1; diry = -1; max_H = 0;

  if(co_points < 2){
    rot_vel_dir = 0;
    max_H = 0.001;
    local_path_radius = 99999;
    alignment_angle = atan2(current_path.poses[st_point + co_points].pose.position.y - closest_point.point.y,
                      current_path.poses[st_point + co_points].pose.position.x - closest_point.point.x);
  }
  else{

    sideC = sqrt(((points[co_points][0] - points[0][0])*(points[co_points][0] - points[0][0])) + (points[co_points][1] - points[0][1])*(points[co_points][1] - points[0][1]));

    Wid = sideC;
    //calculate triangle height height
    for(int i=0; i < co_points; i++)
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
    }


    //calculate ground compensation, which modifiy max_H and W
    //calc_ground_compensation();

    fi_po_x = points[0][0];
    fi_po_y = points[0][1];
    th_po_x = points[co_points][0];
    th_po_y = points[co_points][1];

    //calculate radious
    local_path_radius = max_H/2 + (Wid*Wid)/(8*max_H);
    //ROS_INFO("Fitted circle radius: %f", local_path_radius);

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
      alignment_angle = atan2(points[co_points][1] - closest_point.point.y,
          points[co_points][0] - closest_point.point.x);
    }
    else{
      alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*M_PI/2;
    }
  }

  //ROS_INFO("st_point: %i, co_points: %i, radius: %f", st_point, co_points, local_path_radius);
  //ROS_INFO("closest point: x: %f, y: %f", closest_point.point.x ,closest_point.point.y);

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

  double angle_carrot_to_robot = std::atan2(current_path.poses[st_point + path_po_lenght].pose.position.y - robot_control_state.pose.position.y,
                                      current_path.poses[st_point + path_po_lenght].pose.position.x - robot_control_state.pose.position.x);
  double angle_to_carrot = constrainAngle_mpi_pi(angle_carrot_to_robot - yaw);

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

}


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

  //ROS_INFO("closest: %i, second: %i", closest, second_closest);
  //ROS_INFO("closest: x: %f, y: %f", current_path.poses[closest].pose.position.x, current_path.poses[closest].pose.position.y);
  //ROS_INFO("second: x: %f, y: %f", current_path.poses[second_closest].pose.position.x, current_path.poses[second_closest].pose.position.y);

  if (closest > second_closest){
    return closest;
  }
  else{
    return second_closest;
  }

}


//compute the control parameters by solving the lqr optimization problem
void Lqr_Controller::calcLqr(){
  geometry_msgs::PointStamped closest_point_baseframe;

  //compute errors

  //angle error
  double angles[3];
  quaternion2angles(robot_control_state.pose.orientation, angles);

  lqr_angle_error =  constrainAngle_mpi_pi(angles[0] - alignment_angle);
  //ROS_INFO("yaw: %f, al_angle: %f", angles[0] , alignment_angle);

  //y position error
  double dist_2_closest_point = std::sqrt(std::pow(closest_point.point.x - robot_control_state.pose.position.x, 2)
                                          + std::pow(closest_point.point.y - robot_control_state.pose.position.y, 2));
  double angle_closest_point = atan2(closest_point.point.y - robot_control_state.pose.position.y,
                                     closest_point.point.x - robot_control_state.pose.position.x);
  double angle_to_closest_point = constrainAngle_mpi_pi(angles[0] - angle_closest_point);
  lqr_last_y_error = lqr_y_error;
  lqr_y_error = std::sin(angle_to_closest_point) * dist_2_closest_point;

  //compute control gains
  double v = fabs(robot_control_state.desired_velocity_linear);
  //double v = std::sqrt(std::pow(robot_control_state.velocity_linear.x, 2) + std::pow(robot_control_state.velocity_linear.y, 2)
  //                     + std::pow(robot_control_state.velocity_linear.z, 2));

  //ROS_INFO ("speed: %f", v);

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

//unused, simple numeric solver for the lqr omptimization problem
void Lqr_Controller::solveDare(){
  double epsilon = 0.001;

  double xicr = - robot_control_state.velocity_linear.y /robot_control_state.velocity_angular.z;
  if(isnan(xicr)){
    xicr = 0.0;
  }

  //ROS_INFO("xicr: %f", xicr);

  Eigen::Matrix<double, 2, 2> Q =  Eigen::Matrix<double, 2, 2>::Zero();
  Q(0,0) = 1000;
  Q(1,1) = 0.0;

  Eigen::Matrix<double, 2, 2> P = Eigen::Matrix<double, 2, 2>::Zero();

  Eigen::Matrix<double, 2, 2> P_new;

  Eigen::Matrix<double, 2, 2> A = Eigen::Matrix<double, 2, 2>::Zero();
  A(0,1) = fabs(robot_control_state.desired_velocity_linear);
  Eigen::Matrix<double, 2, 2> A_t = A.transpose();

  Eigen::Matrix<double, 2, 1> B = Eigen::Matrix<double, 2, 1>::Zero();
  B(0,0) =  0.0;
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
      //ROS_INFO("iterations: %i, p11: %f, p12: %f, p22: %f",i, P(0,0), P(0,1), P(1,1));
      K = 1/lqr_r * B_t*P;
      return;
    }

  }
  //ROS_INFO("max iterations, diff: %f, p11: %f, p12: %f, p22: %f",diff, P(0,0), P(0,1), P(1,1));
  K = 1/lqr_r * B_t*P;
  //ROS_INFO("k1: %f, k2:%f, k3: %f", K(0,0), K(0,1), K(0,2));

}

void Lqr_Controller::controllerParamsCallback(vehicle_controller::LqrControllerParamsConfig &config, uint32_t level){
  mp_.carrot_distance = config.lookahead_distance;
  lqr_q11 = config.Q11;
  lqr_q22 = config.Q22;
  lqr_r = config.R;
}
