#include <vehicle_controller/daf_controller.h>


Daf_Controller::Daf_Controller(ros::NodeHandle& nh_)
  : Controller(nh_), nh_dr_params("~/daf_controller_params")
{
  if (followPathServerIsActive()) {
    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_goal_.setCanceled(result, "Controller changed");
  }
  follow_path_server_.reset(new actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>(nh, "/controller/follow_path",
                                                                                               boost::bind(&Controller::followPathGoalCallback, this, _1),
                                                                                               boost::bind(&Controller::followPathPreemptCallback, this, _1),
                                                                                               false));
  follow_path_server_->start();
}

Daf_Controller::~Daf_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    dr_controller_params_server->clearCallback();
  }
}

bool Daf_Controller::configure()
{
  Controller::configure();
  ros::NodeHandle params("~");
  params.param<double>("pub_cmd_hz", pub_cmd_hz, 30);
  params.param<double>("rot_correction_factor", rot_correction_factor, 1);
  params.param<double>("k_p_rotation", k_p_rotation, 5);

  params.param<double>("update_skip_until_vel_increase", update_skip, 5);
  params.param<double>("lower_al_angle", lower_al_angle, 0.2);              //this angle determine when angle correction is executed
  params.param<double>("upper_al_angle", upper_al_angle, 1.0);              //this angle determine middle robot correction wich is compensate in linear movment
  //smaller correction angle means better fiting of trajectories
  params.param("enable_angle_compensation", enable_angle_compensation, true);
  params.param("enable_ground_compensation", enable_ground_compensation, true);
  params.param("enable_velocity_increase", enable_velocity_increase, false);
  params.param("show_trajectory_planing", show_trajectory_planing, false);
  params.param<double>("stability_angle", stability_angle, 1.0);

  //path following visualization
  if(show_trajectory_planing)
  {
    local_path_pub = nh.advertise<nav_msgs::Path>("/local_calc_path", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  }

  stateSubscriber = nh.subscribe("state", 10, &Daf_Controller::stateCallback, this, ros::TransportHints().tcpNoDelay(true));

  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::DafControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Daf_Controller::controllerParamsCallback, this, _1, _2));

  return true;
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

    //increase velocity if robot does not move
    velocity_increase();

//    if(follow_path_server_->isPreemptRequested() || !move_robot)
//    {
//      move_robot = false;
//      reset();
//      follow_path_goal_.setCanceled();
//      return;
//    }
    update();
  }
}


void Daf_Controller::followPathGoalCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle goal)
{
  // Check if another goal exists
  if (followPathServerIsActive()) {
    // Check if new one is newer
    if (goal.getGoalID().stamp >= follow_path_goal_.getGoalID().stamp) {
      // Abort previous goal
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      follow_path_goal_.setCanceled(result, "This goal has been preempted by a newer goal.");
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
  if (follow_path_goal_.getGoal()->follow_path_options.reset_stuck_history)
  {
      stuck->reset();
  }

  current_path = follow_path_goal_.getGoal()->target_path;

  drivepath(follow_path_goal_.getGoal()->target_path);
  drivepathPublisher.publish(follow_path_goal_.getGoal()->target_path);

  if(follow_path_goal_.getGoal()->follow_path_options.desired_speed != 0){
    lin_vel = follow_path_goal_.getGoal()->follow_path_options.desired_speed;
  }
  else{
    lin_vel = mp_.commanded_speed;
  }

  lin_vel_ref = lin_vel;

  psize = (int)current_path.poses.size();

  if(psize > 0)
  {
      ROS_INFO("New Path Received from Action. Path seq: %i size: %i distance to plan path: %f", current_path.header.seq, psize, mp_.carrot_distance);
      move_robot = true;

  }else{
      move_robot = false;

      //sends aborted feedback
      if (followPathServerIsActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::INVALID_MOTION_PLAN;
        follow_path_goal_.setCanceled(result, "Path size is 0");
      }
  }
}

void Daf_Controller::reset()
{
  Controller::reset();

  alignment_finished = true;
  co_unchanged = 0;
  st_point = 0;
  path_po_lenght = 0;

  old_pos_x = 0;
  old_pos_y = 0;
  err_cont = 0;

}

/*****************************************************************************************************************
 * calculate Fittet Circular Path
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
    double curr_dist = std::sqrt(std::pow(robot_control_state.pose.position.x - current_path.poses[i].pose.position.x, 2) +
                                 std::pow(robot_control_state.pose.position.y - current_path.poses[i].pose.position.y, 2));

    if(curr_dist > mp_.carrot_distance) //search for points
    {
      break;
    }
    else{
      path_po_lenght = path_po_lenght + 1;
    }
  }

  double angle_carrot = std::atan2(current_path.poses[st_point + path_po_lenght].pose.position.y - robot_control_state.pose.position.y,
                                      current_path.poses[st_point + path_po_lenght].pose.position.x - robot_control_state.pose.position.x);
  double angle_to_carrot = constrainAngle_mpi_pi(angle_carrot - yaw);

  for(int i=0; i <= path_po_lenght; i++){
    double angle_waypoint = std::atan2(current_path.poses[st_point + i].pose.position.y - robot_control_state.pose.position.y,
                            current_path.poses[st_point + i].pose.position.x - robot_control_state.pose.position.x);
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
    rad = 99999;
    alignment_angle = atan2(current_path.poses[st_point + co_points].pose.position.y - robot_control_state.pose.position.y,
        current_path.poses[st_point + co_points].pose.position.x - robot_control_state.pose.position.x);
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

    //ROS_INFO("max H: %f, Wid: %f, rad: %f", max_H, Wid, rad);

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

  //ROS_INFO("co points: %i, point y: %f, point x: %f", co_points, points[co_points][1], points[co_points][0]);
  //ROS_INFO("st point: %i, path_po length: %i", st_point, path_po_lenght);

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
  if(enable_ground_compensation)
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
    if (followPathServerIsActive()){
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::INVALID_MOTION_PLAN;
      follow_path_goal_.setCanceled(result, "Robot has exceeded angle of stability!");
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
    if(enable_angle_compensation)
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

            if(enable_velocity_increase)
            {

                lin_vel = lin_vel + mp_.max_controller_speed/10;

                if(lin_vel >= mp_.max_controller_speed)
                {
                    lin_vel = mp_.max_controller_speed;

                    if (followPathServerIsActive()){
                      move_base_lite_msgs::FollowPathResult result;
                      result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
                      follow_path_goal_.setCanceled(result, "Robot cannot move! Maximum speed reached!");
                    }
                    reset();
                    ROS_WARN("Robot cannot move! Maximum speed reached!");
                }
            }else
            {
              if (followPathServerIsActive()){
                move_base_lite_msgs::FollowPathResult result;
                result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
                follow_path_goal_.setCanceled(result, "Robot cannot move!");
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
void Daf_Controller::computeMoveCmd()
{
  if(move_robot)
  {
    double linear_tolerance_for_current_path = default_path_options_.goal_pose_position_tolerance;
    double angular_tolerance_for_current_path = default_path_options_.goal_pose_angle_tolerance;

    if (followPathServerIsActive()){
      if (follow_path_goal_.getGoal()->follow_path_options.goal_pose_position_tolerance > 0.0){
        linear_tolerance_for_current_path = follow_path_goal_.getGoal()->follow_path_options.goal_pose_position_tolerance;
      }

      if (follow_path_goal_.getGoal()->follow_path_options.goal_pose_angle_tolerance > 0.0){
        angular_tolerance_for_current_path = follow_path_goal_.getGoal()->follow_path_options.goal_pose_angle_tolerance;
      }
    }
    double goal_position_error =
            std::sqrt(
                std::pow(current_path.poses.back().pose.position.x - robot_control_state.pose.position.x, 2)
              + std::pow(current_path.poses.back().pose.position.y - robot_control_state.pose.position.y, 2));

    //If close to the goal point drive in a straight line
    if(goal_position_error < 0.2)
    {
      alignment_angle = atan2(current_path.poses.back().pose.position.y - robot_control_state.pose.position.y,
                              current_path.poses.back().pose.position.x - robot_control_state.pose.position.x);
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


    calculate_al_rot();

    // if difference is larger thatn both tresholds angle_correction and middle al_offset
    if((fabs(al_an_diff) > (upper_al_angle))||(!alignment_finished))
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
          if (followPathServerIsActive()){
            move_base_lite_msgs::FollowPathResult result;
            result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
            follow_path_goal_.setCanceled(result, "robot misses alignment angle");
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
      //ROS_INFO("DRIVE ROBOT MIDDLE STAGE || yaw: %f al_angle: %f", yaw, alignment_angle);

      //add additional rotation speed based on ground
      calc_angel_compensation();

      cmd.linear.x = lin_vel_dir * lin_vel;
      cmd.angular.z = rot_vel;

      //ROS_INFO("cmd: lin: %f, ang: %f", cmd.linear.x, cmd.angular.z);

    }
    //if difference is below lower treshold ()
    else
    {
      //ROS_INFO("DRIVE ROBOT || yaw: %f al_angle: %f", yaw, alignment_angle);

      rot_vel = rot_vel_dir * lin_vel/rad*rot_correction_factor;  //pazi za meso je dva krat

      cmd.linear.x = lin_vel_dir * lin_vel;
      cmd.angular.z = rot_vel;

      //ROS_INFO("cmd: lin: %f, ang: %f, rad: %f, rot_veldir: %f, correct_fact: %f", cmd.linear.x, cmd.angular.z, rad, rot_vel_dir, rot_correction_factor);
    }

    //check for global goal proximitiy
    if(goal_position_error < linear_tolerance_for_current_path)
    {
      cmd.linear.x = 0;
      cmd.angular.z = 0;
      stop();
      //goal reach reporting
      if (followPathServerIsActive()){
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
        follow_path_goal_.setSucceeded(result, "reached goal");
      }
      move_robot = false;
      return;
    }

    //publish commad
    vehicle_control_interface_->executeTwist(cmd, robot_control_state, yaw, pitch, roll);
    //ROS_INFO("goal_position_error: %f, tolerance: %f", goal_position_error, linear_tolerance_for_current_path);
  }
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


