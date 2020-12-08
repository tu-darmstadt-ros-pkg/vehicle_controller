#include <vehicle_controller/lqr_controller.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

Lqr_Controller::Lqr_Controller(ros::NodeHandle& nh_)
  : Controller(nh_), nh_dr_params("~/lqr_controller_params")
{
  //LQR parameters
  lqr_q11 = 1000;
  lqr_q22 = 0;
  lqr_r = 1;
  rot_vel_dir = 1;
  lin_vel_dir = 1;
}

Lqr_Controller::~Lqr_Controller()
{
  if(dr_controller_params_server){
    nh_dr_params.shutdown();
    dr_controller_params_server->clearCallback();
  }
}

bool Lqr_Controller::configure()
{
  Controller::configure();

  dr_controller_params_server = new dynamic_reconfigure::Server<vehicle_controller::LqrControllerParamsConfig>(nh_dr_params);
  dr_controller_params_server->setCallback(boost::bind(&Lqr_Controller::controllerParamsCallback, this, _1, _2));

  return true;
}

void Lqr_Controller::reset()
{
  Controller::reset();
  lqr_y_error = 0;
  lqr_angle_error = 0;
  state = INACTIVE;
  current = 0;
}


void Lqr_Controller::computeMoveCmd(){
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
