#include <vehicle_controller/controller.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <limits>

#include "ps3d.h"
#include "quaternions.h"

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>

#include <hector_move_base_msgs/move_base_action.h>
#include <vehicle_controller/four_wheel_steer_controller.h>
#include <vehicle_controller/differential_drive_controller.h>

#include <algorithm>
#include <sstream>

#define END_TWIST

static double angular_norm(double diff)
{
    static const double M_2PI = 2.0 * M_PI;
    diff -= floor(diff/M_2PI + .5)*M_2PI;
    return diff;
}

float euclideanDistance(geometry_msgs::Point const & p0, geometry_msgs::Point const & p1)
{
  return sqrt(pow(p1.x - p0.x, 2) + pow(p1.y - p0.y, 2) + pow(p1.z - p0.z, 2));
}

Controller::Controller(const std::string& ns)
    : nh(ns)
    , state(INACTIVE)
{
    motion_control_setup.carrot_distance = 1.0;
    motion_control_setup.current_speed = 0.0;
    motion_control_setup.min_speed = 0.1;
    motion_control_setup.max_speed = 1.0;
    motion_control_setup.inclination_speed_reduction_factor = 0.5 / (30 * M_PI/180.0); // 0.5 per 30 degrees
    motion_control_setup.inclination_speed_reduction_time_constant = 0.3;
    map_frame_id = "nav";
    base_frame_id = "base_link";

    camera_control = false;
    camera_lookat_distance = 1.0;
    camera_lookat_height = -0.2;

    check_if_blocked = true;
    velocity_blocked_time = 5.0;
    velocity_blocked_limit = 0.2;

    motion_control_setup.current_velocity = 0.0;
    motion_control_setup.current_inclination = 0.0;
    velocity_error = 0.0;

    goal_position_tolerance = 0.0;
    goal_angle_tolerance = 0.0;
    alternative_goal_position_tolerance = 0.0;
    alternative_angle_tolerance = 0.0;

    cameraDefaultOrientation.header.frame_id = "base_stabilized";
    tf::Quaternion cameraOrientationQuaternion;
    cameraOrientationQuaternion.setEuler(0.0, 25.0*M_PI/180.0, 0.0);
    tf::quaternionTFToMsg(cameraOrientationQuaternion, cameraDefaultOrientation.quaternion);
}

Controller::~Controller()
{
}

bool Controller::configure()
{
    ros::NodeHandle params("~");
    params.getParam("carrot_distance", motion_control_setup.carrot_distance);
    params.getParam("min_speed", motion_control_setup.min_speed);
    params.getParam("max_speed", motion_control_setup.max_speed);
    params.getParam("speed", motion_control_setup.current_speed);
    params.getParam("frame_id", map_frame_id);
    params.getParam("base_frame_id", base_frame_id);
    params.getParam("camera_control", camera_control);
    params.getParam("camera_lookat_distance", camera_lookat_distance);
    params.getParam("camera_lookat_height", camera_lookat_height);
    params.getParam("check_if_blocked", check_if_blocked);
    params.getParam("velocity_blocked_time", velocity_blocked_time);
    params.getParam("velocity_blocked_limit", velocity_blocked_limit);
    params.getParam("inclination_speed_reduction_factor", motion_control_setup.inclination_speed_reduction_factor);
    params.getParam("inclination_speed_reduction_time_constant", motion_control_setup.inclination_speed_reduction_time_constant);
    params.getParam("goal_position_tolerance", goal_position_tolerance);
    params.getParam("goal_angle_tolerance", goal_angle_tolerance);
    params.getParam("vehicle_type", vehicle_type);

    std::string vehicle_plugin = "differential_steering";
    params.getParam("vehicle_control_type", vehicle_plugin);

    if (vehicle_plugin == "differential_steering")
    {
        this->vehicle_control_interface_.reset(new DifferentialDriveController());
    }
    else
    {
        this->vehicle_control_interface_.reset(new FourWheelSteerController());
    }

    this->vehicle_control_interface_->configure(params, &motion_control_setup);


    ROS_INFO("Loaded %s as low level vehicle motion controller", this->vehicle_control_interface_->getName().c_str());

    stateSubscriber     = nh.subscribe("state", 10, &Controller::stateCallback, this);
    drivetoSubscriber   = nh.subscribe("driveto", 10, &Controller::drivetoCallback, this);
    drivepathSubscriber = nh.subscribe("drivepath", 10, &Controller::drivepathCallback, this);
    cmd_velSubscriber   = nh.subscribe("cmd_vel", 10, &Controller::cmd_velCallback, this);
    speedSubscriber     = nh.subscribe("speed", 10, &Controller::speedCallback, this);


    carrotPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("carrot", 1, true);
    drivepathPublisher  = nh.advertise<nav_msgs::Path>("drivepath", 1, true);

    pathPosePublisher = nh.advertise<nav_msgs::Path>("smooth_path", 1, true);

    diagnosticsPublisher = params.advertise<std_msgs::Float32>("velocity_error", 1, true);

    // action interface
    ros::NodeHandle action_nh("controller");
    actionSubscriber      = action_nh.subscribe("generic", 10, &Controller::actionCallback, this);
    actionGoalSubscriber  = action_nh.subscribe("goal", 10, &Controller::actionGoalCallback, this);
    actionPathSubscriber  = action_nh.subscribe("path", 10, &Controller::actionPathCallback, this);
    actionResultPublisher = action_nh.advertise<hector_move_base_msgs::MoveBaseActionResult>("result", 1);

    // alternative_tolerances_service
    alternative_goal_position_tolerance = goal_position_tolerance;
    alternative_angle_tolerance = goal_angle_tolerance;
    alternative_tolerances_service = action_nh.advertiseService("set_alternative_tolerances", &Controller::alternativeTolerancesService, this);

    if (camera_control)
    {
        cameraOrientationPublisher = nh.advertise<geometry_msgs::QuaternionStamped>("camera/command", 1);
        lookatPublisher = nh.advertise<geometry_msgs::PointStamped>("camera/look_at", 1);
        cameraOrientationPublisher.publish(cameraDefaultOrientation);
    }

    empty_path.header.frame_id = map_frame_id;

    return true;
}

void Controller::stateCallback(const nav_msgs::Odometry& state)
{
    dt = (state.header.stamp - this->pose.header.stamp).toSec();
    if (dt < 0.0 || dt > 1.0) dt = 0.0;

    this->pose.header = state.header;
    this->pose.pose = state.pose.pose;
    this->velocity.header = state.header;
    this->velocity.vector = state.twist.twist.linear;

    try
    {
        listener.waitForTransform(this->map_frame_id, state.header.frame_id, state.header.stamp, ros::Duration(3.0));
        listener.waitForTransform(this->base_frame_id, state.header.frame_id, state.header.stamp, ros::Duration(3.0));
        listener.transformPose(this->map_frame_id, this->pose, this->pose);
        listener.transformVector(this->base_frame_id, this->velocity, this->velocity);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    motion_control_setup.current_velocity = this->velocity.vector.x;
    double inclination = acos(this->pose.pose.orientation.w*this->pose.pose.orientation.w - this->pose.pose.orientation.x*this->pose.pose.orientation.x - this->pose.pose.orientation.y*this->pose.pose.orientation.y + this->pose.pose.orientation.z*this->pose.pose.orientation.z);
    if (inclination >= motion_control_setup.current_inclination)
        motion_control_setup.current_inclination = inclination;
    else
        motion_control_setup.current_inclination = (motion_control_setup.inclination_speed_reduction_time_constant * motion_control_setup.current_inclination + dt * inclination) / (motion_control_setup.inclination_speed_reduction_time_constant + dt);

    update();
}

void Controller::drivetoCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event)
{
    geometry_msgs::PoseStampedConstPtr goal = event.getConstMessage();

    publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new goal");
    driveto(*goal);
}

bool Controller::driveto(const geometry_msgs::PoseStamped& goal)
{
    reset();

    geometry_msgs::PoseStamped goal_transformed;
    try{
        listener.waitForTransform(this->map_frame_id, goal.header.frame_id, goal.header.stamp, ros::Duration(3.0));
        listener.transformPose(this->map_frame_id, goal, goal_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        stop();
        publishActionResult(actionlib_msgs::GoalStatus::REJECTED);
        return false;
    }

    start = pose.pose;
    addLeg(goal_transformed.pose);
    state = DRIVETO;

    ROS_INFO("Received new goal point (x = %.2f, y = %.2f)", goal_transformed.pose.position.x, goal_transformed.pose.position.y);
    publishActionResult(actionlib_msgs::GoalStatus::ACTIVE);
    return true;
}

void Controller::drivepathCallback(const ros::MessageEvent<nav_msgs::Path>& event)
{
    if (event.getPublisherName() == ros::this_node::getName()) return;
    nav_msgs::PathConstPtr path = event.getConstMessage();

    publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new path");
    drivepath(*path);
}

bool Controller::drivepath(const nav_msgs::Path& path)
{
    reset();

    if (path.poses.size() == 0) {
        ROS_DEBUG("Received empty path");
        stop();
        publishActionResult(actionlib_msgs::GoalStatus::SUCCEEDED);
        return false;
    }

    if (path.poses.size() == 1) {
        ROS_WARN("Received path with only one pose, ignoring as this lead to crashes");
        stop();
        publishActionResult(actionlib_msgs::GoalStatus::SUCCEEDED);
        return false;
    }

    tf::StampedTransform transform;
    if (!path.header.frame_id.empty()) {
        try {
            listener.waitForTransform(this->map_frame_id, path.header.frame_id, path.header.stamp, ros::Duration(3.0));
            listener.lookupTransform(this->map_frame_id, path.header.frame_id, path.header.stamp, transform);
        }
        catch (tf::TransformException ex) {
            ROS_INFO("TRAFO in Monstertruck Controller failed.");
            ROS_ERROR("%s", ex.what());
            stop();
            publishActionResult(actionlib_msgs::GoalStatus::REJECTED);
            return false;
        }
    } else {
        ROS_WARN("Received a path with empty frame_id. Assuming %s frame.", map_frame_id.c_str());
        transform.setIdentity();
    }

    // compute transformation of waypoints
    std::deque<geometry_msgs::Pose> transformedPath;
    for(unsigned i = 0; i < path.poses.size(); ++i)
    {
        tf::Pose tf_waypoint;
        geometry_msgs::Pose transformed_waypoint;
        tf::poseMsgToTF(path.poses[i].pose, tf_waypoint);
        tf_waypoint = transform * tf_waypoint;
        tf::poseTFToMsg(tf_waypoint, transformed_waypoint);
        transformedPath.push_back(transformed_waypoint);
    }


    // check if this path shall be smoothed
    // <=> path is output of exploration planner
    //     and hence, the points lie in the centers
    //     of neighbouring grid cells.
    float lu = 0.05 - 1e-5;
    float lo = sqrt(2.0) * 0.05 + 1e-5;
    bool path_to_be_smoothed = path.poses.size() > 2;
    std::stringstream sstr;
    for(unsigned i = 1; path_to_be_smoothed && i < transformedPath.size() - 1; ++i)
    {
        float d = euclideanDistance(transformedPath[i].position,transformedPath[i - 1].position);
        path_to_be_smoothed = path_to_be_smoothed && lu < d && d < lo;
        sstr << " " << d << ":" << (lu < d && d < lo ? "T" : "F");
    }
    //ROS_INFO("%s\n",sstr.str().c_str());

    if(path_to_be_smoothed) //
    //  if(false) //
    {
        std::vector<geometry_msgs::Pose> smoothedPoses;
        this->start = transformedPath[0];
        this->start.orientation = this->pose.pose.orientation;

        // bool allow_reverse = vehicle_type == "tracked";
        bool allow_reverse = false;
        Pathsmoother3D ps3d(allow_reverse);

        deque_vec3 in_path;
        quat in_start_orientation;
        quat in_end_orientation;

        vector_vec3 out_smoothed_positions;
        vector_quat out_smoothed_orientations;

        in_path.resize(transformedPath.size());
        std::transform(transformedPath.begin(), transformedPath.end(), in_path.begin(),
                       [](geometry_msgs::Pose const & pose_) -> vec3
        {
            return vec3(pose_.position.x, pose_.position.y, pose_.position.z);
        });

        in_start_orientation = quat(this->pose.pose.orientation.w, this->pose.pose.orientation.x,
                                    this->pose.pose.orientation.y, this->pose.pose.orientation.z);
        in_end_orientation = quat(transformedPath.back().orientation.w, transformedPath.back().orientation.x,
                                  transformedPath.back().orientation.y, transformedPath.back().orientation.z);

        ps3d.smooth(in_path, in_start_orientation, in_end_orientation, out_smoothed_positions, out_smoothed_orientations, false);

        smoothedPoses.resize(out_smoothed_positions.size());
        std::transform(out_smoothed_positions.begin(), out_smoothed_positions.end(),
                       out_smoothed_orientations.begin(), smoothedPoses.begin(),
                       [transform](vec3 const & position, quat const & orientation) -> geometry_msgs::Pose
        {
            geometry_msgs::Pose pose;
            pose.position.x = position(0);
            pose.position.y = position(1);
            pose.position.z = position(2);
            pose.orientation.w = orientation.w();
            pose.orientation.x = orientation.x();
            pose.orientation.y = orientation.y();
            pose.orientation.z = orientation.z();
            return pose;
        });

        for(unsigned i = 1; i < smoothedPoses.size(); ++i)
            addLeg(smoothedPoses[i]);

        nav_msgs::Path path2publish;
        path2publish.header.frame_id = map_frame_id;
        path2publish.header.stamp = ros::Time::now();
        for(unsigned i = 0; i < smoothedPoses.size(); ++i)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = path2publish.header;
            ps.pose = smoothedPoses[i];
            path2publish.poses.push_back(ps);
        }
        pathPosePublisher.publish(path2publish);
    }
    else
    {
        for(nav_msgs::Path::_poses_type::const_iterator waypoint = path.poses.begin(); waypoint != path.poses.end(); ++waypoint)
        {
            tf::Pose tf_waypoint;
            geometry_msgs::Pose transformed_waypoint;
            tf::poseMsgToTF(waypoint->pose, tf_waypoint);
            tf_waypoint = transform * tf_waypoint;
            tf::poseTFToMsg(tf_waypoint, transformed_waypoint);
            if (waypoint == path.poses.begin())
            {
                this->start = transformed_waypoint;
                this->start.orientation = this->pose.pose.orientation;
            }
            else
            {
                addLeg(transformed_waypoint);
            }
        }
        nav_msgs::Path path;
        path.header.frame_id = map_frame_id;
        path.header.stamp = ros::Time::now();
        pathPosePublisher.publish(path);
    }

    state = DRIVEPATH;
    //@TODO The below crashed when a path with only one entry has been received. See check on top of this function.
    ROS_INFO("Received new path to goal point (x = %.2f, y = %.2f)", legs.back().p2.x, legs.back().p2.y);
    publishActionResult(actionlib_msgs::GoalStatus::ACTIVE);
    return true;
}

void Controller::cmd_velCallback(const geometry_msgs::Twist& velocity)
{
    publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a velocity command");
    reset();

    if (velocity.linear.x == 0.0) {
        state = INACTIVE;
    } else {
        state = VELOCITY;
    }

    vehicle_control_interface_->executeTwist(velocity);
}

void Controller::speedCallback(const std_msgs::Float32& speed) {
    motion_control_setup.current_speed = speed.data;
}

bool Controller::alternativeTolerancesService(monstertruck_msgs::SetAlternativeTolerance::Request& req,
                                              monstertruck_msgs::SetAlternativeTolerance::Response& res)
{
    ROS_DEBUG("[monstertruck_controller]: tolerance service");
    this->alternative_tolerance_goalID.reset(new actionlib_msgs::GoalID(req.goalID));
    alternative_goal_position_tolerance = req.linearTolerance;
    alternative_angle_tolerance = req.angularTolerance;
    return true;
}

void Controller::actionCallback(const hector_move_base_msgs::MoveBaseActionGeneric& action)
{
    publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new action");
    this->goalID.reset(new actionlib_msgs::GoalID(action.goal_id));

    hector_move_base_msgs::MoveBasePathPtr path_action = hector_move_base_msgs::getAction<hector_move_base_msgs::MoveBasePath>(action);
    if (path_action) {
        drivepath(path_action->target_path);
        drivepathPublisher.publish(path_action->target_path);
    }

    hector_move_base_msgs::MoveBaseGoalPtr goal_action = hector_move_base_msgs::getAction<hector_move_base_msgs::MoveBaseGoal>(action);
    if (goal_action) {
        driveto(goal_action->target_pose);
        drivepathPublisher.publish(empty_path); // publish empty path
    }
}

void Controller::actionGoalCallback(const hector_move_base_msgs::MoveBaseActionGoal& goal_action)
{
    publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new goal");
    this->goalID.reset(new actionlib_msgs::GoalID(goal_action.goal_id));

    driveto(goal_action.goal.target_pose);
    drivepathPublisher.publish(empty_path); // publish empty path
}

void Controller::actionPathCallback(const hector_move_base_msgs::MoveBaseActionPath& path_action)
{
    publishActionResult(actionlib_msgs::GoalStatus::PREEMPTED, "received a new path");
    this->goalID.reset(new actionlib_msgs::GoalID(path_action.goal_id));

    drivepath(path_action.goal.target_path);
    drivepathPublisher.publish(path_action.goal.target_path);
}

void Controller::publishActionResult(actionlib_msgs::GoalStatus::_status_type status, const std::string& text) {
    if (!goalID) return;

    hector_move_base_msgs::MoveBaseActionResult result;
    result.header.stamp = this->pose.header.stamp;
    result.status.goal_id = *goalID;
    result.status.status = status;
    result.status.text = text;

    actionResultPublisher.publish(result);

    if (status != actionlib_msgs::GoalStatus::ACTIVE && status != actionlib_msgs::GoalStatus::PENDING)
        goalID.reset();
}

void Controller::addLeg(geometry_msgs::Pose const& pose)
{
    Leg leg;
    double angles[3];

    leg.p2.x = pose.position.x;
    leg.p2.y = pose.position.y;

    if (legs.size() == 0) {
        leg.p1.x = start.position.x;
        leg.p1.y = start.position.y;
        leg.course = atan2(leg.p2.y - leg.p1.y, leg.p2.x - leg.p1.x);

        if (start.orientation.w == 0.0 && start.orientation.x == 0.0 && start.orientation.y == 0 && start.orientation.z == 0.0) {
            leg.p1.orientation = leg.course;
        } else {
            quaternion2angles(start.orientation, angles);
            leg.p1.orientation = angles[0];
        }

    } else {
        const Leg& last = legs.back();
        leg.p1.x = last.p2.x;
        leg.p1.y = last.p2.y;
        leg.p1.orientation = last.p2.orientation;
        leg.course = atan2(leg.p2.y - leg.p1.y, leg.p2.x - leg.p1.x);
    }

    leg.backward = fabs(angular_norm(leg.course - leg.p1.orientation)) > M_PI_2;
    if (pose.orientation.w == 0.0 && pose.orientation.x == 0.0 && pose.orientation.y == 0 && pose.orientation.z == 0.0) {
        leg.p2.orientation = !leg.backward ? leg.course : angular_norm(leg.course + M_PI);
    } else {
        quaternion2angles(pose.orientation, angles);
        leg.p2.orientation = angles[0];
    }

    leg.speed = motion_control_setup.current_speed;
    leg.length2 = (leg.p2.x-leg.p1.x)*(leg.p2.x-leg.p1.x) + (leg.p2.y-leg.p1.y)*(leg.p2.y-leg.p1.y);
    leg.length = sqrt(leg.length2);
    leg.percent = 0.0;

    //  std::cout << "Leg " << legs.size() << ":" << std::endl;
    //  std::cout << "  length:    " << leg.length << std::endl;
    //  std::cout << "  course:    " << (leg.course*180.0/M_PI) << std::endl;
    //  std::cout << "  direction: " << (leg.backward ? "backward" : "forward") << std::endl;

    if (leg.length2 == 0.0) return;
    legs.push_back(leg);
}

void Controller::reset()
{
    state = INACTIVE;
    current = 0;
    dt = 0.0;
    legs.clear();
}

void Controller::update()
{
    if (state < DRIVETO) return;

    // get current orientation
    double angles[3];
    quaternion2angles(pose.pose.orientation, angles);

    double linear_tolerance_for_current_path = goal_position_tolerance;
    double angular_tolerance_for_current_path = goal_angle_tolerance;

    // Check if alternative tolerance is set by Service
    if (alternative_tolerance_goalID && goalID && (alternative_tolerance_goalID->id == goalID->id)) {
        ROS_DEBUG("[monstertruck_controller]: goalIDs are equal, using alternative tolerance.");
        linear_tolerance_for_current_path = alternative_goal_position_tolerance;
        angular_tolerance_for_current_path = alternative_angle_tolerance;
    }

    // Check if goal has been reached based an goal_position_tolerance/goal_angle_tolerance
    double goal_position_error = sqrt(pow(legs.back().p2.x - pose.pose.position.x, 2) + pow(legs.back().p2.y - pose.pose.position.y, 2));
    double goal_angle_error_   = angular_norm(legs.back().p2.orientation - angles[0]);
    if (goal_position_error < linear_tolerance_for_current_path && goal_angle_error_ < angular_tolerance_for_current_path) {
        ROS_INFO("Current position and orientation are within goal tolerance.");
        current = legs.size();
        // reached goal point handled by the following loop
    }

    // calculate projection
    while(1)
    {
        if (current == legs.size()) {
            state = INACTIVE;
            ROS_INFO("Reached goal point!");

#ifdef END_TWIST
            double angles[3];
            quaternion2angles(pose.pose.orientation, angles);
            double goal_angle_error_   = angular_norm(legs.back().p2.orientation - angles[0]);
#endif

#ifdef END_TWIST
            if(goal_angle_error_ < goal_angle_tolerance / 2.0)
            {
#endif
                stop();
                publishActionResult(actionlib_msgs::GoalStatus::SUCCEEDED);
                return;
#ifdef END_TWIST
            }
            else
            {
                geometry_msgs::Twist twist;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = 0.5;
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = 0;
                this->vehicle_control_interface_->executeTwist(twist);
                return;
            }
#endif
        }

        legs[current].percent = ((pose.pose.position.x - legs[current].p1.x) * (legs[current].p2.x - legs[current].p1.x)
                                 + (pose.pose.position.y - legs[current].p1.y) * (legs[current].p2.y - legs[current].p1.y))
                / legs[current].length2;
        ROS_DEBUG("Robot has passed %.1f percent of leg %u.", legs[current].percent, current);
        if (legs[current].percent < 1.0) break;

        ++current;
        ROS_DEBUG("Robot reached waypoint %d", current);
    }

    // calculate carrot
    Point carrot;
    unsigned int carrot_waypoint = current;
    float carrot_percent = legs[current].percent;
    float carrot_remaining = motion_control_setup.carrot_distance;

    while(carrot_waypoint < legs.size()) {
        if (carrot_remaining <= (1.0f - carrot_percent) * legs[carrot_waypoint].length) {
            carrot_percent += carrot_remaining / legs[carrot_waypoint].length;
            break;
        }

        carrot_remaining -= (1.0f - carrot_percent) * legs[carrot_waypoint].length;
        if (carrot_waypoint+1 < legs.size() && legs[carrot_waypoint].backward == legs[carrot_waypoint+1].backward) {
            ROS_DEBUG("Carrot reached waypoint %d", carrot_waypoint);
            carrot_percent = 0.0f;
            carrot_waypoint++;
        } else {
            ROS_DEBUG("Carrot reached last waypoint or change of direction");
            carrot_percent = 1.0f + carrot_remaining / legs[carrot_waypoint].length;
            break;
        }
    }

    carrot.x           = (1.0f - carrot_percent) * legs[carrot_waypoint].p1.x + carrot_percent * legs[carrot_waypoint].p2.x;
    carrot.y           = (1.0f - carrot_percent) * legs[carrot_waypoint].p1.y + carrot_percent * legs[carrot_waypoint].p2.y;
    // carrot.orientation = legs[carrot_waypoint].p1.orientation + std::min(carrot_percent, 1.0f) * angular_norm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);

    if (carrot_waypoint == (legs.size()-1) ){
        carrot.orientation = legs[carrot_waypoint].p1.orientation + std::min(carrot_percent, 1.0f) * angular_norm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);
    }else{
        carrot.orientation = legs[carrot_waypoint].p1.orientation + carrot_percent * angular_norm(legs[carrot_waypoint].p2.orientation - legs[carrot_waypoint].p1.orientation);
    }

    if (carrotPosePublisher) {
        carrotPose.header = pose.header;
        carrotPose.pose.position.x = carrot.x;
        carrotPose.pose.position.y = carrot.y;
        double ypr[3] = { carrot.orientation, 0, 0 };
        angles2quaternion(ypr, carrotPose.pose.orientation);
        carrotPosePublisher.publish(carrotPose);
    }

    // calculate steering angle
    double relative_angle = angular_norm(atan2(carrot.y - pose.pose.position.y, carrot.x - pose.pose.position.x) - angles[0]);
    double orientation_error = angular_norm(carrot.orientation - angles[0]);
    float sign = legs[current].backward ? -1.0 : 1.0;
    float speed = sign * legs[current].speed;

    this->vehicle_control_interface_->executeMotionCommand(relative_angle, orientation_error, motion_control_setup.carrot_distance, speed );

    // check if vehicle is blocked
    if (check_if_blocked && dt > 0.0) {
        double current_velocity_error = 0.0;
        current_velocity_error = (motion_control_setup.current_velocity - vehicle_control_interface_->getCommandedSpeed()) / std::max(fabs(vehicle_control_interface_->getCommandedSpeed()), 0.1);
        if (vehicle_control_interface_->getCommandedSpeed() > 0) {
            if (current_velocity_error > 0.0) current_velocity_error = 0.0;
        } else if (vehicle_control_interface_->getCommandedSpeed() < 0) {
            if (current_velocity_error < 0.0) current_velocity_error = 0.0;
        }
        velocity_error = (velocity_blocked_time * velocity_error + dt * current_velocity_error) / (velocity_blocked_time + dt);

        ROS_DEBUG("Current velocity:   %f", motion_control_setup.current_velocity);
        ROS_DEBUG("Commanded velocity: %f", vehicle_control_interface_->getCommandedSpeed());
        ROS_DEBUG("==> Mean error: %f %%", velocity_error * 100.0);

        std_msgs::Float32 temp;
        temp.data = velocity_error;
        diagnosticsPublisher.publish(temp);

        if (fabs(velocity_error) > velocity_blocked_limit) {
            ROS_WARN("I think I am blocked! Terminating current drive goal...");
            ROS_WARN("Current velocity:   %f", motion_control_setup.current_velocity);
            ROS_WARN("Commanded velocity: %f", vehicle_control_interface_->getCommandedSpeed());
            ROS_WARN("==> Mean error: %f %%", velocity_error * 100.0);
            state = INACTIVE;
            velocity_error = 0.0;
            stop();

            publishActionResult(actionlib_msgs::GoalStatus::ABORTED, "blocked");
        }
    }

    // camera control
    if (camera_control) {
        // calculate lookat position
        Point lookat;
        unsigned int lookat_waypoint = current;
        bool found_lookat_position = false;

        while(lookat_waypoint < legs.size()) {
            lookat.x           = legs[lookat_waypoint].p2.x;
            lookat.y           = legs[lookat_waypoint].p2.y;
            lookat.orientation = legs[lookat_waypoint].p2.orientation;

            float distance = sqrt((pose.pose.position.x - lookat.x)*(pose.pose.position.x - lookat.x) + (pose.pose.position.y - lookat.y)*(pose.pose.position.y - lookat.y));
            double relative_angle = angular_norm(atan2(lookat.y - pose.pose.position.y, lookat.x - pose.pose.position.x) - angles[0]);

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
            lookat_msg.header = pose.header;
            lookat_msg.point.x = lookat.x;
            lookat_msg.point.y = lookat.y;
            lookat_msg.point.z = pose.pose.position.z + camera_lookat_height;
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

void Controller::limitSpeed(float &speed) {
    float inclination_max_speed = std::max(fabs(speed) * (1.0 - motion_control_setup.current_inclination * motion_control_setup.inclination_speed_reduction_factor), 0.0);

    if (speed > 0.0) {
        if (speed > motion_control_setup.max_speed) speed = motion_control_setup.max_speed;
        if (speed > inclination_max_speed) speed = inclination_max_speed;
        if (speed < motion_control_setup.min_speed) speed = motion_control_setup.min_speed;
    } else if (speed < 0.0) {
        if (speed < -motion_control_setup.max_speed) speed = -motion_control_setup.max_speed;
        if (speed < -inclination_max_speed) speed = -inclination_max_speed;
        if (speed > -motion_control_setup.min_speed) speed = -motion_control_setup.min_speed;
    }
}


void Controller::stop()
{
    this->vehicle_control_interface_->stop();
    drivepathPublisher.publish(empty_path);
    if (camera_control) cameraOrientationPublisher.publish(cameraDefaultOrientation);
}

void Controller::cleanup()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    Controller c;
    c.configure();
    while(ros::ok())
    {
        ros::spin();
    }
    c.cleanup();

    ros::shutdown();
    return 0;
}
