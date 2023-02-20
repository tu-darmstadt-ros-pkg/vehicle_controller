/*
    Copyright (c) 2015, Paul Manns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <vehicle_controller/stuck_detector.h>
#include <vehicle_controller/quaternions.h>
#include <vehicle_controller/utility.h>
#include <numeric>
#include <ros/console.h>
#include <std_msgs/Float64.h>

const double StuckDetector::DEFAULT_DETECTION_WINDOW = 5.0;

StuckDetector::StuckDetector(const ros::NodeHandle& nh, double detection_window) : DETECTION_WINDOW(detection_window), nh_(ros::NodeHandle(nh, "stuck_detector"))
{
  cmded_speed_pub_ = nh_.advertise<std_msgs::Float64>("average_commanded_speed", 10);
  estimated_speed_pub_ = nh_.advertise<std_msgs::Float64>("average_estimated_speed", 10);
  speed_threshold_pub_ = nh_.advertise<std_msgs::Float64>("speed_threshold", 10);

  cmded_rot_speed_pub_ = nh_.advertise<std_msgs::Float64>("average_commanded_rotational_speed", 10);
  estimated_rot_speed_pub_ = nh_.advertise<std_msgs::Float64>("average_estimated_rotational_speed", 10);
  rot_speed_threshold_pub_ = nh_.advertise<std_msgs::Float64>("rotational_speed_threshold", 10);
}

void StuckDetector::update(geometry_msgs::PoseStamped const & pose, double cmded_speed, double cmded_rotation)
{
    pose_history.push_back(pose);
    speed_history.push_back(std::abs(cmded_speed));
    rotation_rate_history.push_back(std::abs(cmded_rotation));

    double secs_to_remove = elapsedSecs() - DETECTION_WINDOW;

    std::deque<double> time;
    std::transform(pose_history.begin(), pose_history.end(), std::back_inserter(time),
                   [this](geometry_msgs::PoseStamped const & p)
                   {
                        return (p.header.stamp - pose_history.begin()->header.stamp).toSec();
                   });
    auto it = std::lower_bound(time.begin(), time.end(), secs_to_remove);

    if(it != time.end() && it != time.begin()) {
        it--;
        pose_history.erase(pose_history.begin(),
                            pose_history.begin() + std::distance(time.begin(), it));
        speed_history.erase(speed_history.begin(),
                            speed_history.begin() + std::distance(time.begin(), it));
        rotation_rate_history.erase(rotation_rate_history.begin(),
                                    rotation_rate_history.begin() + std::distance(time.begin(), it));
    }
}

double StuckDetector::elapsedSecs() const
{
    if(pose_history.size() < 2)
        return 0.0;
    ros::Time start = pose_history.front().header.stamp;
    ros::Time end   = pose_history.back().header.stamp;
    return (end - start).toSec();
}

void StuckDetector::reset()
{
    pose_history.clear();
    speed_history.clear();
    rotation_rate_history.clear();
}

double StuckDetector::quat2ZAngle(geometry_msgs::Quaternion const & q) const
{
    double a[3];
    quaternion2angles(q, a);
    return a[0];
}

bool StuckDetector::isStuck() const
{
    if(pose_history.size() < 2 || speed_history.size() < 2)
        return false;

    double time_diff = elapsedSecs();
    if (time_diff < DETECTION_WINDOW)
      return false;

    // Compute driven path length
    double driven_distance = 0;
    double rotation_distance = 0;
    for (unsigned int i = 0; i < pose_history.size() - 1; ++i) {
      driven_distance += euclideanDistance(pose_history[i].pose.position, pose_history[i+1].pose.position);
      double z_start = constrainAngle_mpi_pi(quat2ZAngle(pose_history[i].pose.orientation));
      double z_end = constrainAngle_mpi_pi(quat2ZAngle(pose_history[i+1].pose.orientation));
      rotation_distance += std::abs(constrainAngle_mpi_pi(z_end - z_start));
    }
    double avg_driven_speed = driven_distance / time_diff;

    // Linear motion
    // Average commanded speed
    double avg_cmded_speed = std::accumulate(speed_history.begin(), speed_history.end(), 0.0)
                             / static_cast<double>(speed_history.size());

    // Stuck if average driven speed is lower than percentage of average commanded speed
    double speed_threshold = MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION * std::abs(avg_cmded_speed);
    bool lin_stuck = avg_driven_speed < speed_threshold && avg_cmded_speed > MIN_COMMANDED_SPEED;

    // Angular motion
    double avg_cmded_rotational_rate = std::accumulate(rotation_rate_history.begin(), rotation_rate_history.end(), 0.0)
                                       / static_cast<double>(rotation_rate_history.size());
    double avg_driven_rotational_rate = rotation_distance / time_diff;
    double rotational_rate_threshold = MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION * avg_cmded_rotational_rate;
    bool rot_stuck = avg_driven_rotational_rate < rotational_rate_threshold && avg_cmded_rotational_rate > MIN_ANGULAR_CHANGE;

    bool stuck = rot_stuck || lin_stuck;

    // Publish results
    publishDouble(cmded_speed_pub_, avg_cmded_speed);
    publishDouble(estimated_speed_pub_, avg_driven_speed);
    publishDouble(speed_threshold_pub_, speed_threshold);

    publishDouble(cmded_rot_speed_pub_, avg_cmded_rotational_rate);
    publishDouble(estimated_rot_speed_pub_, avg_driven_rotational_rate);
    publishDouble(rot_speed_threshold_pub_, rotational_rate_threshold);
    return stuck;
}
void StuckDetector::publishDouble(const ros::Publisher& publisher, double value)
{
  std_msgs::Float64 float_msg;
  float_msg.data = value;
  publisher.publish(float_msg);
}
