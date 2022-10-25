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

const double StuckDetector::DEFAULT_DETECTION_WINDOW = 5.0;

StuckDetector::StuckDetector(double detection_window) :DETECTION_WINDOW(detection_window)
{

}

void StuckDetector::update(geometry_msgs::PoseStamped const & pose, double cmded_speed)
{
    pose_history.push_back(pose);
    speed_history.push_back(std::abs(cmded_speed));

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


    // Linear motion
    // Compute driven path length
    double driven_distance = 0;
    for (unsigned int i = 0; i < pose_history.size() - 1; ++i) {
      driven_distance += euclideanDistance(pose_history[i].pose.position, pose_history[i+1].pose.position);
    }
    double avg_driven_speed = driven_distance / time_diff;

    // Average commanded speed
    double avg_cmded_speed = std::accumulate(speed_history.begin(), speed_history.end(), 0.0)
                             / static_cast<double>(speed_history.size());

    // Stuck if average driven speed is lower than percentage of average commanded speed
    double speed_threshold = MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION * std::abs(avg_cmded_speed);
    bool lin_stuck = avg_driven_speed < speed_threshold;

    // Angular motion
    const geometry_msgs::Pose start_pose = pose_history.front().pose;
    double zstart = constrainAngle_mpi_pi(quat2ZAngle(start_pose.orientation));

    auto it_max_ang = std::max_element(pose_history.begin() + 1, pose_history.end(),
                        [this,start_pose,zstart](geometry_msgs::PoseStamped const & pl,
                                          geometry_msgs::PoseStamped const & pr)
                        {
                            double zl = constrainAngle_mpi_pi(quat2ZAngle(pl.pose.orientation));
                            double zr = constrainAngle_mpi_pi(quat2ZAngle(pr.pose.orientation));
                            return std::abs(constrainAngle_mpi_pi(zl - zstart))
                                    < std::abs(constrainAngle_mpi_pi(zr - zstart));
                        });

    double max_ang = std::abs(constrainAngle_mpi_pi(
                                  constrainAngle_mpi_pi(quat2ZAngle(it_max_ang->pose.orientation))
                                  - zstart));
    bool rot_stuck = max_ang < MIN_ANGULAR_CHANGE;

    bool stuck = rot_stuck && lin_stuck;
    if (stuck) {
      ROS_INFO_STREAM("Stuck detected. " <<
                      "ang:  " << max_ang << " < " << MIN_ANGULAR_CHANGE << "  ,  " <<
                      "vlin:  " << avg_driven_speed << " < " << speed_threshold
                      );
    }
    return stuck;
}
