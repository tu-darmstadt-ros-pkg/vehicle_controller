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

const double StuckDetector::DEFAULT_DETECTION_WINDOW = 8.0;

StuckDetector::StuckDetector(MotionParameters const & mp, double detection_window) : mp(mp), DETECTION_WINDOW(detection_window)
{

}

void StuckDetector::update(geometry_msgs::PoseStamped const & pose)
{
    pose_history.push_back(pose);

    double secs_to_remove = elapsedSecs() - DETECTION_WINDOW;

    std::deque<double> time;
    std::transform(pose_history.begin(), pose_history.end(), std::back_inserter(time),
                   [this](geometry_msgs::PoseStamped const & p)
                   {
                        return (p.header.stamp - pose_history.begin()->header.stamp).toSec();
                   });
    auto it = std::lower_bound(time.begin(), time.end(), secs_to_remove);

    if(it != time.end())
        pose_history.erase(pose_history.begin(),
                            pose_history.begin() + std::distance(time.begin(), it));
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

bool StuckDetector::operator ()() const
{
    if(pose_history.size() < 2)
        return false;

    auto const & start_pose = pose_history.begin()->pose;
    auto it_max_lin = std::max_element(pose_history.begin() + 1, pose_history.end(),
                        [this,start_pose](geometry_msgs::PoseStamped const & pl,
                                          geometry_msgs::PoseStamped const & pr)
                        {
                            return euclideanDistance(start_pose.position, pl.pose.position)
                                   < euclideanDistance(start_pose.position, pr.pose.position);
                        });

    auto it_max_ang = std::max_element(pose_history.begin() + 1, pose_history.end(),
                        [this,start_pose](geometry_msgs::PoseStamped const & pl,
                                          geometry_msgs::PoseStamped const & pr)
                        {
                            double zstart = quat2ZAngle(start_pose.orientation);
                            double zl = quat2ZAngle(pl.pose.orientation);
                            double zr = quat2ZAngle(pr.pose.orientation);
                            return std::abs(constrainAngle_mpi_pi(zl - zstart))
                                    < std::abs(constrainAngle_mpi_pi(zr - zstart));
                        });

    double max_ang = std::abs(constrainAngle_mpi_pi(quat2ZAngle(it_max_ang->pose.orientation)
                                                    - quat2ZAngle(start_pose.orientation)));
    double max_lin = euclideanDistance(it_max_lin->pose.position, start_pose.position);
    double time_diff = elapsedSecs();
    return max_ang < MIN_ANGULAR_CHANGE
        && max_lin / time_diff < MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION * mp.commanded_speed
        && time_diff >= DETECTION_WINDOW;
}
