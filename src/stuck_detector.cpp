#include <vehicle_controller/stuck_detector.h>
#include "quaternions.h"
#include "utility.h"


StuckDetector::StuckDetector(MotionParameters const & mp, double detection_window) : mp(mp), DETECTION_WINDOW(detection_window)
{

}

void StuckDetector::update(geometry_msgs::PoseStamped const & pose)
{
    pose_history_.push_back(pose);

    double secs_to_remove = elapsedSecs() - DETECTION_WINDOW;

    std::deque<double> time;
    std::transform(pose_history_.begin(), pose_history_.end(), std::back_inserter(time),
                   [this](geometry_msgs::PoseStamped const & p)
                   {
                        return (p.header.stamp - pose_history_.begin()->header.stamp).toSec();
                   });
    auto it = std::upper_bound(time.begin(), time.end(), secs_to_remove);

    if(it != time.end())
        pose_history_.erase(pose_history_.begin(),
                            pose_history_.begin() + std::distance(time.begin(), it));
}

double StuckDetector::elapsedSecs()
{
    if(pose_history_.size() < 2)
        return 0.0;
    ros::Time start = pose_history_.begin()->header.stamp;
    ros::Time end   = pose_history_.end()->header.stamp;
    return (end - start).toSec();
}

void StuckDetector::reset()
{
    pose_history_.clear();
}

double StuckDetector::quat2ZAngle(geometry_msgs::Quaternion const & q)
{
    double a[3];
    quaternion2angles(q, a);
    return a[0];
}

bool StuckDetector::operator ()()
{
    if(pose_history_.size() < 2)
        return false;

    auto const & start_pose = pose_history_.begin()->pose;
    auto it_max_lin = std::max_element(pose_history_.begin() + 1, pose_history_.end(),
                        [this,start_pose](geometry_msgs::PoseStamped const & pl,
                                          geometry_msgs::PoseStamped const & pr)
                        {
                            return euclideanDistance(start_pose.position, pl.pose.position)
                                   < euclideanDistance(start_pose.position, pr.pose.position);
                        });

    auto it_max_ang = std::max_element(pose_history_.begin() + 1, pose_history_.end(),
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
    return max_ang < M_PI / 4 && max_lin / time_diff < 0.1 * mp.commanded_speed;
}
