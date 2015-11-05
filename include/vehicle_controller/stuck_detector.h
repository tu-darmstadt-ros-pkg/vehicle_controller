#ifndef STUCK_DETECTOR_H
#define STUCK_DETECTOR_H

#include <deque>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <vehicle_controller/motion_parameters.h>

class StuckDetector
{
private:
    std::deque< geometry_msgs::PoseStamped > pose_history_;

    double time_diff;
    double DETECTION_WINDOW;
    MotionParameters const & mp;

protected:
    double elapsedSecs();

public:
    static constexpr double DEFAULT_DETECTION_WINDOW = 8.0;

    StuckDetector(MotionParameters const & mp, double detection_window = DEFAULT_DETECTION_WINDOW);

    double quat2ZAngle(geometry_msgs::Quaternion const & q);

    void update(geometry_msgs::PoseStamped const & pose);

    void reset();

    bool operator()();
};

#endif // STUCK_DETECTOR_H
