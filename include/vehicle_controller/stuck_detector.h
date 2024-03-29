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

#ifndef STUCK_DETECTOR_H
#define STUCK_DETECTOR_H

#include <deque>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <vehicle_controller/motion_parameters.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

/**
 * @brief The StuckDetector class is used to detect if the robot has maneuvered
 *        into  a position where following the path in the usual manner does not
 *        work anymore.
 */
class StuckDetector
{
private:
    std::deque< geometry_msgs::PoseStamped > pose_history;
    std::deque< double > speed_history;
    std::deque< double > rotation_rate_history;

    double const MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION = 0.2;
    double const MIN_COMMANDED_ANGULAR_SPEED = 0.1;  // radians/second
    double const MIN_COMMANDED_LINEAR_SPEED = 0.05;  // meters/second

    double DETECTION_WINDOW;                         // seconds

    ros::NodeHandle nh_;

    ros::Publisher cmded_speed_pub_;
    ros::Publisher estimated_speed_pub_;
    ros::Publisher speed_threshold_pub_;

    ros::Publisher cmded_rot_speed_pub_;
    ros::Publisher estimated_rot_speed_pub_;
    ros::Publisher rot_speed_threshold_pub_;

protected:
    [[nodiscard]] double elapsedSecs() const;
    [[nodiscard]] double quat2ZAngle(geometry_msgs::Quaternion const & q) const;
    static void publishDouble(const ros::Publisher& publisher, double value);

public:
    static const double DEFAULT_DETECTION_WINDOW;

    explicit StuckDetector(const ros::NodeHandle& nh, double detection_window = DEFAULT_DETECTION_WINDOW);

    /**
     * @brief Update the pose and speed history that is investigated to determine if the robot is stuck
     * @param pose current pose of the robot
     * @param cmded_speed linear speed commanded to the robot
     * @param cmded_rotation angular speed commanded to the robot
     */
    void update(geometry_msgs::PoseStamped const & pose, double cmded_speed, double cmded_rotation);

    /**
     * @brief Reset the stuck detection by clearing the history
     */
    void reset();

    /**
     * @brief isStuck
     *        Core function of the stuck detection class. The robot is deemed stuck if *one of the
     *        following two* conditions is satisfied
     *        During the sliding time window
     *        - the robot's estimated average angular speed is less than MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION of the average commanded speed
     *          and the average commanded speed is higher than MIN_COMMANDED_ANGULAR_SPEED
     *        - the robot's estimated average linear speed is less than MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION of the average commanded speed
     *        and the average commanded speed is higher than MIN_COMMANDED_LINEAR_SPEED
     * @return true if the robot is stuck else false
     */
    [[nodiscard]] bool isStuck() const;
};

#endif // STUCK_DETECTOR_H
