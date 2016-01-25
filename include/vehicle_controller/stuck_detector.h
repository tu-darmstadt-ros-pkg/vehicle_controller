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

/**
 * @brief The StuckDetector class is used to detect if the robot has maneuvered
 *        into  a position where following the path in the usual manner does not
 *        work anymore.
 */
class StuckDetector
{
private:
    std::deque< geometry_msgs::PoseStamped > pose_history;
    MotionParameters const & mp;

    double const MIN_ANGULAR_CHANGE = M_PI / 5.0;              // radians
    double const MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION = 0.1; // 1

    double DETECTION_WINDOW;                                   // seconds

protected:
    double elapsedSecs() const;
    double quat2ZAngle(geometry_msgs::Quaternion const & q) const;

public:
    static const double DEFAULT_DETECTION_WINDOW;

    StuckDetector(MotionParameters const & mp, double detection_window = DEFAULT_DETECTION_WINDOW);

    /**
     * @brief update the pose history that is investigated to determine if the robot is stuck
     * @param pose added to the pose history
     */
    void update(geometry_msgs::PoseStamped const & pose);

    /**
     * @brief reset the stuck detection by clearing the pose history
     */
    void reset();

    /**
     * @brief operator ()
     *        Core function of the stuck detection class. The robot is deemed stuck if the
     *        following two conditions are satisfied
     *        During the sliding time window
     *        - the robot's orientation has changed less than MIN_ANGULAR_CHANGE
     *        - the robot's position has changed less than MIN_ACTUAL_TO_COMMANDED_SPEED_FRACTION
     *          of what the commanded linear speed would imply
     * @return true if the robot is stuck else false
     */
    bool operator()() const;
};

#endif // STUCK_DETECTOR_H
