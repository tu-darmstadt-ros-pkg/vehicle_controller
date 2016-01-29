/*******************************************************************************
 * Copyright (c) 2016, Paul Manns
 *
 * All rights reserved.
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef MPC_H
#define MPC_H

#include <ros/ros.h>
#include <monstertruck_msgs/MpcTrigger.h>
#include <vehicle_controller/acado_mpc_wrapper.h>
#include <geometry_msgs/Twist.h>

#include <thread>

#include <chrono>
#include <queue>
#include <mutex>

class MPC
{
private:
    bool executing;

    double const             critical_compute_time = 0.1;
    double const             compute_time_check_interval = 0.05;

    std::mutex               cmd_vel_raw_pub_mtx;
    std::thread              compute_time_checker;
    std::chrono::system_clock::time_point start;

    ros::NodeHandle          nh;

    ros::Subscriber          trigger_sub;
    ros::Publisher           cmd_vel_raw_pub;

    AcadoMpcWrapper          acado_mpc;

protected:
    void setupSubscribers();
    void setupPublishers();
    void setupAcadoMpc();
    void publishTwist(geometry_msgs::Twist const & twist);

    virtual void triggerCallback(monstertruck_msgs::MpcTrigger const & msg);

public:
    MPC(std::string const & ns = std::string());

    virtual ~MPC();
};


#endif // MPC_H
