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

#include <vehicle_controller/mpc.h>

using geometry_msgs::Twist;

using std::chrono::duration;
using std::chrono::system_clock;
using std::lock_guard;
using std::mutex;
using std::string;


MPC::MPC(string const & ns)
    : compute_time_checker(std::thread([this]() -> void
      {
        while(true)
        {
            std::this_thread::sleep_for(
                    duration<double>(compute_time_check_interval));
            if (executing
             && duration<double>(system_clock::now() - start).count()
                                     > critical_compute_time)
            {
                ROS_WARN("ACADO hacking too much time!  "
                         "Stopping robot until ACADO has converged.");
                publishTwist(Twist());
        }}})), nh(ns)
{
    setupSubscribers();
    setupPublishers();
    setupAcadoMpc();
}

MPC::~MPC()
{

}

void MPC::setupSubscribers()
{
    trigger_sub = nh.subscribe("trigger_mpc", 1, &MPC::triggerCallback, this);
}

void MPC::setupPublishers()
{
    cmd_vel_raw_pub = nh.advertise<Twist>("cmd_vel_raw", 1);
}

void MPC::setupAcadoMpc()
{

}

void MPC::publishTwist(Twist const & twist)
{
    lock_guard<mutex> guard(cmd_vel_raw_pub_mtx);
    cmd_vel_raw_pub.publish(twist);
}

void MPC::triggerCallback(monstertruck_msgs::MpcTrigger const & msg)
{
    geometry_msgs::Twist twist;

    executing = true;
    start = std::chrono::system_clock::now();

    AcadoMpcWrapper::EXECUTE_RETC retc =
            acado_mpc.execute(msg.position,
                              msg.orientation,
                              msg.target_position,
                              msg.target_orientation,
                              twist);

    if( retc == AcadoMpcWrapper::RET_SUCCESS)
        publishTwist(twist);
    else
    {
        ROS_WARN("ACADO failed with return code %d.", retc);
        publishTwist(msg.alternative);
    }
    executing = false;

    auto end = system_clock::now();
    ROS_INFO("Elapsed Time = %fs",
             std::chrono::duration<double>(end - start).count());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    MPC mpc;

    while(ros::ok())
    {
        ros::spin();
    }

    ros::shutdown();
    return 0;
}
