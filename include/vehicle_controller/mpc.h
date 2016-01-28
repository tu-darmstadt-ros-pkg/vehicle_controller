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
    std::mutex               cmd_vel_raw_pub_mtx;
    std::thread              check_time;
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
