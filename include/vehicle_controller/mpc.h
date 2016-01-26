#ifndef MPC_H
#define MPC_H

#include <ros/ros.h>
#include <monstertruck_msgs/MpcTrigger.h>
#include <vehicle_controller/acado_mpc_wrapper.h>
#include <geometry_msgs/Twist.h>

class MPC
{
private:
    ros::NodeHandle nh;

    ros::Subscriber trigger_sub;
    ros::Publisher  cmd_vel_raw_pub;

    AcadoMpcWrapper acado_mpc;

protected:
    void setupSubscribers();
    void setupPublishers();
    void setupAcadoMpc();

    virtual void triggerCallback(monstertruck_msgs::MpcTrigger const & msg);

public:
    MPC(std::string const & ns = std::string());
    virtual ~MPC();
};



#endif // MPC_H