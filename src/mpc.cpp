#include <vehicle_controller/mpc.h>

#include <ros/ros.h>

using std::string;

MPC::MPC(string const & ns)
    : nh(ns)
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
    trigger_sub = nh.subscribe("trigger_mpc", 10, &MPC::triggerCallback, this);
}

void MPC::setupPublishers()
{
    cmd_vel_raw_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);
}

void MPC::setupAcadoMpc()
{

}

void MPC::triggerCallback(monstertruck_msgs::MpcTrigger const & msg)
{

    ROS_WARN("void MPC::triggerCallback(monstertruck_msgs::MpcTrigger const & msg)");

    geometry_msgs::Twist twist;
    if( acado_mpc.execute(msg.position, msg.orientation,
                          msg.target_position, msg.target_orientation, twist)
            == AcadoMpcWrapper::RET_SUCCESS)
        cmd_vel_raw_pub.publish(twist);
    else
        cmd_vel_raw_pub.publish(msg.alternative);
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
