#include <vehicle_controller/mpc.h>

using std::lock_guard;
using std::mutex;
using std::string;

MPC::MPC(string const & ns)
    : check_time(std::thread([this]() -> void
      {
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.05));
            if (executing && std::chrono::duration<double>(
                    std::chrono::system_clock::now() - start).count() > 0.1)
            {
                ROS_WARN("ACADO consuming too much time!");
                publishTwist(geometry_msgs::Twist());
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
    cmd_vel_raw_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_raw", 1);
}

void MPC::setupAcadoMpc()
{

}

void MPC::publishTwist(geometry_msgs::Twist const & twist)
{
    lock_guard<mutex> guard(cmd_vel_raw_pub_mtx);
    cmd_vel_raw_pub.publish(twist);
}

void MPC::triggerCallback(monstertruck_msgs::MpcTrigger const & msg)
{
    geometry_msgs::Twist twist;

    executing = true;
    start = std::chrono::system_clock::now();

    if( acado_mpc.execute(msg.position, msg.orientation,
                          msg.target_position, msg.target_orientation, twist)
            == AcadoMpcWrapper::RET_SUCCESS)
        publishTwist(twist);
    else
        publishTwist(msg.alternative);
    executing = false;

    auto end = std::chrono::system_clock::now();
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
