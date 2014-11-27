#include "mir_rockin_refbox/rockin_refbox_ros.h"

RockinRefboxRos* g_refbox_ros;

void mySignalHandler(int sig)
{
    if(g_refbox_ros)
    {
        ROS_ERROR("KILLING REFBOX");
        g_refbox_ros->stopRefbox();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rockin_refbox_ros");
    ros::NodeHandle nh("~");

    g_refbox_ros = new RockinRefboxRos(nh);
    g_refbox_ros->startRefbox();

    signal(SIGINT, mySignalHandler);

    while (ros::ok()) {
        g_refbox_ros->executeCycle();
        ros::Rate(10).sleep();
        ros::spinOnce();
    }

    delete g_refbox_ros;

    return 0;
}
