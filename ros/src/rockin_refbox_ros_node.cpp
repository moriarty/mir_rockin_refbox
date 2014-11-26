#include "mir_rockin_refbox/rockin_refbox_ros.h"

RockinRefboxRos* g_refbox_ros;

void mySignalHandler(int sig)
{
    if(g_refbox_ros)
    {
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

    ros::spin();

    delete g_refbox_ros;

    return 0;
}
