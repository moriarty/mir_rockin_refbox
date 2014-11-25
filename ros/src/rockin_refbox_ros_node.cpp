#include "mir_rockin_refbox/rockin_refbox_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rockin_refbox_ros");
    ros::NodeHandle nh("~");

    RockinRefboxRos* refbox_ros = new RockinRefboxRos(nh);

    ros::spin();

    delete refbox_ros;

    return 0;
}
