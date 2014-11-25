#include "mir_rockin_refbox/rockin_refbox_ros.h"

RockinRefboxRos::RockinRefboxRos(ros::NodeHandle &nh)
{
    nh_ = &nh;
    
    // ROS Params
    if (!this->getRefboxConfigParams())
    {
        ROS_ERROR("could not get refbox parameters.");
        exit(0);
    }

    // ROS Subscribers
    event_in_sub_ = nh_->subscribe<std_msgs::String>("event_in", 1, &RockinRefboxRos::cbEventIn, this);
    conveyor_control_sub_ = nh_->subscribe<std_msgs::String>("conveyor_control", 1, &RockinRefboxRos::cbConveyorControl, this); 
    drill_control_sub_ = nh_->subscribe<std_msgs::String>("drill_control", 1, &RockinRefboxRos::cbDrillControl, this);
    camera_control_sub_ = nh_->subscribe<std_msgs::String>("camera_control", 1, &RockinRefboxRos::cbCameraControl, this);

    // ROS Publishers
    event_out_pub_ = nh_->advertise<std_msgs::String>("event_out", 1);
    conveyor_status_pub_ = nh_->advertise<std_msgs::String>("conveyor_status", 1);
    drill_status_pub_ = nh_->advertise<std_msgs::String>("drill_status", 1);
    camera_status_pub_ = nh_->advertise<std_msgs::String>("camera_status", 1);
    //camera_image_pub_ = nh_->advertise<std_msgs::String>("topic", 1);
}

RockinRefboxRos::~RockinRefboxRos() { }

bool RockinRefboxRos::getRefboxConfigParams()
{
    // REFBOX IP
    if (nh_->hasParam("refbox/ip"))
    {
        ros::param::param<std::string>("refbox/ip", refbox_ip_,"192.168.1.100");
    } else {
        ROS_ERROR("no refbox/ip param");
        return false;
    }
    // REFBOX PORT
    if (nh_->hasParam("refbox/port"))
    {
        ros::param::param<int>("refbox/port", refbox_port_, 4446);
    } else {
        ROS_ERROR("no refbox/port param");
        return false;
    }
    /* TEAM NAME
    if (nh_->hasParam("team/name"))
    {
        ros::param::param<std::string>("refbox/ip", team_name_);
    } else {
        ROS_ERROR("no refbox/ip param");
        return false;
    }
    // TEAM ROBOT NAME
    if (nh_->hasParam("team/robot"))
    {
        ros::param::param<std::string>("team/robot", team_robot_);
    } else {
        ROS_ERROR("no team/robot param");
        return false;
    }
    // TEAM PORT
    if (nh_->hasParam("team/port"))
    {
        ros::param::param<int>("team/port", team_port_);
    } else {
        ROS_ERROR("no team/port param");
        return false;
    }
*/
    
}

void RockinRefboxRos::cbEventIn(const std_msgs::String::ConstPtr& msg)
{
    event_in_ = msg->data;
}
void RockinRefboxRos::cbConveyorControl(const std_msgs::String::ConstPtr& msg)
{
    conveyor_control_ = msg->data;
}
void RockinRefboxRos::cbDrillControl(const std_msgs::String::ConstPtr& msg)
{
    drill_control_ = msg->data;
}
void RockinRefboxRos::cbCameraControl(const std_msgs::String::ConstPtr& msg)
{
    camera_control_ = msg->data;
}
