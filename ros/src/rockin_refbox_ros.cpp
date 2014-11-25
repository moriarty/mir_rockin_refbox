#include "mir_rockin_refbox/rockin_refbox_ros.h"

RockinRefboxRos::RockinRefboxRos(ros::NodeHandle &nh)
{
    nh_ = &nh;

    event_out_pub_;
    conveyor_status_pub_;
    drill_status_pub_;
    camera_status_pub_;
    camera_image_pub_;   
    
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