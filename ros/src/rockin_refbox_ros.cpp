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

    //
    refbox_ = new RockinRefbox(team_robot_, team_name_, refbox_ip_,
        refbox_port_, team_port_);
    if(!refbox_){
        ROS_ERROR("new RockinRefbox creation failed!!!!");
    }

    //signal(SIGINT, boost::bind(&RockinRefboxRos::mySignalHandler,this, _1));

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

RockinRefboxRos::~RockinRefboxRos()
{
    delete refbox_;
}

bool RockinRefboxRos::startRefbox()
{
    if(refbox_)
    {
        refbox_->start();
        return true;
    }
    return false;
}

bool RockinRefboxRos::stopRefbox()
{
    if(refbox_)
    {
        refbox_->stop();
        return true;
    }
    return false;
}

/*
void RockinRefboxRos::mySignalHandler(int sig)
{
    stopRefbox();
    ros::shutdown();
}
*/


bool RockinRefboxRos::getRefboxConfigParams()
{
    // REFBOX IP
    if (nh_->hasParam("refbox/ip"))
    {
        nh_->param<std::string>("refbox/ip", refbox_ip_,"192.168.2.107");
        ROS_INFO("Refbox IP: %s", refbox_ip_.c_str() );
        ROS_INFO("namespace!!!!!! %s", nh_->getNamespace().c_str());
    } else {
        ROS_ERROR("no refbox/ip param");
        return false;
    }
    // REFBOX PORT
    if (nh_->hasParam("refbox/port"))
    {
        nh_->param<int>("refbox/port", refbox_port_, 4446);
        ROS_INFO("Refbox Port: %d", refbox_port_);
    } else {
        ROS_ERROR("no refbox/port param");
        return false;
    }
    // TEAM NAME
    if (nh_->hasParam("team/name"))
    {
        nh_->param<std::string>("refbox/ip", team_name_, "b-it-bots");
        ROS_INFO("Team Name %s", team_name_.c_str());
    } else {
        ROS_ERROR("no refbox/ip param");
        return false;
    }
    // TEAM ROBOT NAME
    if (nh_->hasParam("team/robot"))
    {
        nh_->param<std::string>("team/robot", team_robot_, "youbot-brsu");
        ROS_INFO("Robot Name: %s", team_robot_.c_str());
    } else {
        ROS_ERROR("no team/robot param");
        return false;
    }
    // TEAM PORT
    if (nh_->hasParam("team/port"))
    {
        nh_->param<int>("team/port", team_port_, 4446);
        ROS_INFO("Team port %d", team_port_);
    } else {
        ROS_ERROR("no team/port param");
        return false;
    }
    return true;

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
