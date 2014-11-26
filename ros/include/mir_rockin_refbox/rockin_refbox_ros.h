#ifndef ROCKIN_REFBOX_ROS_H_
#define ROCKIN_REFBOX_ROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mir_rockin_refbox/rockin_refbox.h>

using std::string;

class RockinRefboxRos
{
public:
    RockinRefboxRos(ros::NodeHandle &nh);
    ~RockinRefboxRos();

private:
    // methods
    bool controlDrillingMachine(const std::string& command);
    bool controlConveyorBelt(const std::string& command);
    bool controlQualityCamera(const std::string& command);
    void eventOut(const std::string& event_msg);
    void conveyorStatus(const std::string& status);
    void drillStatus(const std::string& status);
    void cameraStatus(const std::string& status);

    void cbEventIn(const std_msgs::String::ConstPtr& msg);
    void cbConveyorControl(const std_msgs::String::ConstPtr& msg);
    void cbDrillControl(const std_msgs::String::ConstPtr& msg);
    void cbCameraControl(const std_msgs::String::ConstPtr& msg);
    bool getRefboxConfigParams();

    // variables
    RockinRefbox* refbox_;

    ros::NodeHandle* nh_;
    string event_in_;
    string conveyor_control_;
    string drill_control_;
    string camera_control_;

    string refbox_ip_;
    int refbox_port_;
    string team_name_;
    string team_robot_;
    int team_port_;

    // ROS Publishers
    ros::Publisher event_out_pub_;
    ros::Publisher conveyor_status_pub_;
    ros::Publisher drill_status_pub_;
    ros::Publisher camera_status_pub_;
    ros::Publisher camera_image_pub_;   
    
    // ROS Subscribers
    ros::Subscriber event_in_sub_;
    ros::Subscriber conveyor_control_sub_;
    ros::Subscriber drill_control_sub_;
    ros::Subscriber camera_control_sub_;

};

#endif /* ROCKIN_REFBOX_ROS_H_ */
