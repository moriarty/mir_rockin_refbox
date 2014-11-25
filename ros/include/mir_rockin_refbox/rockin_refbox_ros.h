#ifndef ROCKIN_REFBOX_ROS_H_
#define ROCKIN_REFBOX_ROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <protobuf_comm/peer.h>

#include <raw_refbox_comm/BeaconSignal.pb.h>
#include <raw_refbox_comm/VersionInfo.pb.h>
#include <raw_refbox_comm/BenchmarkState.pb.h>
#include <raw_refbox_comm/Inventory.pb.h>
#include <raw_refbox_comm/Order.pb.h>
#include <raw_refbox_comm/DrillingMachine.pb.h>
#include <raw_refbox_comm/ConveyorBelt.pb.h>
#include <raw_refbox_comm/Camera.pb.h>
#include <raw_refbox_comm/Image.pb.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>

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
