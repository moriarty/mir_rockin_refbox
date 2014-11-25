#ifndef ROCKIN_REFBOX_ROS_H_
#define ROCKIN_REFBOX_ROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mir_protobuf_comm/peer.h>

#include <mir_rockin_proto_msgs/BeaconSignal.pb.h>
#include <mir_rockin_proto_msgs/VersionInfo.pb.h>
#include <mir_rockin_proto_msgs/BenchmarkState.pb.h>
#include <mir_rockin_proto_msgs/Inventory.pb.h>
#include <mir_rockin_proto_msgs/Order.pb.h>
#include <mir_rockin_proto_msgs/DrillingMachine.pb.h>
#include <mir_rockin_proto_msgs/ConveyorBelt.pb.h>
#include <mir_rockin_proto_msgs/Camera.pb.h>
#include <mir_rockin_proto_msgs/Image.pb.h>

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

    // variables
    ros::NodeHandle* nh_;
    string event_in_;
    string conveyor_control_;
    string drill_control_;
    string camera_control_;


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
