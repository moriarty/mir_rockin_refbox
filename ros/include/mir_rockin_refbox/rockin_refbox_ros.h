#ifndef ROCKIN_REFBOX_ROS_H_
#define ROCKIN_REFBOX_ROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <csignal>

#include <mir_rockin_refbox/rockin_refbox.h>
#include <mir_rockin_refbox/BenchmarkState.h>

using std::string;
using std::signal;

class RockinRefboxRos
{
public:
    RockinRefboxRos(ros::NodeHandle &nh);
    ~RockinRefboxRos();

    //void mySignalHandler(int sig);
    bool startRefbox();
    bool stopRefbox();
    void executeCycle();

private:
    enum State {
        INIT,
        IDLE,
        RUNNING
    };
    // methods
    bool controlDrillingMachine(const std::string& command);
    bool controlConveyorBelt(const std::string& command);
    bool controlQualityCamera(const std::string& command);
    void eventOut(const std::string& event_msg);
    void conveyorStatus(const std::string& status);
    void drillStatus(const std::string& status);
    void cameraStatus(const std::string& status);

    void cbEventIn(const std_msgs::String::ConstPtr& msg);
    void cbRequestIn(const std_msgs::String::ConstPtr& msg);
    void cbConveyorControl(const std_msgs::String::ConstPtr& msg);
    void cbDrillControl(const std_msgs::String::ConstPtr& msg);
    void cbCameraControl(const std_msgs::String::ConstPtr& msg);
    
    bool getRefboxConfigParams();
    string parseIntoRoboCupTask(std::shared_ptr<OrderInfo> order_info, 
        std::shared_ptr<Inventory> inventory);

    void initState();
    void idleState();
    void runningState();
    void handleRequest();

    // variables
    RockinRefbox* refbox_;
    ros::NodeHandle* nh_;

    // Data from ROS topics
    string event_in_;
    string request_in_;
    string conveyor_control_;
    string drill_control_;
    string camera_control_;
    //TODO FEEDBACK TO PASS TO REFBOX ?

    // internal state variables
    State state_;

    // ROS params
    string refbox_ip_;
    int refbox_public_port_;
    string team_name_;
    string team_robot_;
    int team_private_port_;

    // ROS Publishers
    ros::Publisher event_out_pub_;
    ros::Publisher conveyor_status_pub_;
    ros::Publisher drill_status_pub_;
    ros::Publisher camera_status_pub_;
    ros::Publisher camera_image_pub_;
    ros::Publisher benchmark_state_pub_;
    ros::Publisher refbox_task_pub_;

    // ROS Subscribers
    ros::Subscriber event_in_sub_;
    ros::Subscriber conveyor_control_sub_;
    ros::Subscriber drill_control_sub_;
    ros::Subscriber camera_control_sub_;
    ros::Subscriber request_in_sub_;

};

#endif /* ROCKIN_REFBOX_ROS_H_ */
