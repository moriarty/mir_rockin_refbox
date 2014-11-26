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
        refbox_public_port_, team_private_port_);
    if(!refbox_){
        ROS_ERROR("new RockinRefbox creation failed!!!!");
    }

    //signal(SIGINT, boost::bind(&RockinRefboxRos::mySignalHandler,this, _1));

    // ROS Subscribers
    event_in_sub_ = nh_->subscribe<std_msgs::String>("event_in", 1, &RockinRefboxRos::cbEventIn, this);
    request_in_sub_ = nh_->subscribe<std_msgs::String>("request_in", 1, &RockinRefboxRos::cbRequestIn, this);
    conveyor_control_sub_ = nh_->subscribe<std_msgs::String>("conveyor_control", 1, &RockinRefboxRos::cbConveyorControl, this);
    drill_control_sub_ = nh_->subscribe<std_msgs::String>("drill_control", 1, &RockinRefboxRos::cbDrillControl, this);
    camera_control_sub_ = nh_->subscribe<std_msgs::String>("camera_control", 1, &RockinRefboxRos::cbCameraControl, this);

    // ROS Publishers
    event_out_pub_ = nh_->advertise<std_msgs::String>("event_out", 1);
    conveyor_status_pub_ = nh_->advertise<std_msgs::String>("conveyor_status", 1);
    drill_status_pub_ = nh_->advertise<std_msgs::String>("drill_status", 1);
    camera_status_pub_ = nh_->advertise<std_msgs::String>("camera_status", 1);
    benchmark_state_pub_ = nh_->advertise<mir_rockin_refbox::BenchmarkState>("benchmark_state", 1);
    refbox_task_pub_ = nh_->advertise<std_msgs::String>("refbox_task", 1);
    //camera_image_pub_ = nh_->advertise<std_msgs::String>("topic", 1);
}

RockinRefboxRos::~RockinRefboxRos()
{
    delete refbox_;
}

void RockinRefboxRos::initState()
{
    state_ = IDLE;
}

void RockinRefboxRos::idleState()
{
    if (event_in_ == "e_start") {
        state_ = RUNNING;
    } else {
        state_ = IDLE;
    }
}

void RockinRefboxRos::runningState()
{
    //
    handleRequest();
    state_ = IDLE;
}

void RockinRefboxRos::handleRequest()
{
    if (request_in_ == "r_state") {
        ROS_INFO("Handling r_state request");
        std::shared_ptr<BenchmarkState> benchmark_state = refbox_->get_benchmark_state();
        if (benchmark_state) {
            mir_rockin_refbox::BenchmarkState bm_msg;
            bm_msg.benchmark_state = benchmark_state->state();
            benchmark_state_pub_.publish(bm_msg);
            request_in_ = "";
        }
    }
    if (request_in_ == "r_task") {
        ROS_INFO("Handling r_task request");
        std::shared_ptr<OrderInfo> order_info = refbox_->get_order();
        std::shared_ptr<Inventory> inventory = refbox_->get_inventory();
        if (order_info && inventory)
        {
            std_msgs::String task_spec;
            task_spec.data = parseIntoRoboCupTask(order_info, inventory);
            refbox_task_pub_.publish(task_spec);
            request_in_ = "";
        }
    }
}

string RockinRefboxRos::parseIntoRoboCupTask(std::shared_ptr<OrderInfo> order_info, 
    std::shared_ptr<Inventory> inventory)
{
    //TODO A LOT OF CRAP
    return "BTT<initialsituation(<W01,(AX-07,AX-02)>);goalsituation(<S01,(AX-07,AX-02)>)>";
}

void RockinRefboxRos::executeCycle()
{
    switch (state_) {
        case INIT:
        initState();
        break;
        case IDLE:
        idleState();
        break;
        case RUNNING:
        runningState();
        break;
        default:
        initState();
    }
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
        ROS_ERROR("STOP REFBOX_ !!!!!!");
        refbox_->stop();
        return true;
    }
    ROS_ERROR("REFBOX_ DOESN'T EXIST - CAN'T STOP!!!!");
    return false;
}

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
    if (nh_->hasParam("refbox/public_port"))
    {
        nh_->param<int>("refbox/public_port", refbox_public_port_, 4446);
        ROS_INFO("Refbox Port: %d", refbox_public_port_);
    } else {
        ROS_ERROR("no refbox/public_port param");
        return false;
    }
    // TEAM NAME
    if (nh_->hasParam("team/name"))
    {
        nh_->param<std::string>("team/name", team_name_, "b-it-bots");
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
    if (nh_->hasParam("team/private_port"))
    {
        nh_->param<int>("team/private_port", team_private_port_, 4446);
        ROS_INFO("Team port %d", team_private_port_);
    } else {
        ROS_ERROR("no team/private_port param");
        return false;
    }
    return true;

}

void RockinRefboxRos::cbEventIn(const std_msgs::String::ConstPtr& msg)
{
    event_in_ = msg->data;
}
void RockinRefboxRos::cbRequestIn(const std_msgs::String::ConstPtr& msg)
{
    request_in_ = msg->data;
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
