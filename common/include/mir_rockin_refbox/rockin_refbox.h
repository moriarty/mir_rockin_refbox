#ifndef ROCKIN_REFBOX_H_
#define ROCKIN_REFBOX_H_

//#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
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
#include <raw_refbox_comm/BenchmarkFeedback.pb.h>
#include <raw_refbox_comm/Time.pb.h>
using namespace protobuf_comm;
using namespace rockin_msgs;

class RockinRefbox
{
    public:
        RockinRefbox(const std::string &name, const std::string &team_name, const std::string &host, int recv_port,
                    int send_port);
        virtual ~RockinRefbox();

        void handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
        void handle_send_error(std::string msg);
        void handle_message(boost::asio::ip::udp::endpoint &sender, uint16_t component_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg);
        void service_signals(const boost::system::error_code &error, int signum);

        void start();
        void stop();
        

    private:
        void send_beacon_signal();

    private:
        std::string name_;
        std::string team_name_;
        std::string host_;
        int private_port_;
        int public_port_;       
        unsigned long sequence_number_;
        ProtobufBroadcastPeer *peer_public_;
        ProtobufBroadcastPeer *peer_team_;
        boost::asio::deadline_timer *timer_;
        bool run_timer_;
        boost::asio::io_service io_service_;
        
        std::shared_ptr<BeaconSignal> beacon_msg_;
        std::shared_ptr<VersionInfo> version_msg_;
        std::shared_ptr<BenchmarkState> benchmark_state_msg_;
        std::shared_ptr<Inventory> inventory_msg_;
        std::shared_ptr<OrderInfo> order_msg_;
        std::shared_ptr<DrillingMachineStatus> drilling_machine_msg_;
        std::shared_ptr<ConveyorBeltStatus> conveyor_belt_msg_;
        std::shared_ptr<Image> image_msg_;
};
#endif
