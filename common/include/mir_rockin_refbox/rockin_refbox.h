#ifndef ROCKIN_REFBOX_H_
#define ROCKIN_REFBOX_H_

//#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <protobuf_comm/peer.h>
using namespace protobuf_comm;

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
        int recv_port_;
        int send_port_;       
        unsigned long sequence_number_;
        ProtobufBroadcastPeer *peer_public_;
        boost::asio::deadline_timer *timer_;
        bool run_timer_;
};
#endif
