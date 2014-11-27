
#include <mir_rockin_refbox/rockin_refbox.h>

RockinRefbox::RockinRefbox(const std::string &name, const std::string &team_name, const std::string &host, int public_port,
        int private_port) : name_(name), team_name_(team_name), host_(host), public_port_(public_port), private_port_(private_port), sequence_number_(0),
                         run_timer_(false), debug_mode_(false)
{
    std::cout << "Starting Rockin Refbox" << std::endl;
    peer_public_ = new ProtobufBroadcastPeer(host_, public_port_);

    std::cout << "Created public peer" << std::endl;

    MessageRegister &message_register = peer_public_->message_register();
    message_register.add_message_type<BeaconSignal>();
    message_register.add_message_type<BenchmarkState>();
    message_register.add_message_type<VersionInfo>();
    message_register.add_message_type<Inventory>();
    message_register.add_message_type<OrderInfo>();
    message_register.add_message_type<DrillingMachineStatus>();
    message_register.add_message_type<ConveyorBeltStatus>();
    message_register.add_message_type<CompressedImage>();
    message_register.add_message_type<BenchmarkFeedback>();

    std::cout << "Registered messages" << std::endl;

    peer_team_ = new ProtobufBroadcastPeer(host_, private_port_, &message_register);
    std::cout << "Created team peer" << std::endl;

    peer_public_->signal_received().connect(boost::bind(&RockinRefbox::handle_message, this, _1, _2, _3, _4));
    peer_public_->signal_recv_error().connect(boost::bind(&RockinRefbox::handle_recv_error,this, _1, _2));
    peer_public_->signal_send_error().connect(boost::bind(&RockinRefbox::handle_send_error, this, _1));

    peer_team_->signal_received().connect(boost::bind(&RockinRefbox::handle_message, this, _1, _2, _3, _4));
    peer_team_->signal_recv_error().connect(boost::bind(&RockinRefbox::handle_recv_error,this, _1, _2));
    peer_team_->signal_send_error().connect(boost::bind(&RockinRefbox::handle_send_error, this, _1));
}


RockinRefbox::~RockinRefbox()
{
    delete timer_;
    delete peer_public_;
    delete peer_team_;
}

void RockinRefbox::start()
{
    run_timer_ = true;
    timer_ = new boost::asio::deadline_timer(io_service_);
    timer_->expires_from_now(boost::posix_time::milliseconds(1000));
    timer_->async_wait(boost::bind(&RockinRefbox::send_beacon_signal, this));
    boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service_));
}

void RockinRefbox::stop()
{
    if (timer_)
    {
        timer_->cancel();
    }

    io_service_.stop();
    run_timer_ = false;
}

void RockinRefbox::send_beacon_signal()
{
    boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
    std::shared_ptr<BeaconSignal> signal(new BeaconSignal());
    Time *time = signal->mutable_time();
    boost::posix_time::time_duration const since_epoch =
      now - boost::posix_time::from_time_t(0);

    time->set_sec(static_cast<google::protobuf::int64>(since_epoch.total_seconds()));
    time->set_nsec(
      static_cast<google::protobuf::int64>(since_epoch.fractional_seconds() * 
             (1000000000/since_epoch.ticks_per_second())));

    signal->set_peer_name(name_);
    signal->set_team_name(team_name_);
    signal->set_seq(++sequence_number_);
    peer_team_->send(signal);
    
    if (run_timer_)
    {
        timer_->expires_at(timer_->expires_at()
            + boost::posix_time::milliseconds(1000));
        timer_->async_wait(boost::bind(&RockinRefbox::send_beacon_signal,this));
    }
}

void RockinRefbox::handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
    std::cout << "Received error from " << endpoint.address().to_string() << ":" << endpoint.port() << ", " << msg << std::endl;
}


void RockinRefbox::handle_send_error(std::string msg)
{
    std::cout <<"Message send error: " << msg << std::endl;
}


void RockinRefbox::handle_message(boost::asio::ip::udp::endpoint &sender, uint16_t component_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
{
    
    std::shared_ptr<BeaconSignal> bs;
    if ((bs = std::dynamic_pointer_cast<BeaconSignal>(msg))) 
    {
        beacon_msg_ = bs;
        if (debug_mode_)
        {
            std::cout << "Detected robot: " << beacon_msg_->team_name() << " " << beacon_msg_->peer_name() << " (seq " << beacon_msg_->seq() << ")" << std::endl;
        }
    }

    std::shared_ptr<VersionInfo> vi;
    if ((vi = std::dynamic_pointer_cast<VersionInfo>(msg))) 
    {
        version_msg_ = vi;
        if (debug_mode_)
        {
            std::cout << "VersionInfo received: " << version_msg_->version_string() << std::endl;
        }
    }

    std::shared_ptr<BenchmarkState> bms;
    if ((bms = std::dynamic_pointer_cast<BenchmarkState>(msg))) 
    {
        benchmark_state_msg_ = bms;
        if (debug_mode_)
        {
            std::cout << "BenchmarkState received:" << std::endl;

            std::cout << "  Time: " << benchmark_state_msg_->benchmark_time().sec() << "s" << std::endl;

            std::cout << "  Phase: ";
            switch (benchmark_state_msg_->phase().type()) {
                case BenchmarkPhase::NONE: std::cout << "NONE"; break;
                case BenchmarkPhase::FBM: std::cout << "FBM"; break;
                case BenchmarkPhase::TBM: std::cout << "TBM"; break;
            }
            std::cout << " " << benchmark_state_msg_->phase().type_id();
            if (benchmark_state_msg_->phase().has_description()) std::cout << " (" << benchmark_state_msg_->phase().description() << ")";
            std::cout << std::endl;

            std::cout << "  State: ";
            switch (benchmark_state_msg_->state()) {
                case BenchmarkState::INIT: std::cout << "INIT" << std::endl; break;
                case BenchmarkState::RUNNING: std::cout << "RUNNING" << std::endl; break;
                case BenchmarkState::PAUSED: std::cout << "PAUSED" << std::endl; break;
                case BenchmarkState::FINISHED: std::cout << "FINISHED" << std::endl; break;
            }

            std::cout << "  Known teams: ";
            for (int i = 0; i < benchmark_state_msg_->known_teams_size(); i++) std::cout << benchmark_state_msg_->known_teams(i) << ", ";
            std::cout << std::endl;

            std::cout << "  Connected teams: ";
            for (int i = 0; i < benchmark_state_msg_->connected_teams_size(); i++) std::cout << benchmark_state_msg_->connected_teams(i) << ", ";
            std::cout << std::endl;
        }
    }

    std::shared_ptr<Inventory> in;
    if ((in = std::dynamic_pointer_cast<Inventory>(msg))) 
    {
        inventory_msg_ = in;
        if (debug_mode_)
        {
            std::cout << "Inventory received:" << std::endl;

            for (int i = 0; i < inventory_msg_->items_size(); i++) {
                const Inventory_Item &item = inventory_msg_->items(i);
                std::cout << "  Object " << i << ": " << item.object().description() << std::endl;
                if (item.has_location()) std::cout << "    In location: " << item.location().description() << std::endl;
                if (item.has_container()) std::cout << "    In container: " << item.container().description() << std::endl;
                if (item.has_quantity()) std::cout << "    Quantity: " << item.quantity() << std::endl;
            }
        }
    }

    std::shared_ptr<OrderInfo> o;
    if ((o = std::dynamic_pointer_cast<OrderInfo>(msg)))
    {
        order_msg_ = o;
        if (debug_mode_)
        {
            std::cout << "OrderInfo received" << std::endl;

            for (int i = 0; i < order_msg_->orders_size(); i++) {
                const Order &order = order_msg_->orders(i);
                std::cout << "  Order " << i << ":" << std::endl;
                std::cout << "    Identifier: " << order.id() << std::endl;
                std::cout << "    Status: ";
                switch (order.status()) {
                    case Order::OFFERED: std::cout << "OFFERED"; break;
                    case Order::TIMEOUT: std::cout << "TIMEOUT"; break;
                    case Order::IN_PROGRESS: std::cout << "IN_PROGRESS"; break;
                    case Order::PAUSED: std::cout << "PAUSED"; break;
                    case Order::ABORTED: std::cout << "ABORTED"; break;
                    case Order::FINISHED: std::cout << "FINISHED"; break;
                }
                std::cout << std::endl;
                std::cout << "    Object: " << order.object().description() << std::endl;
                if (order.has_container()) std::cout << "    Container: " << order.container().description() << std::endl;
                std::cout << "    Quantity delivered: " << order.quantity_delivered() << std::endl;
                if (order.has_quantity_requested()) std::cout << "    Quantity requested: " << order.quantity_requested() << std::endl;
                if (order.has_destination()) std::cout << "    Destination: " << order.destination().description() << std::endl;
                if (order.has_source()) std::cout << "    Source: " << order.source().description() << std::endl;
                if (order.has_processing_team()) std::cout << "    Processing team: " << order.processing_team() << std::endl;
            }
        }
    }

    std::shared_ptr<DrillingMachineStatus> dms;
    if ((dms = std::dynamic_pointer_cast<DrillingMachineStatus>(msg)))
    {
        drilling_machine_msg_ = dms;
        if (debug_mode_)
        {
            std::cout << "Drilling machine status received: ";
            switch (drilling_machine_msg_->state()) {
                case DrillingMachineStatus::AT_BOTTOM: std::cout << "AT_BOTTOM"; break;
                case DrillingMachineStatus::AT_TOP: std::cout << "AT_TOP"; break;
                case DrillingMachineStatus::MOVING_DOWN: std::cout << "MOVING_DOWN"; break;
                case DrillingMachineStatus::MOVING_UP: std::cout << "MOVING_UP"; break;
                case DrillingMachineStatus::UNKNOWN: std::cout << "UNKNOWN"; break;
            }
            std::cout << std::endl;
        }
    }

    std::shared_ptr<ConveyorBeltStatus> cbs;
    if ((cbs = std::dynamic_pointer_cast<ConveyorBeltStatus>(msg)))
    {
        conveyor_belt_msg_ = cbs;
        if (debug_mode_)
        {
            std::cout << "Conveyor belt status received: ";
            switch (conveyor_belt_msg_->state()) {
                case ConveyorBeltRunMode::START: std::cout << "RUNNING"; break;
                case ConveyorBeltRunMode::STOP: std::cout << "STOPPED"; break;
            }
            std::cout << std::endl;
        }
    }

    std::shared_ptr<CompressedImage> im;
    if ((im = std::dynamic_pointer_cast<CompressedImage>(msg)))
    {
        image_msg_ = im;
        std::cout << "received compressed image " << std::endl;
        if (debug_mode_)
        {
            std::cout << "CompressedImage received (format=" << image_msg_->format() <<")" << std::endl;
        }
    }

}

void RockinRefbox::set_debug(bool value)
{
    debug_mode_ = value;
}

std::shared_ptr<BenchmarkState> RockinRefbox::get_benchmark_state()
{
    return benchmark_state_msg_;
}
std::shared_ptr<Inventory> RockinRefbox::get_inventory()
{
    return inventory_msg_;
}
std::shared_ptr<OrderInfo> RockinRefbox::get_order()
{
    return order_msg_;
}
std::shared_ptr<DrillingMachineStatus> RockinRefbox::get_drilling_machine_status()
{
    return drilling_machine_msg_;
}
std::shared_ptr<ConveyorBeltStatus> RockinRefbox::get_conveyor_belt_status()
{
    return conveyor_belt_msg_;
}
std::shared_ptr<CompressedImage> RockinRefbox::get_image()
{
    return image_msg_;
}

void RockinRefbox::send_conveyor_belt_command(bool on)
{
    ConveyorBeltCommand conveyor_belt_command;
/*
    if ((conveyor_belt_msg_->state() == START && on) || (conveyor_belt_msg_->state() == STOP && !on))
    {
        return;      
    }
    else
    {
        */
        if (on)
        {
            conveyor_belt_command.set_command(START);
        }
        else
        {
            conveyor_belt_command.set_command(STOP);
        }
        peer_team_->send(conveyor_belt_command);
 //   }

}

void RockinRefbox::send_drilling_machine_command(bool down)
{
    DrillingMachineCommand drilling_machine_command;
    DrillingMachineStatus::State state = drilling_machine_msg_->state();    
    if (down)
    {
 //       if (state == DrillingMachineStatus::AT_BOTTOM) return;
 //       if (state == DrillingMachineStatus::MOVING_DOWN) return;        
        drilling_machine_command.set_command(DrillingMachineCommand::MOVE_DOWN);
    }
    else
    {
 //       if (state == DrillingMachineStatus::AT_TOP) return;
 //       if (state == DrillingMachineStatus::MOVING_UP) return;
        drilling_machine_command.set_command(DrillingMachineCommand::MOVE_UP);
    }
    peer_team_->send(drilling_machine_command);
}

void RockinRefbox::send_camera_command()
{
    CameraCommand cam_cmd;
    peer_team_->send(cam_cmd);
}
