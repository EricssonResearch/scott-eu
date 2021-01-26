#include <thread>
#include <turtlebot2i_edge/wireless.h>
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"

WirelessNetwork::WirelessNetwork(int n_robots) :
        stamping_(n_robots), to_stamp_(n_robots), stamped_(n_robots), stamping_mutex_(n_robots), stamping_cv_(n_robots),
        pinging_(n_robots), ping_results_(n_robots), pinging_mutex_(n_robots), pinging_cv_(n_robots) {
    ns3::GlobalValue::Bind("SimulatorImplementationType", ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

    robots_ = ns3::NodeContainer(n_robots);         // first nodes, so that they get IDs in [0, n_robots-1]
    mec_server_ = ns3::CreateObject<ns3::Node>();

    addMobility(robots_);
    addInternetStack(robots_);
    addInternetStack(mec_server_);
}

WirelessNetwork::~WirelessNetwork() {
    ns3::Simulator::Destroy();
}

void WirelessNetwork::createApplications() {
    /*
     * Note that the correct interface of the MEC server is the second one (index 1), since we install the internet
     * stack in the constructor, before installing the net device.
     *
     * See https://www.nsnam.org/wiki/HOWTO_use_IP_interface_indexes.
     */

    ns3::ApplicationContainer apps;
    ns3::Ipv4Address any = ns3::Ipv4Address::GetAny();
    ns3::Ipv4Address server_address = mec_server_->GetObject<ns3::Ipv4>()->GetAddress(1, 0).GetLocal();

    ns3::PacketSinkHelper packet_sink_helper("ns3::TcpSocketFactory", ns3::InetSocketAddress(any, 2000));
    packet_sink_helper.SetAttribute("EnableSeqTsSizeHeader", ns3::BooleanValue(true));
    apps = packet_sink_helper.Install(mec_server_);
    apps.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize", ns3::MakeCallback(
            &WirelessNetwork::updateStampedBytes, this));

    ns3::UdpEchoServerHelper udp_echo_server_helper(9);
    udp_echo_server_helper.Install(mec_server_);

    ns3::BulkSendHelper bulk_send_helper("ns3::TcpSocketFactory", ns3::InetSocketAddress(server_address, 2000));
    bulk_send_helper.SetAttribute("EnableSeqTsSizeHeader", ns3::BooleanValue(true));
    apps = bulk_send_helper.Install(robots_);
    apps.Stop(ns3::Seconds(0.1));

    ns3::V4PingHelper ping_helper(server_address);
    ping_helper.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(0.1)));
    apps = ping_helper.Install(robots_);
    apps.Stop(ns3::Seconds(0.1));
    for (auto it=apps.Begin(); it!=apps.End(); it++) {
        ns3::Ptr<ns3::Application> app = *it;
        app->TraceConnectWithoutContext("RttStats", ns3::MakeCallback(&WirelessNetwork::saveRttStats, this));
    }
}

void WirelessNetwork::createWarehouse(const ns3::Box &boundaries, int n_rooms_x, int n_rooms_y) {
    warehouse_ = ns3::CreateObject<ns3::Building>();
    warehouse_->SetBoundaries(boundaries);
    warehouse_->SetBuildingType(ns3::Building::Commercial);
    warehouse_->SetExtWallsType(ns3::Building::ConcreteWithWindows);
    warehouse_->SetNFloors(1);
    warehouse_->SetNRoomsX(n_rooms_x);
    warehouse_->SetNRoomsY(n_rooms_y);
}

void WirelessNetwork::simulate() {
    ns3::Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    ns3::Simulator::Run();
}

int WirelessNetwork::getRobotId(const ns3::Address &address) {
    ns3::Ipv4Address robot_address = ns3::InetSocketAddress::ConvertFrom(address).GetIpv4();
    auto it = std::find_if(robots_.Begin(), robots_.End(), [robot_address](const ns3::Ptr<ns3::Node> &robot) {
        ns3::Ipv4Address robot_address_ = robot->GetObject<ns3::Ipv4>()->GetAddress(1, 0).GetLocal();
        return robot_address_ == robot_address;
    });
    if (it == robots_.End())
        throw std::logic_error("No robot associated to this address");
    int robot_id = it - robots_.Begin();
    return robot_id;
}

void WirelessNetwork::updateStampedBytes(ns3::Ptr<const ns3::Packet> packet, const ns3::Address &robot_address,
                                         const ns3::Address &server_address, const ns3::SeqTsSizeHeader &header) {
    (void) packet;
    (void) server_address;
    int robot_id = getRobotId(robot_address);
    int n_bytes = header.GetSize();

    std::unique_lock<std::mutex> ul(stamping_mutex_[robot_id]);
    stamped_[robot_id] += n_bytes;
    if (stamped_[robot_id] >= to_stamp_[robot_id]) {
        stamping_[robot_id] = false;
        stamping_cv_[robot_id].notify_one();
    }
}

void WirelessNetwork::saveRttStats(const ns3::Ptr<ns3::Node> &robot, double rtt_min, double rtt_avg, double rtt_max,
                                   double rtt_mdev, double packet_loss) {
    int robot_id = robot->GetId();
    bool all_lost = std::abs(packet_loss - 1) < std::numeric_limits<double>::epsilon();
    double max_double = std::numeric_limits<double>::max();

    std::unique_lock<std::mutex> ul(pinging_mutex_[robot_id]);
    PingResult &ping_result = ping_results_[robot_id];
    ping_result.rtt_min = all_lost ? max_double : rtt_min;
    ping_result.rtt_avg = all_lost ? max_double : rtt_avg;
    ping_result.rtt_max = all_lost ? max_double : rtt_max;
    ping_result.rtt_mdev = all_lost ? 0 : rtt_mdev;
    ping_result.packet_loss = packet_loss;
    pinging_[robot_id] = false;
    pinging_cv_[robot_id].notify_one();
}

int WirelessNetwork::stamp(int robot_id, int n_bytes, const ns3::Time &max_duration) {
    /*
     * Important: the callback transferCompleted() might be called either from the current thread or from the thread
     * simulating the network (blocked in ns3::Simulator::Run()). It depends on the implementation of the ns-3 core,
     * but we cannot make assumptions (even because the behavior may change in the future). Therefore, we must
     * synchronize the threads using mutex and condition variable. If the callback is called from the current thread,
     * mutex and condition variable are superfluous but the code still works.
     */

    ns3::Ptr<ns3::Node> robot = robots_.Get(robot_id);
    ns3::Ptr<ns3::BulkSendApplication> app = ns3::StaticCast<ns3::BulkSendApplication>(robot->GetApplication(0));

    std::unique_lock<std::mutex> ul(stamping_mutex_[robot_id]);
    stamped_[robot_id] = 0;
    to_stamp_[robot_id] = n_bytes;
    stamping_[robot_id] = true;
    ul.unlock();

    app->SetMaxBytes(n_bytes);
    app->Start(ns3::Seconds(0));

    ul.lock();
    stamping_cv_[robot_id].wait_for(ul, std::chrono::duration<double>(max_duration.GetSeconds()),
                                    [this, robot_id]() { return !stamping_[robot_id]; });
    int stamped = stamped_[robot_id];
    ul.unlock();

    app->Stop(ns3::Seconds(0));
    return stamped;
}

PingResult WirelessNetwork::ping(int robot_id, const ns3::Time &duration) {
    ns3::Ptr<ns3::Node> robot = robots_.Get(robot_id);
    ns3::Ptr<ns3::V4Ping> app = ns3::StaticCast<ns3::V4Ping>(robot->GetApplication(1));
    ns3::TimeValue interval;
    app->GetAttribute("Interval", interval);

    std::unique_lock<std::mutex> ul(pinging_mutex_[robot_id]);
    pinging_[robot_id] = true;
    ul.unlock();

    app->Stop(duration);            // before starting!
    app->Start(ns3::Seconds(0));

    ul.lock();
    pinging_cv_[robot_id].wait(ul, [this, robot_id]() { return !pinging_[robot_id]; });
    return ping_results_[robot_id];
}

int WirelessNetwork::nRobots() const {
    return robots_.GetN();
}

ns3::NodeContainer WirelessNetwork::robots() const {
    return robots_;
}

ns3::Ptr<ns3::Node> WirelessNetwork::mecServer() const {
    return mec_server_;
}

void WirelessNetwork::setRobotPosition(int robot_id, const ns3::Vector &position) {
    setNodePosition(robots_.Get(robot_id), position);
}

std::pair<int,int> WirelessNetwork::getRobotRoom(int robot_id) const {
    ns3::Ptr<ns3::Node> robot = robots_.Get(robot_id);
    ns3::Ptr<ns3::MobilityBuildingInfo> mobility_building_info = robot->GetObject<ns3::MobilityBuildingInfo>();
    int room_x = mobility_building_info->GetRoomNumberX();
    int room_y = mobility_building_info->GetRoomNumberY();
    return {room_x, room_y};
}

void WirelessNetwork::setNodePosition(const ns3::Ptr<ns3::Node> &node, const ns3::Vector &position) {
    ns3::Ptr<ns3::MobilityModel> mobility_model = node->GetObject<ns3::MobilityModel>();
    ns3::Ptr<ns3::MobilityBuildingInfo> mobility_building_info = node->GetObject<ns3::MobilityBuildingInfo>();
    mobility_model->SetPosition(position);
    mobility_building_info->MakeConsistent(mobility_model);
}

void WirelessNetwork::addMobility(const ns3::NodeContainer &nodes) {
    ns3::MobilityHelper mobility_helper;
    mobility_helper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility_helper.Install(nodes);
    ns3::BuildingsHelper::Install(nodes);
}

void WirelessNetwork::addInternetStack(const ns3::NodeContainer &nodes) {
    ns3::InternetStackHelper internet_stack_helper;
    internet_stack_helper.Install(nodes);
}