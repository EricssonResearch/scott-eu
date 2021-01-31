#include <thread>
#include <sstream>
#include <turtlebot2i_edge/wireless.h>
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"

WirelessNetwork::WirelessNetwork(int n_robots) :
        uploading_(n_robots), to_upload_(n_robots), uploaded_(n_robots), uploading_mutex_(n_robots),
        uploading_cv_(n_robots), downloading_(n_robots), to_download_(n_robots), downloaded_(n_robots),
        downloading_mutex_(n_robots), downloading_cv_(n_robots),  pinging_(n_robots), rtt_(n_robots),
        pinging_mutex_(n_robots), pinging_cv_(n_robots) {
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
     * Note that the correct interface for the IP address is the second one (index 1), the first one is the loopback
     * interface. This is because we install the internet stack in the constructor, before installing the net device.
     *
     * See https://www.nsnam.org/wiki/HOWTO_use_IP_interface_indexes.
     */

    ns3::ApplicationContainer apps;
    ns3::Ptr<ns3::Application> app;
    ns3::Ipv4Address any = ns3::Ipv4Address::GetAny();
    ns3::Ipv4Address server_address = mec_server_->GetObject<ns3::Ipv4>()->GetAddress(1, 0).GetLocal();

    // apps for sending bytes in upload (robot -> MEC server)
    ns3::BulkSendHelper bulk_send_helper("ns3::TcpSocketFactory", ns3::InetSocketAddress(server_address, 2000));
    bulk_send_helper.SetAttribute("EnableSeqTsSizeHeader", ns3::BooleanValue(true));
    apps = bulk_send_helper.Install(robots_);
    apps.Stop(ns3::Seconds(0.1));

    // apps for sending bytes in download (MEC server -> robot)
    for (int i=0; i<robots_.GetN(); i++) {
        ns3::Ptr<ns3::Node> robot = robots_.Get(i);
        ns3::Ipv4Address robot_address = robot->GetObject<ns3::Ipv4>()->GetAddress(1, 0).GetLocal();
        bulk_send_helper = ns3::BulkSendHelper("ns3::TcpSocketFactory", ns3::InetSocketAddress(robot_address, 2000));
        bulk_send_helper.SetAttribute("EnableSeqTsSizeHeader", ns3::BooleanValue(true));
        apps = bulk_send_helper.Install(mec_server_);
        apps.Stop(ns3::Seconds(0.1));
    }

    // apps for pinging (robot -> MEC server)
    ns3::UdpEchoClientHelper udp_echo_client_helper(server_address, 9);
    udp_echo_client_helper.SetAttribute("MaxPackets", ns3::UintegerValue(1));
    udp_echo_client_helper.SetAttribute("PacketSize", ns3::UintegerValue(56));  // like Linux ping
    apps = udp_echo_client_helper.Install(robots_);
    apps.Stop(ns3::Seconds(0.1));
    for (auto it=apps.Begin(); it!=apps.End(); it++) {
        app = *it;
        app->TraceConnectWithoutContext("RxWithTime", ns3::MakeCallback(&WirelessNetwork::saveRtt, this));
    }

    // apps for receiving bytes
    ns3::PacketSinkHelper packet_sink_helper("ns3::TcpSocketFactory", ns3::InetSocketAddress(any, 2000));
    packet_sink_helper.SetAttribute("EnableSeqTsSizeHeader", ns3::BooleanValue(true));
    apps = packet_sink_helper.Install(mec_server_);         // upload
    app = apps.Get(0);
    app->TraceConnectWithoutContext("RxWithSeqTsSize", ns3::MakeCallback(&WirelessNetwork::updateUploadedBytes, this));
    apps = packet_sink_helper.Install(robots_);             // download
    for (auto it=apps.Begin(); it!=apps.End(); it++) {
        app = *it;
        app->TraceConnectWithoutContext("RxWithSeqTsSize", ns3::MakeCallback(&WirelessNetwork::updateDownloadedBytes, this));
    }

    // app for receiving ping
    ns3::UdpEchoServerHelper udp_echo_server_helper(9);
    udp_echo_server_helper.Install(mec_server_);
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
    ns3::Ipv4Address robot_address;
    if (ns3::Ipv4Address::IsMatchingType(address))
        robot_address = ns3::Ipv4Address::ConvertFrom(address);
    else if (ns3::InetSocketAddress::IsMatchingType(address))
        robot_address = ns3::InetSocketAddress::ConvertFrom(address).GetIpv4();
    else
        throw std::logic_error("Non-matching address type");

    auto it = std::find_if(robots_.Begin(), robots_.End(), [robot_address](const ns3::Ptr<ns3::Node> &robot) {
        ns3::Ipv4Address robot_address_ = robot->GetObject<ns3::Ipv4>()->GetAddress(1, 0).GetLocal();
        return robot_address_ == robot_address;
    });
    if (it == robots_.End()) {
        std::ostringstream oss;
        oss << "No robot associated to address " << robot_address;
        throw std::logic_error(oss.str());
    }
    int robot_id = it - robots_.Begin();
    return robot_id;
}

void WirelessNetwork::updateUploadedBytes(ns3::Ptr<const ns3::Packet> packet, const ns3::Address &robot_address,
                                          const ns3::Address &server_address, const ns3::SeqTsSizeHeader &header) {
    (void) packet;
    (void) server_address;
    int robot_id = getRobotId(robot_address);
    int n_bytes = header.GetSize();

    std::unique_lock<std::mutex> ul(uploading_mutex_[robot_id]);
    uploaded_[robot_id] += n_bytes;
    if (uploaded_[robot_id] >= to_upload_[robot_id]) {
        uploading_[robot_id] = false;
        uploading_cv_[robot_id].notify_one();
    }
}

int WirelessNetwork::upload(int robot_id, int n_bytes, const ns3::Time &max_duration) {
    /*
     * Important: the callback transferCompleted() might be called either from the current thread or from the thread
     * simulating the network (blocked in ns3::Simulator::Run()). It depends on the implementation of the ns-3 core,
     * but we cannot make assumptions (even because the behavior may change in the future). Therefore, we must
     * synchronize the threads using mutex and condition variable. If the callback is called from the current thread,
     * mutex and condition variable are superfluous but the code still works.
     */

    ns3::Ptr<ns3::Node> robot = robots_.Get(robot_id);
    ns3::Ptr<ns3::BulkSendApplication> app = ns3::StaticCast<ns3::BulkSendApplication>(robot->GetApplication(0));
    int max_duration_ = max_duration.GetMilliSeconds();

    std::unique_lock<std::mutex> ul(uploading_mutex_[robot_id]);
    uploaded_[robot_id] = 0;
    to_upload_[robot_id] = n_bytes;
    uploading_[robot_id] = true;
    ul.unlock();

    app->SetMaxBytes(n_bytes);
    app->Start(ns3::Seconds(0));

    ul.lock();
    if (max_duration_ == 0)
        uploading_cv_[robot_id].wait(ul, [this, robot_id]() { return !uploading_[robot_id]; });
    else
        uploading_cv_[robot_id].wait_for(ul, std::chrono::milliseconds(max_duration_),
                                         [this, robot_id]() { return !uploading_[robot_id]; });
    int stamped = uploaded_[robot_id];
    ul.unlock();

    app->Stop(ns3::Seconds(0));
    return stamped;
}

void WirelessNetwork::updateDownloadedBytes(ns3::Ptr<const ns3::Packet> packet, const ns3::Address &server_address,
                                            const ns3::Address &robot_address, const ns3::SeqTsSizeHeader &header) {
    (void) packet;
    (void) server_address;
    int robot_id = getRobotId(robot_address);
    int n_bytes = header.GetSize();

    std::unique_lock<std::mutex> ul(downloading_mutex_[robot_id]);
    downloaded_[robot_id] += n_bytes;
    if (downloaded_[robot_id] >= to_download_[robot_id]) {
        downloading_[robot_id] = false;
        downloading_cv_[robot_id].notify_one();
    }
}

int WirelessNetwork::download(int robot_id, int n_bytes, const ns3::Time &max_duration) {
    ns3::Ptr<ns3::Application> app_ = mec_server_->GetApplication(robot_id);
    ns3::Ptr<ns3::BulkSendApplication> app = ns3::StaticCast<ns3::BulkSendApplication>(app_);
    int max_duration_ = max_duration.GetMilliSeconds();

    std::unique_lock<std::mutex> ul(downloading_mutex_[robot_id]);
    downloaded_[robot_id] = 0;
    to_download_[robot_id] = n_bytes;
    downloading_[robot_id] = true;
    ul.unlock();

    app->SetMaxBytes(n_bytes);
    app->Start(ns3::Seconds(0));

    ul.lock();
    if (max_duration_ == 0)
        downloading_cv_[robot_id].wait(ul, [this, robot_id]() { return !downloading_[robot_id]; });
    else
        downloading_cv_[robot_id].wait_for(ul, std::chrono::milliseconds(max_duration_),
                                           [this, robot_id]() { return !downloading_[robot_id]; });
    int stamped = downloaded_[robot_id];
    ul.unlock();

    app->Stop(ns3::Seconds(0));
    return stamped;
}

void WirelessNetwork::saveRtt(const ns3::Ptr<ns3::Node> &robot, const ns3::Time &time) {
    int robot_id = robot->GetId();
    std::unique_lock<std::mutex> ul(pinging_mutex_[robot_id]);
    rtt_[robot_id] = time.GetSeconds() * 1e3 - rtt_[robot_id];
    pinging_[robot_id] = false;
    pinging_cv_[robot_id].notify_one();
}

double WirelessNetwork::ping(int robot_id, const ns3::Time &max_rtt) {
    ns3::Ptr<ns3::Node> robot = robots_.Get(robot_id);
    ns3::Ptr<ns3::UdpEchoClient> app = ns3::StaticCast<ns3::UdpEchoClient>(robot->GetApplication(1));
    int max_rtt_ = max_rtt.GetMilliSeconds();

    std::unique_lock<std::mutex> ul(pinging_mutex_[robot_id]);
    rtt_[robot_id] = ns3::Simulator::Now().GetSeconds() * 1e3;  // not GetMilliSeconds() because it rounds to integer
    pinging_[robot_id] = true;
    ul.unlock();

    app->Start(ns3::Seconds(0));

    ul.lock();
    if (max_rtt_ == 0)
        pinging_cv_[robot_id].wait(ul, [this, robot_id]() { return !pinging_[robot_id]; });
    else
        pinging_cv_[robot_id].wait_for(ul, std::chrono::milliseconds(max_rtt_),
                                       [this, robot_id]() { return !pinging_[robot_id]; });
    double rtt = pinging_[robot_id] ? (max_rtt_+1) : rtt_[robot_id];
    ul.unlock();

    app->Stop(ns3::Seconds(0));
    return rtt;
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