#include <thread>
#include <turtlebot2i_edge/wireless.h>
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

WirelessNetwork::WirelessNetwork(int n_robots) : sending_(n_robots), sending_mutex_(n_robots), sending_cv_(n_robots) {
    ns3::GlobalValue::Bind("SimulatorImplementationType", ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

    robots_ = ns3::NodeContainer(n_robots);
    mec_server_ = ns3::CreateObject<ns3::Node>();

    addMobility(robots_);
    addInternetStack(robots_);
    addInternetStack(mec_server_);
}

WirelessNetwork::~WirelessNetwork() {
    ns3::Simulator::Destroy();
}

void WirelessNetwork::createApplications() {
    ns3::Ipv4Address any = ns3::Ipv4Address::GetAny();
    ns3::PacketSinkHelper packet_sink_helper("ns3::TcpSocketFactory", ns3::InetSocketAddress(any, 2000));
    ns3::ApplicationContainer packet_sinks = packet_sink_helper.Install(mec_server_);
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

void WirelessNetwork::transferCompleted(const ns3::Ptr<ns3::Node> &robot) {
    // get robot_id
    auto it = std::find_if(robots_.Begin(), robots_.End(), [robot](const ns3::Ptr<ns3::Node> &r) { return r == robot; });
    int robot_id = it - robots_.Begin();

    // save stats and notify
    std::unique_lock<std::mutex> ul(sending_mutex_[robot_id]);
    sending_[robot_id] = false;
    sending_cv_[robot_id].notify_one();
}

void WirelessNetwork::offload(int robot_id, int n_bytes) {
    /*
     * Important: the callback transferCompleted() might be called either from the current thread or from the thread
     * simulating the network (blocked in ns3::Simulator::Run()). It depends on the implementation of the ns-3 core, but
     * we cannot make assumptions (even because the behavior may change in the future). Therefore, we must synchronize
     * the threads using mutex and condition variable. If the callback is called from the current thread, mutex and
     * condition variable are superfluous but the code still works.
     *
     * Note also that the correct interface of the MEC server is the second one (index 1), since we install the internet
     * stack in the constructor (see https://www.nsnam.org/wiki/HOWTO_use_IP_interface_indexes).
     */

    ns3::Ptr<ns3::Node> robot = robots_.Get(robot_id);

    // install app
    ns3::Ipv4Address dest_ip = mec_server_->GetObject<ns3::Ipv4>()->GetAddress(1, 0).GetLocal();
    ns3::BulkSendHelper bulk_send_helper("ns3::TcpSocketFactory", ns3::InetSocketAddress(dest_ip, 2000));
    bulk_send_helper.SetAttribute("MaxBytes", ns3::UintegerValue(n_bytes));
    ns3::ApplicationContainer apps = bulk_send_helper.Install(robot);

    // transfer bytes
    std::unique_lock<std::mutex> ul(sending_mutex_[robot_id]);
    sending_[robot_id] = true;
    ul.unlock();
    ns3::Ptr<ns3::BulkSendApplication> app = ns3::StaticCast<ns3::BulkSendApplication>(apps.Get(0));
    app->TraceConnectWithoutContext("Latency", ns3::MakeCallback(&WirelessNetwork::transferCompleted, this));
    app->Send();

    // wait for completion
    ul.lock();
    sending_cv_[robot_id].wait(ul, [this, robot_id]() { return !sending_[robot_id]; });
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
