#include <turtlebot2i_edge/wireless.h>
#include "ns3/mobility-module.h"

WirelessNetwork::WirelessNetwork(int n_robots) {
    ns3::GlobalValue::Bind("SimulatorImplementationType", ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

    robots_ = ns3::NodeContainer(n_robots);
    mec_server_ = ns3::CreateObject<ns3::Node>();

    addMobility(robots_);
}

WirelessNetwork::~WirelessNetwork() {
    ns3::Simulator::Destroy();  // TODO: try if it gives problems if the simulation is not started
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
    ns3::Simulator::Run();
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
