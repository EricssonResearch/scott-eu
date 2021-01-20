/*
 * Simulates the following network with ns-3:
 *
 *                  WiFi
 *      *       *       ...     *       *
 *      |       |               |       |
 *   robot1  robot2          robotN  MEC server
 *
 * TODO
 */

#include <iostream>
#include <thread>
#include <vector>

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/wifi-module.h>
#include <ns3/point-to-point-module.h>
#include <ns3/tap-bridge-module.h>
#include <ns3/mobility-module.h>
#include <ns3/buildings-module.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

void setupWifi(const ns3::NodeContainer &robots, const ns3::Ptr<ns3::Node> &mec_server, const ns3::Vector &mec_server_pos) {
    ns3::NodeContainer nodes(robots, mec_server);

    // settings
    ns3::WifiHelper wifi;
    wifi.SetStandard(ns3::WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", ns3::StringValue("OfdmRate54Mbps"));

    // PHY
    ns3::YansWifiPhyHelper phy;
    ns3::YansWifiChannelHelper wifi_channel = ns3::YansWifiChannelHelper::Default();
    phy.SetChannel(wifi_channel.Create());

    // MAC
    ns3::WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // install net devices
    ns3::NetDeviceContainer net_devices = wifi.Install(phy, mac, nodes);

    // install TapBridge net devices to communicate with OS's taps
    ns3::TapBridgeHelper tapBridge;
    tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));
    for (int i=0; i<nodes.GetN(); i++) {
        std::string tap = std::string("w") + std::to_string(i) + "tap";
        tapBridge.SetAttribute("DeviceName", ns3::StringValue(tap));
        tapBridge.Install(nodes.Get(i), net_devices.Get(i));
    }

    // add mobility model
    ns3::MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
    ns3::BuildingsHelper::Install(nodes);

    // set position of mec_server
    ns3::Ptr<ns3::MobilityModel> mobility_model = mec_server->GetObject<ns3::MobilityModel>();
    mobility_model->SetPosition(mec_server_pos);
}

ns3::Ptr<ns3::Building> setupWarehouse(const ns3::Box &boundaries, int n_rooms_x, int n_rooms_y) {
    ns3::Ptr<ns3::Building> warehouse = ns3::CreateObject<ns3::Building>();
    warehouse->SetBoundaries(boundaries);
    warehouse->SetBuildingType (ns3::Building::Commercial);
    warehouse->SetExtWallsType (ns3::Building::ConcreteWithWindows);
    warehouse->SetNFloors(1);
    warehouse->SetNRoomsX(n_rooms_x);
    warehouse->SetNRoomsY(n_rooms_y);
    return warehouse;
}

std::pair<ns3::NodeContainer,ns3::NodeContainer> setupNetwork(int n_robots, ns3::Vector mec_server_pos) {
    ns3::GlobalValue::Bind("SimulatorImplementationType", ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

    ns3::NodeContainer robots(n_robots);
    ns3::Ptr<ns3::Node> mec_server = ns3::CreateObject<ns3::Node>();
    setupWifi(robots, mec_server, mec_server_pos);

    return {robots, mec_server};
}

void updatePosition(const ns3::Ptr<ns3::Node> &robot, const nav_msgs::Odometry::ConstPtr &msg) {
    ns3::Ptr<ns3::MobilityModel> mobility_model = robot->GetObject<ns3::MobilityModel>();
    const geometry_msgs::Point& position = msg->pose.pose.position;
    mobility_model->SetPosition({position.x, position.y, position.z});
    ROS_INFO("Position of Robot %d: {%f, %f, %f}", robot->GetId(), position.x, position.y, position.z);
}

void runRosNode(const ns3::NodeContainer &robots) {
    ros::NodeHandle node_handle;
    std::vector<ros::Subscriber> subscribers;
    for (int i=0; i<robots.GetN(); i++) {
        ns3::Ptr<ns3::Node> robot = robots.Get(i);
        std::string topic = "odom_" + std::to_string(i);
        auto callback = [robot](const nav_msgs::Odometry::ConstPtr& msg) { updatePosition(robot, msg); };
        ros::Subscriber subscriber = node_handle.subscribe<nav_msgs::Odometry>(topic, 1, callback);
        subscribers.push_back(subscriber);
    }
    ros::spin();
}

int main(int argc, char **argv) {
    ns3::Vector mec_server_pos;
    ns3::Box warehouse_boundaries;
    int n_robots, n_rooms_x, n_rooms_y;

    ros::init(argc, argv, "ns3_wifi_adhoc");

    ROS_INFO("Getting parameters from parameter server...");
    if (!ros::param::get("/network/mec_server/position/x", mec_server_pos.x) ||
        !ros::param::get("/network/mec_server/position/y", mec_server_pos.y) ||
        !ros::param::get("/network/robots/n", n_robots) ||
        !ros::param::get("/network/warehouse/rooms/x_min", warehouse_boundaries.xMin) ||
        !ros::param::get("/network/warehouse/rooms/x_max", warehouse_boundaries.xMax) ||
        !ros::param::get("/network/warehouse/rooms/y_min", warehouse_boundaries.yMin) ||
        !ros::param::get("/network/warehouse/rooms/y_max", warehouse_boundaries.yMax) ||
        !ros::param::get("/network/warehouse/rooms/z_min", warehouse_boundaries.zMin) ||
        !ros::param::get("/network/warehouse/rooms/z_max", warehouse_boundaries.zMax) ||
        !ros::param::get("/network/warehouse/rooms/n_x", n_rooms_x) ||
        !ros::param::get("/network/warehouse/rooms/n_y", n_rooms_y)) {
        ROS_ERROR("ROS parameter server does not contain the necessary parameters");
        return -1;
    }
    ROS_INFO("Number of robots: %d", n_robots);
    ROS_INFO("Position of MEC server: {%f, %f}", mec_server_pos.x, mec_server_pos.y);

    ROS_INFO("Setting up network...");
    std::pair<ns3::NodeContainer,ns3::NodeContainer> nodes = setupNetwork(n_robots, mec_server_pos);
    ns3::NodeContainer& robots = nodes.first;

    ROS_INFO("Setting up warehouse...");
    ns3::Ptr<ns3::Building> warehouse = setupWarehouse(warehouse_boundaries, n_rooms_x, n_rooms_y);

    ROS_INFO("Starting ROS node...");
    std::thread thread(runRosNode, robots);     // on a second thread, because...

    ROS_INFO("Simulating network...");
    ns3::Simulator::Run();                      // ...this thread simulates the network

    thread.join();                              // gracefully let the robot thread stop
    ns3::Simulator::Destroy();                  // after joining, otherwise the other thread uses invalid nodes

    return 0;
}
