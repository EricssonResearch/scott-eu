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
#include <ns3/netanim-module.h>

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

    // set position of mec_server
    ns3::Ptr<ns3::MobilityModel> mobility_model = mec_server->GetObject<ns3::MobilityModel>();
    mobility_model->SetPosition(mec_server_pos);
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
    for (auto it=robots.Begin(); it != robots.End(); it++) {
        ns3::Ptr<ns3::Node> robot = *it;
        std::string topic = "odom" + std::to_string(robot->GetId());
        auto callback = [robot](const nav_msgs::Odometry::ConstPtr& msg) { updatePosition(robot, msg); };
        ros::Subscriber subscriber = node_handle.subscribe<nav_msgs::Odometry>(topic, 1, callback);
        subscribers.push_back(subscriber);
    }
    ros::spin();
}

int main(int argc, char **argv) {
    int n_robots = 1;
    ns3::Vector mec_server_pos = {0, 0, 0};

    ROS_INFO("Parsing command line arguments...");
    ros::init(argc, argv, "ns3_wifi_adhoc");
    ns3::CommandLine cmd(__FILE__);
    cmd.AddValue("robots", "Number of robots", n_robots);
    cmd.Parse (argc, argv);

    ROS_INFO("Getting parameters from parameter server...");
    if (!ros::param::get("mec_server/position/x", mec_server_pos.x) ||
        !ros::param::get("mec_server/position/y", mec_server_pos.y) ||
        !ros::param::get("mec_server/position/z", mec_server_pos.z)) {
        ROS_ERROR("Parameters for MEC server not present");
        return 1;
    }

    ROS_INFO("Setting up network...");
    ROS_INFO("Number of robots: %d", n_robots);
    ROS_INFO("Position of MEC server: {%f, %f, %f}", mec_server_pos.x, mec_server_pos.y, mec_server_pos.z);
    std::pair<ns3::NodeContainer,ns3::NodeContainer> nodes = setupNetwork(n_robots, mec_server_pos);
    ns3::NodeContainer& robots = nodes.first;

    ROS_INFO("Starting ROS node...");
    std::thread thread(runRosNode, robots);     // on a second thread, because...

    ROS_INFO("Starting ns-3 simulation...");
    ns3::Simulator::Run();                      // ...this thread simulates the network

    thread.join();                              // gracefully let the robot thread stop
    ns3::Simulator::Destroy();                  // after joining, otherwise the other thread uses invalid nodes

    return 0;
}
