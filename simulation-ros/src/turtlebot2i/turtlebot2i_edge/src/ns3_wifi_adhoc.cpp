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

void setup_wifi(const ns3::NodeContainer &robots, const ns3::Ptr<ns3::Node> &server, const ns3::Vector &server_pos) {
    ns3::NodeContainer nodes(robots, server);

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

    // set position of server
    ns3::Ptr<ns3::MobilityModel> mobility_model = server->GetObject<ns3::MobilityModel>();
    mobility_model->SetPosition(server_pos);
}

std::pair<ns3::NodeContainer,ns3::NodeContainer> setup_network(int n_robots, ns3::Vector server_pos) {
    ns3::GlobalValue::Bind("SimulatorImplementationType", ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

    ns3::NodeContainer robots(n_robots);
    ns3::Ptr<ns3::Node> server = ns3::CreateObject<ns3::Node>();
    setup_wifi(robots, server, server_pos);

    return {robots, server};
}

void update_position(const ns3::Ptr<ns3::Node> &robot, const nav_msgs::Odometry::ConstPtr &msg) {
    ns3::Ptr<ns3::MobilityModel> mobility_model = robot->GetObject<ns3::MobilityModel>();
    const geometry_msgs::Point& position = msg->pose.pose.position;
    mobility_model->SetPosition({position.x, position.y, position.z});
    ROS_INFO("Position of robot %d: {%f, %f, %f}", robot->GetId(), position.x, position.y, position.z);
}

void run_ros_node(const ns3::NodeContainer &robots) {
    ros::NodeHandle node_handle;
    std::vector<ros::Subscriber> subscribers;
    for (auto it=robots.Begin(); it != robots.End(); it++) {
        ns3::Ptr<ns3::Node> robot = *it;
        std::string topic = "odom" + std::to_string(robot->GetId());
        auto callback = [robot](const nav_msgs::Odometry::ConstPtr& msg) { update_position(robot, msg); };
        ros::Subscriber subscriber = node_handle.subscribe<nav_msgs::Odometry>(topic, 1, callback);
        subscribers.push_back(subscriber);
    }
    ros::spin();
}

int main(int argc, char **argv) {
    int n_robots = 1;
    ns3::Vector server_pos = {0, 0, 0};

    ROS_INFO("Parsing command line arguments...");
    ros::init(argc, argv, "ns3_wifi_adhoc");
    ns3::CommandLine cmd(__FILE__);
    cmd.AddValue("robots", "Number of robots", n_robots);
    cmd.AddValue("server-x", "x-coordinate of server", server_pos.x);
    cmd.AddValue("server-y", "x-coordinate of server", server_pos.y);
    cmd.AddValue("server-z", "x-coordinate of server", server_pos.z);
    cmd.Parse (argc, argv);

    ROS_INFO("Setting up network...");
    std::pair<ns3::NodeContainer,ns3::NodeContainer> nodes = setup_network(n_robots, server_pos);
    ns3::NodeContainer& robots = nodes.first;

    ROS_INFO("Starting ROS node...");
    std::thread thread(run_ros_node, robots);   // on a second thread, because...

    ROS_INFO("Starting ns-3 simulation...");
    ns3::Simulator::Run();      // ...this thread simulates the network

    thread.join();              // gracefully let the robot thread stop
    ns3::Simulator::Destroy();  // after joining, otherwise the other thread uses invalid nodes

    return 0;
}