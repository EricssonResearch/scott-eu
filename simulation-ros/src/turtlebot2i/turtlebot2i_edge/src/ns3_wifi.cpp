/*
 * Simulates the following network with ns-3:
 *
 *              WiFi 10.0.0.0/24
 *      *       *       ...     *       *
 *      |       |               |       |    10.2.0.0/30
 *   robot1  robot2          robotN     AP --------------- MEC server
 *
 * The robot ROS nodes must be run within network namespaces set up with scripts/nns_wifi.py.
 * The positions of the robots are updated according to the positions in V-REP.
 *
 * Adapted from:
 * - https://github.com/nps-ros2/ns3_gazebo/blob/master/ns3_gazebo_ws/src/diff_drive_ns3/src/diff_drive_ns3_ros2.cpp
 * - https://www.nsnam.org/doxygen/tap-wifi-virtual-machine_8cc_source.html
 * - https://www.nsnam.org/doxygen/third_8cc_source.html
 *
 * Reference: https://www.nsnam.org/wiki/HOWTO_Use_Linux_Containers_to_set_up_virtual_networks
 */

#include <iostream>
#include <thread>

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/internet-module.h>
#include <ns3/wifi-module.h>
#include <ns3/point-to-point-module.h>
#include <ns3/tap-bridge-module.h>
#include <ns3/mobility-module.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

static const int COUNT=2;   // TODO

ns3::NodeContainer create_nodes(int n) {
    // create nodes
    ns3::NodeContainer nodes;
    nodes.Create(n);

    // install internet stack
    ns3::InternetStackHelper stack;
    stack.Install(nodes);

    return nodes;
}

void setup_wifi(const ns3::Ptr<ns3::Node>& ap, ns3::NodeContainer& stations) {
    // settings
    ns3::WifiHelper wifi;
    wifi.SetStandard(ns3::WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", ns3::StringValue("OfdmRate54Mbps"));
    ns3::Ssid ssid = ns3::Ssid("hrc-warehouse");

    // PHY
    ns3::YansWifiPhyHelper phy;
    ns3::YansWifiChannelHelper wifi_channel = ns3::YansWifiChannelHelper::Default();
    phy.SetChannel(wifi_channel.Create());

    // MAC
    ns3::WifiMacHelper mac_sta, mac_ap;
    mac_sta.SetType("ns3::StaWifiMac", "Ssid", ns3::SsidValue(ssid), "ActiveProbing", ns3::BooleanValue(false));
    mac_ap.SetType("ns3::ApWifiMac", "Ssid", ns3::SsidValue(ssid));

    // install net devices
    ns3::NetDeviceContainer sta_net_devices = wifi.Install(phy, mac_sta, stations);
    ns3::Ptr<ns3::NetDevice> ap_net_device = wifi.Install(phy, mac_ap, ap).Get(0);

    // assign IP address to AP
    ns3::Ptr<ns3::Ipv4> ipv4 = ap->GetObject<ns3::Ipv4>();
    int32_t if_index = ipv4->AddInterface(ap_net_device);
    ns3::Ipv4InterfaceAddress ipv4_address("10.0.0.254", "/24");
    ipv4->AddAddress(if_index, ipv4_address);

    // install special TapBridge net devices to communicate with OS's taps (connection to network namespaces)
    ns3::TapBridgeHelper tapBridge;
    tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));
    for (int i=0; i<COUNT; i++) {
        std::string tap = std::string("w") + std::to_string(i) + "tap";
        tapBridge.SetAttribute("DeviceName", ns3::StringValue(tap));
        tapBridge.Install(stations.Get(i), sta_net_devices.Get(i));
    }

    // add mobility model
    ns3::MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(stations);
    mobility.Install(ap);

    // set position of AP
    ns3::Ptr<ns3::MobilityModel> mobility_model = ap->GetObject<ns3::MobilityModel>();  // TODO: funziona? senno' devo mettere ConstantPositionMobilityModel
    mobility_model->SetPosition({0, 0, 0});     // TODO
}

void setup_p2p(ns3::NodeContainer& nodes) {
    // settings
    ns3::PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", ns3::StringValue("5Mbps"));    // TODO
    p2p.SetChannelAttribute("Delay", ns3::StringValue("2ms"));        // TODO

    // install net devices
    ns3::NetDeviceContainer net_devices;
    net_devices = p2p.Install(nodes);

    // assign IP addresses
    ns3::Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.2.0.0", "/30");
    ipv4.Assign(net_devices);
}

ns3::NodeContainer setup_network() {
    ns3::GlobalValue::Bind("SimulatorImplementationType", ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

    ns3::NodeContainer robots = create_nodes(COUNT);
    ns3::NodeContainer p2p_nodes = create_nodes(2);
    ns3::Ptr<ns3::Node> ap = p2p_nodes.Get(0);

    setup_wifi(ap, robots);
    setup_p2p(p2p_nodes);

    ns3::Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    return robots;
}

void update_position(const ns3::Ptr<ns3::Node>& robot, const nav_msgs::Odometry::ConstPtr& msg) {
    ns3::Ptr<ns3::MobilityModel> mobility_model = robot->GetObject<ns3::MobilityModel>();    // TODO: funziona? senno' devo mettere ConstantPositionMobilityModel
    const geometry_msgs::Point& position = msg->pose.pose.position;
    mobility_model->SetPosition({position.x, position.y, position.z});
    ROS_INFO("Position updated: {%f, %f, %f}", position.x, position.y, position.z);
}

void run_ros_node(ns3::NodeContainer& robots) {
    ros::NodeHandle node_handle;
    for (auto it=robots.Begin(); it != robots.End(); it++) {
        ns3::Ptr<ns3::Node> robot = *it;
        auto callback = [robot](const nav_msgs::Odometry::ConstPtr& msg) { update_position(robot, msg); };
        ros::Subscriber subscriber = node_handle.subscribe<nav_msgs::Odometry>("odom", 1000, callback);
    }
    ros::spin();
}

int main(int argc, char **argv) {
    ns3::NodeContainer robots = setup_network();
    ros::init(argc, argv, "ns3_wifi");

    ROS_INFO("Starting ROS node...");
    std::thread thread(run_ros_node, std::ref(robots));     // on a second thread, because...

    ROS_INFO("Starting ns-3 simulation...");
    ns3::Simulator::Run();      // ...this thread simulates the network

    thread.join();              // gracefully let the robot thread stop
    ns3::Simulator::Destroy();  // after joining, otherwise the other thread uses invalid nodes

    return 0;
}
