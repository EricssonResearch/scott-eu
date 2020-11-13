/*
 * Runs a simulation of a WiFi network with ns-3 and a ROS node that sends the updated positions of the robots to ns-3.
 *
 * Adapted from: https://github.com/nps-ros2/ns3_gazebo/blob/master/ns3_gazebo_ws/src/diff_drive_ns3/src/diff_drive_ns3_ros2.cpp
 */

#include <cstdio>
#include <memory>
#include <thread>
#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/wifi-module.h>
#include <ns3/tap-bridge-module.h>
#include <ns3/mobility-module.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"

static const int COUNT=2;

void setup_ns3(ns3::NodeContainer& ns3_nodes) {
    // run ns3 real-time with checksums
    ns3::GlobalValue::Bind("SimulatorImplementationType", ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

    // Create ns3 ghost nodes
    ns3_nodes.Create(COUNT);

    // WiFi settings
    ns3::WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", ns3::StringValue("OfdmRate54Mbps"));

    // ad-hoc WiFi network
    ns3::WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    // physical layer
    ns3::YansWifiChannelHelper wifiChannel(ns3::YansWifiChannelHelper::Default());
    ns3::YansWifiPhyHelper wifiPhy(ns3::YansWifiPhyHelper::Default());
    wifiPhy.SetChannel(wifiChannel.Create());

    // Install the wireless devices onto our ghost ns3_nodes.
    ns3::NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, ns3_nodes);

    // antenna locations
    ns3::Ptr<ns3::ListPositionAllocator>positionAlloc =
            ns3::CreateObject<ns3::ListPositionAllocator>();
    for (int i=0; i<COUNT; i++) {
        positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));
    }
    ns3::MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ns3_nodes);

    // connect Wifi through TapBridge devices
    ns3::TapBridgeHelper tapBridge;
    tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));
    char buffer[10];
    for (int i=0; i<COUNT; i++) {
        sprintf(buffer, "wifi_tap%d", i);
        tapBridge.SetAttribute("DeviceName", ns3::StringValue(buffer));
        tapBridge.Install(ns3_nodes.Get(i), devices.Get(i));
    }
}

void refresh_position(ns3::Node& ns3_node, const nav_msgs::Odometry::ConstPtr& msg) {
    ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility_model = ns3_node.GetObject<ns3::ConstantPositionMobilityModel>();
    const geometry_msgs::Point& position = msg->pose.pose.position;
    mobility_model->SetPosition({position.x, position.y, position.z});
    ROS_INFO("Position updated: {%f, %f, %f}", position.x, position.y, position.z);
}

void run_ros_node(ns3::NodeContainer& ns3_nodes) {
    ns3::Ptr<ns3::Node> ns3_node = ns3_nodes.Get(0);
    ROS_INFO("Starting robot in thread.");
    ros::NodeHandle node_handle;
    auto callback = [ns3_node](const nav_msgs::Odometry::ConstPtr& msg) { refresh_position(*ns3_node, msg); };
    ros::Subscriber subscriber = node_handle.subscribe<nav_msgs::Odometry>("odom", 1000, callback);
    ros::spin();
    ROS_INFO("Stopped robot in thread.");
}

int main(int argc, char **argv) {
    std::cout << "Starting ns-3 WiFi simulator and ROS node that sends the updated positions of the robots to ns-3.\n"
              << "Press Ctrl-C twice to stop both.\n" << std::endl;

    // setup simulated network
    ns3::NodeContainer ns3_nodes;
    setup_ns3(ns3_nodes);

    // start ROS node as a second thread
    ros::init(argc, argv, "ros_ns3");
    std::thread robot_thread(run_ros_node, std::ref(ns3_nodes));

    // run until Ctrl-C
    std::cout << "Starting ns-3 Wifi simulator in main.\n";
    ns3::Simulator::Run();
    robot_thread.join();        // gracefully let the robot thread stop
    ns3::Simulator::Destroy();  // after joining, otherwise the other thread uses invalid nodes
    std::cout << "Stopped ns-3 Wifi simulator in main.\n" << "Done." << std::endl;

    return 0;
}
