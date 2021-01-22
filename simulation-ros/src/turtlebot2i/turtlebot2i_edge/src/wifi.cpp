#include <turtlebot2i_edge/wifi.h>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

WifiNetwork::WifiNetwork(int n_robots) : WirelessNetwork(n_robots) {
    addMobility(mecServer());
}

void WifiNetwork::createNetwork() {
    ns3::NodeContainer robots = WirelessNetwork::robots();
    ns3::Ptr<ns3::Node> mec_server = WirelessNetwork::mecServer();
    ns3::NodeContainer nodes(robots, mec_server);

    // general settings
    ns3::WifiHelper wifi_helper;
    wifi_helper.SetStandard(ns3::WIFI_STANDARD_80211n_5GHZ);    // throughput 100 Mbps, TODO: check with simulation
    wifi_helper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                        "DataMode", ns3::StringValue("HtMcs7"),
                                        "ControlMode", ns3::StringValue("HtMcs0"));

    // PHY
    ns3::YansWifiChannelHelper channel_helper = ns3::YansWifiChannelHelper::Default();
    ns3::Ptr<ns3::YansWifiChannel> channel = channel_helper.Create();
    ns3::YansWifiPhyHelper phy_helper;
    phy_helper.SetChannel(channel);

    // MAC
    ns3::WifiMacHelper mac_helper;
    mac_helper.SetType("ns3::AdhocWifiMac");

    // install net devices
    ns3::NetDeviceContainer net_devices = wifi_helper.Install(phy_helper, mac_helper, nodes);

    // install TapBridge net devices to communicate with OS's taps
    ns3::TapBridgeHelper tap_bridge_helper;
    tap_bridge_helper.SetAttribute("Mode", ns3::StringValue("UseLocal"));
    for (int i=0; i<nodes.GetN(); i++) {
        std::string tap = std::string("w") + std::to_string(i) + "tap";
        tap_bridge_helper.SetAttribute("DeviceName", ns3::StringValue(tap));
        tap_bridge_helper.Install(nodes.Get(i), net_devices.Get(i));
    }
}

void WifiNetwork::setMecServerPosition(const ns3::Vector &position) {
    setNodePosition(mecServer(), position);
}
