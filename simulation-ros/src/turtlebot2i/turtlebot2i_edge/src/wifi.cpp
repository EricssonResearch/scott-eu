#include <turtlebot2i_edge/wifi.h>
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"

WifiNetwork::WifiNetwork(int n_robots, int n_congesting_nodes) : WirelessNetwork(n_robots, n_congesting_nodes) {}

void WifiNetwork::createNetwork() {
    ns3::Ptr<ns3::Node> ap = baseStation();
    ns3::NodeContainer sta_nodes(robots(), congestingNodes());
    ns3::NodeContainer p2p_nodes(ap, mecServer());

    // WiFi general settings
    ns3::WifiHelper wifi_helper;
    wifi_helper.SetStandard(ns3::WIFI_STANDARD_80211n_5GHZ);    // high throughput
    wifi_helper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                        "DataMode", ns3::StringValue("HtMcs7"),
                                        "ControlMode", ns3::StringValue("HtMcs0"));

    // WiFi PHY
    ns3::YansWifiChannelHelper channel_helper = ns3::YansWifiChannelHelper::Default();
    ns3::Ptr<ns3::YansWifiChannel> channel = channel_helper.Create();
    ns3::YansWifiPhyHelper phy_helper;
    phy_helper.SetChannel(channel);

    // WiFi MAC
    ns3::Ssid ssid("hrc-warehouse");
    ns3::WifiMacHelper mac_helper_sta, mac_helper_ap;
    mac_helper_sta.SetType("ns3::StaWifiMac",
                           "Ssid", ns3::SsidValue(ssid),
                           "ActiveProbing", ns3::BooleanValue(false));
    mac_helper_ap.SetType("ns3::ApWifiMac",
                          "Ssid", ns3::SsidValue(ssid));

    // P2P general settings
    ns3::PointToPointHelper point_to_point_helper;
    point_to_point_helper.SetDeviceAttribute("DataRate", ns3::StringValue("100Mbps"));
    point_to_point_helper.SetChannelAttribute("Delay", ns3::StringValue("2ms"));

    // install net devices
    ns3::NetDeviceContainer net_devices_sta = wifi_helper.Install(phy_helper, mac_helper_sta, sta_nodes);
    ns3::NetDeviceContainer net_devices_ap = wifi_helper.Install(phy_helper, mac_helper_ap, ap);
    ns3::NetDeviceContainer net_devices_wifi(net_devices_sta, net_devices_ap);
    ns3::NetDeviceContainer net_devices_p2p = point_to_point_helper.Install(p2p_nodes);

    // assign IP addresses
    ns3::Ipv4AddressHelper ipv4_address_helper;
    ipv4_address_helper.SetBase("10.0.0.0", "/24");
    ipv4_address_helper.Assign(net_devices_wifi);
    ipv4_address_helper.SetBase("10.0.1.0", "/30");
    ipv4_address_helper.Assign(net_devices_p2p);
}
