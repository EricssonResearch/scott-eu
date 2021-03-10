#include <turtlebot2i_edge/wifi.h>
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

WifiNetwork::WifiNetwork(int n_robots, int n_congesting_nodes) :
        WirelessNetwork(n_robots, n_congesting_nodes), snr_measuring_(n_robots), snr_(n_robots), snr_mutex_(n_robots) {
    addMobility(mecServer());
}

void WifiNetwork::createNetwork() {
    ns3::NodeContainer robots = WirelessNetwork::robots();
    ns3::Ptr<ns3::Node> mec_server = WirelessNetwork::mecServer();
    ns3::NodeContainer congesting_nodes = WirelessNetwork::congestingNodes();
    ns3::NodeContainer nodes(robots, mec_server, congesting_nodes);

    // general settings
    ns3::WifiHelper wifi_helper;
    wifi_helper.SetStandard(ns3::WIFI_STANDARD_80211n_5GHZ);    // high throughput
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

    // assign IP addresses
    ns3::Ipv4AddressHelper ipv4_address_helper;
    ipv4_address_helper.SetBase("10.0.0.0", "/24");
    ipv4_address_helper.Assign(net_devices);
}

void WifiNetwork::createApplications() {
    WirelessNetwork::createApplications();

    ns3::Ptr<ns3::Node> mec_server = mecServer();
    ns3::Ptr<ns3::WifiPhy> phy = ns3::StaticCast<ns3::WifiNetDevice>(mec_server->GetDevice(1))->GetPhy();
    phy->TraceConnectWithoutContext("MonitorSnifferRx", ns3::MakeCallback(&WifiNetwork::saveSnr, this));
}

void WifiNetwork::saveSnr(ns3::Ptr<const ns3::Packet> packet, uint16_t channel_frequency, ns3::WifiTxVector tx_vector,
                          ns3::MpduInfo a_mpdu, ns3::SignalNoiseDbm signal_noise, uint16_t sta_id) {
    (void) packet;
    (void) channel_frequency;
    (void) tx_vector;
    (void) a_mpdu;

    ns3::WifiMacHeader mac_header;
    packet->PeekHeader(mac_header);
    ns3::Mac48Address robot_address = mac_header.GetAddr3();
    uint16_t robot_id;
    try {
        robot_id = getRobotId(robot_address);
    } catch (const std::logic_error& e) {       // not from a robot
        return;
    }

    std::unique_lock<std::mutex> ul(snr_mutex_[robot_id]);
    if (!snr_measuring_[robot_id]) return;      // not measuring SNR, a packet has been received for another purpose
    double signal = dbm_to_watt(signal_noise.signal);
    double noise = dbm_to_watt(signal_noise.noise);
    snr_[robot_id] = signal / noise;
    snr_measuring_[robot_id] = false;

    std::cout << "signal: " << signal << ", " << signal_noise.signal << " dB" << std::endl;
    std::cout << "noise: " << noise << ", " << signal_noise.noise << " dB" << std::endl;
}

double WifiNetwork::measureSnr(int robot_id, const ns3::Time &max_duration) {
    std::unique_lock<std::mutex> ul(snr_mutex_[robot_id]);
    snr_measuring_[robot_id] = true;
    snr_[robot_id] = 0;
    ul.unlock();

    ping(robot_id, max_duration);

    ul.lock();
    double snr = snr_[robot_id];
    return snr;
}

void WifiNetwork::setMecServerPosition(const ns3::Vector &position) {
    setNodePosition(mecServer(), position);
}
