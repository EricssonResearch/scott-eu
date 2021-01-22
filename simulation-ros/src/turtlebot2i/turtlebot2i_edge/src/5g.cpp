#include <turtlebot2i_edge/5g.h>
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/tap-bridge-module.h"

Nr5GNetwork::Nr5GNetwork(int n_robots) : WirelessNetwork(n_robots) {
    gnb_ = ns3::CreateObject<ns3::Node>();
//    addMobility(gnb_);
}

// adapted from https://gitlab.com/cttc-lena/nr/-/blob/master/examples/cttc-3gpp-channel-simple-ran.cc
void Nr5GNetwork::createNetwork() {
    // TODO: parameterize with ROS parameter server
    ns3::NodeContainer robots = WirelessNetwork::robots();

    // general settings
    ns3::Ptr<ns3::NrHelper> nr_helper = ns3::CreateObject<ns3::NrHelper>();
    ns3::Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", ns3::TimeValue(ns3::MilliSeconds(0)));
    nr_helper->SetSchedulerAttribute("FixedMcsDl", ns3::BooleanValue(true));
    nr_helper->SetSchedulerAttribute("StartingMcsDl", ns3::UintegerValue(28));
    nr_helper->SetChannelConditionModelAttribute("UpdatePeriod", ns3::TimeValue(ns3::MilliSeconds(0)));
    nr_helper->SetPathlossAttribute("ShadowingEnabled", ns3::BooleanValue(true));

    // EPC (LTE core network, non-standalone 5G)
    ns3::Ptr<ns3::NrPointToPointEpcHelper> epc_helper = ns3::CreateObject<ns3::NrPointToPointEpcHelper>();
    nr_helper->SetEpcHelper(epc_helper);

    // spectrum
    ns3::CcBwpCreator cc_bwp_creator;
    ns3::CcBwpCreator::SimpleOperationBandConf band_conf(28e9, 400e6, 1, ns3::BandwidthPartInfo::UMi_StreetCanyon_LoS);
    ns3::OperationBandInfo band = cc_bwp_creator.CreateOperationBandContiguousCc(band_conf);
    nr_helper->InitializeOperationBand(&band);
    ns3::BandwidthPartInfoPtrVector bandwidth_parts = ns3::CcBwpCreator::GetAllBwps({band});

    // beamforming
    ns3::Ptr<ns3::IdealBeamformingHelper> beamforming_helper = ns3::CreateObject<ns3::IdealBeamformingHelper>();
    beamforming_helper->SetAttribute("IdealBeamformingMethod", ns3::TypeIdValue(ns3::DirectPathBeamforming::GetTypeId()));
    nr_helper->SetIdealBeamformingHelper(beamforming_helper);

    // antennas
    nr_helper->SetUeAntennaAttribute("NumRows", ns3::UintegerValue(2));
    nr_helper->SetUeAntennaAttribute("NumColumns", ns3::UintegerValue(4));
    nr_helper->SetUeAntennaAttribute("IsotropicElements", ns3::BooleanValue(true));
    nr_helper->SetGnbAntennaAttribute("NumRows", ns3::UintegerValue(4));
    nr_helper->SetGnbAntennaAttribute("NumColumns", ns3::UintegerValue(8));
    nr_helper->SetGnbAntennaAttribute("IsotropicElements", ns3::BooleanValue(true));

    // install net devices
    ns3::Ptr<ns3::NetDevice> gnb_net_device = nr_helper->InstallGnbDevice(gnb_, bandwidth_parts).Get(0);
    ns3::NetDeviceContainer robot_net_devices = nr_helper->InstallUeDevice(robots, bandwidth_parts);
    nr_helper->GetGnbPhy(gnb_net_device, 0)->SetAttribute("Numerology", ns3::UintegerValue(0));

    // install TapBridge net devices to communicate with OS's taps
    ns3::TapBridgeHelper tap_bridge_helper;
    tap_bridge_helper.SetAttribute("Mode", ns3::StringValue("UseLocal"));
    for (int i=0; i<robots.GetN(); i++) {
        std::string tap = std::string("nr") + std::to_string(i) + "tap";
        std::cout << tap << std::endl;
        tap_bridge_helper.SetAttribute("DeviceName", ns3::StringValue(tap));
        tap_bridge_helper.Install(robots.Get(i), robot_net_devices.Get(i));
    }

    std::cout << "ciaoooo" << std::endl;

    // finish configuration
    ns3::DynamicCast<ns3::NrGnbNetDevice>(gnb_net_device)->UpdateConfig();
    for (auto it = robot_net_devices.Begin(); it != robot_net_devices.End(); it++) {
        ns3::DynamicCast<ns3::NrUeNetDevice>(*it)->UpdateConfig();  // required
//        nr_helper->AttachToEnb(*it, gnb_net_device);
    }

    std::cout << "ciaoooo2" << std::endl;
    std::cout << "ciaoooo2.1" << std::endl;

    std::cout << robots.GetN() << std::endl;
    std::cout << robots.Get(0) << std::endl;

    addMobility(robots);
    addMobility(gnb_);

//    std::cout << robots.Get(0)->GetObject<ns3::MobilityModel>() << std::endl;
    std::cout << "ciaoooo2.5" << std::endl;
    std::cout << gnb_->GetObject<ns3::MobilityModel>() << std::endl;

    // install internet stack
    ns3::InternetStackHelper internet_helper;
//    internet_helper.Install(gnb_);
    internet_helper.Install(robots);
    ns3::Ipv4AddressHelper ip_address_helper;
    ip_address_helper.SetBase("10.0.0.0", "/24");
    ip_address_helper.Assign(gnb_net_device);       // 10.0.0.1
    ip_address_helper.Assign(robot_net_devices);

    nr_helper->AttachToClosestEnb(robot_net_devices, gnb_net_device);

//    epc_helper->AssignUeIpv4Address(robot_net_devices);

    std::cout << "ciaoooo3" << std::endl;
}

void Nr5GNetwork::setGnbPosition(const ns3::Vector &position) {
    setNodePosition(gnb_, position);
}
