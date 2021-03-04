/*
 * Simulates the following network:
 *
 *                 WiFi 10.0.0.0/24
 *      *       *       ...     *       *
 *      |       |               |       |
 *   robot1  robot2          robotN  MEC server
 *
 * In order to use it, you must first run: sudo python3 scripts/netns.py wifi setup
 */

#pragma once

#include <turtlebot2i_edge/wireless.h>

class WifiNetwork : public WirelessNetwork {
    std::vector<bool> snr_measuring_;
    std::vector<double> snr_;
    std::vector<std::mutex> snr_mutex_;

    void saveSnr(ns3::Ptr<const ns3::Packet> packet, uint16_t channel_frequency, ns3::WifiTxVector tx_vector,
                 ns3::MpduInfo a_mpdu, ns3::SignalNoiseDbm signal_noise, uint16_t robot_id);

public:
    explicit WifiNetwork(int n_robots);

    void createNetwork() override;
    void createApplications() override;

    double measureSnr(int robot_id, const ns3::Time &max_duration) override;
    void setMecServerPosition(const ns3::Vector &position);
};
