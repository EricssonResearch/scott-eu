/*
 * Simulates the following network:
 *
 *                5G NR
 *      *       *       ...     *       *
 *      |       |               |       |      fiber
 *   robot1  robot2          robotN    gNB ------------ MEC server
 */

#pragma once

#include <turtlebot2i_edge/wireless.h>

class Nr5GNetwork : public WirelessNetwork {
    ns3::Ptr<ns3::Node> gnb_;

public:
    explicit Nr5GNetwork(int n_robots, int n_congesting_nodes);

    void createNetwork() override;
    void setGnbPosition(const ns3::Vector &position);
    double measureSnr(int robot_id, const ns3::Time &max_duration) override;
};
