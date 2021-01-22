/*
 * Simulates the following network:
 *
 *                5G NR
 *      *       *       ...     *       *
 *      |       |               |       |      fiber
 *   robot1  robot2          robotN    gNB ------------ MEC server
 *
 * In order to use it, you must first run: sudo python3 scripts/netns.py 5g setup
 */

#pragma once

#include <turtlebot2i_edge/wireless.h>

class Nr5GNetwork : public WirelessNetwork {
    ns3::Ptr<ns3::Node> gnb_;

public:
    explicit Nr5GNetwork(int n_robots);

    void createNetwork() override;
    void setGnbPosition(const ns3::Vector &position);
};
