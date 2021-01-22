/*
 * Simulates the following network:
 *
 *                  WiFi
 *      *       *       ...     *       *
 *      |       |               |       |
 *   robot1  robot2          robotN  MEC server
 *
 * In order to use it, you must first run: sudo python3 scripts/netns.py wifi setup
 */

#pragma once

#include <turtlebot2i_edge/wireless.h>

class WifiNetwork : public WirelessNetwork {
public:
    explicit WifiNetwork(int n_robots);

    void createNetwork() override;
    void setMecServerPosition(const ns3::Vector &position);
};
