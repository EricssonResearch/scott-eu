/*
 * Simulates the following network:
 *
 *             WiFi 10.0.0.0/24
 *      *       *       ...     *       *
 *      |       |               |       |      Point-to-point 10.0.1.0/30
 *   robot1  robot2          robotN     AP ------------------------------- MEC server
 */

#pragma once

#include <turtlebot2i_edge/wireless.h>

class WifiNetwork : public WirelessNetwork {
public:
    WifiNetwork(int n_robots, int n_congesting_nodes);
    void createNetwork() override;
};
