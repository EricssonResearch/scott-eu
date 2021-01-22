#pragma once

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/buildings-module.h"

class WirelessNetwork {
    ns3::NodeContainer robots_;
    ns3::Ptr<ns3::Node> mec_server_;
    ns3::Ptr<ns3::Building> warehouse_;

protected:
    static void addMobility(const ns3::NodeContainer &nodes);
    static void setNodePosition(const ns3::Ptr<ns3::Node> &node, const ns3::Vector &position);

    ns3::NodeContainer robots() const;
    ns3::Ptr<ns3::Node> mecServer() const;

public:
    explicit WirelessNetwork(int n_robots);
    virtual ~WirelessNetwork();

    virtual void createNetwork() = 0;
    void createWarehouse(const ns3::Box &boundaries, int n_rooms_x, int n_rooms_y);
    void simulate();

    int nRobots() const;
    std::pair<int,int> getRobotRoom(int robot_id) const;
    void setRobotPosition(int robot_id, const ns3::Vector &position);
};
