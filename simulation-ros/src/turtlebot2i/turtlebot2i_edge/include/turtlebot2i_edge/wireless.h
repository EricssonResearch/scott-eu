#pragma once

#include <ros/ros.h>
#include <mutex>
#include <condition_variable>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/buildings-module.h"

class WirelessNetwork {
    ns3::NodeContainer robots_;
    ns3::Ptr<ns3::Node> mec_server_;
    ns3::Ptr<ns3::Building> warehouse_;

    std::vector<bool> sending_;
    std::vector<std::mutex> sending_mutex_;
    std::vector<std::condition_variable> sending_cv_;

    void transferCompleted(const ns3::Ptr<ns3::Node> &robot);

protected:
    static void addMobility(const ns3::NodeContainer &nodes);
    static void addInternetStack(const ns3::NodeContainer &nodes);
    static void setNodePosition(const ns3::Ptr<ns3::Node> &node, const ns3::Vector &position);

    ns3::NodeContainer robots() const;
    ns3::Ptr<ns3::Node> mecServer() const;

public:
    explicit WirelessNetwork(int n_robots);
    virtual ~WirelessNetwork();

    virtual void createNetwork() = 0;
    void createApplications();
    void createWarehouse(const ns3::Box &boundaries, int n_rooms_x, int n_rooms_y);
    void simulate();

    void offload(int robot_id, int n_bytes);     // robot -> MEC server

    int nRobots() const;
    std::pair<int,int> getRobotRoom(int robot_id) const;
    void setRobotPosition(int robot_id, const ns3::Vector &position);
};
