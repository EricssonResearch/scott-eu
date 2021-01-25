#pragma once

#include <ros/ros.h>
#include <mutex>
#include <condition_variable>
#include <turtlebot2i_edge/Ping.h>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/buildings-module.h"
#include "ns3/applications-module.h"

class WirelessNetwork {
    ns3::NodeContainer robots_;
    ns3::Ptr<ns3::Node> mec_server_;
    ns3::Ptr<ns3::Building> warehouse_;

    std::vector<bool> stamping_;
    std::vector<int> to_stamp_;                 // number of bytes to offload
    std::vector<int> stamped_;                  // already offloaded
    std::vector<std::mutex> stamping_mutex_;
    std::vector<std::condition_variable> stamping_cv_;

    std::vector<bool> pinging_;
    std::vector<std::mutex> pinging_mutex_;
    std::vector<std::condition_variable> pinging_cv_;

    void packetSinkRx(ns3::Ptr<const ns3::Packet> packet, const ns3::Address &from, const ns3::Address &to,
                      const ns3::SeqTsSizeHeader &header);
    void echoClientTx(ns3::Ptr<const ns3::Packet> packet);
    void echoClientRx(ns3::Ptr<const ns3::Packet> packet);

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

    void offload(int robot_id, int n_bytes);                                // robot -> MEC server
    turtlebot2i_edge::Ping::Response ping(int robot_id, int n_packets);     // robot -> MEC server

    int nRobots() const;
    std::pair<int,int> getRobotRoom(int robot_id) const;
    void setRobotPosition(int robot_id, const ns3::Vector &position);
};
