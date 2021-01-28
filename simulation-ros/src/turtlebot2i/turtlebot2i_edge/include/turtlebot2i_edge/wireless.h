#pragma once

#include <mutex>
#include <condition_variable>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/buildings-module.h"
#include "ns3/applications-module.h"

class WirelessNetwork {
    ns3::NodeContainer robots_;
    ns3::Ptr<ns3::Node> mec_server_;
    ns3::Ptr<ns3::Building> warehouse_;

    std::vector<bool> stamping_;
    std::vector<int> to_stamp_;                 // total number of bytes to stamp
    std::vector<int> stamped_;                  // number of bytes stamped so far
    std::vector<std::mutex> stamping_mutex_;
    std::vector<std::condition_variable> stamping_cv_;

    std::vector<bool> pinging_;
    std::vector<double> rtt_;
    std::vector<std::mutex> pinging_mutex_;
    std::vector<std::condition_variable> pinging_cv_;

    int getRobotId(const ns3::Address &address);
    void updateStampedBytes(ns3::Ptr<const ns3::Packet> packet, const ns3::Address &robot_address,
                            const ns3::Address &server_address, const ns3::SeqTsSizeHeader &header);
    void saveRtt(const ns3::Ptr<ns3::Node> &robot, const ns3::Time &time);

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

    int stamp(int robot_id, int n_bytes, const ns3::Time &max_duration);    // robot -> MEC server
    double ping(int robot_id, const ns3::Time &max_rtt);                    // robot -> MEC server

    int nRobots() const;
    std::pair<int,int> getRobotRoom(int robot_id) const;
    void setRobotPosition(int robot_id, const ns3::Vector &position);
};
