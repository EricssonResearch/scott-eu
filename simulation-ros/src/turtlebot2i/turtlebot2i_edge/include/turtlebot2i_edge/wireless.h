#pragma once

#include <mutex>
#include <condition_variable>
#include <ns3/frame-exchange-manager.h>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/buildings-module.h"
#include "ns3/applications-module.h"

class WirelessNetwork {
    ns3::NodeContainer robots_;
    ns3::Ptr<ns3::Node> mec_server_;
    ns3::Ptr<ns3::Node> base_station_;
    ns3::NodeContainer congesting_nodes_;
    ns3::Ptr<ns3::Building> warehouse_;

    std::vector<bool> uploading_;
    std::vector<int> to_upload_;                    // total number of bytes to upload
    std::vector<int> uploaded_;                     // number of bytes uploaded so far
    std::vector<ns3::Time> uploading_start_;        // starting time
    std::vector<std::mutex> uploading_mutex_;
    std::vector<std::condition_variable> uploading_cv_;

    std::vector<bool> downloading_;
    std::vector<int> to_download_;
    std::vector<int> downloaded_;
    std::vector<ns3::Time> downloading_start_;
    std::vector<std::mutex> downloading_mutex_;
    std::vector<std::condition_variable> downloading_cv_;

    std::vector<bool> pinging_;
    std::vector<ns3::Time> rtt_;
    std::vector<std::mutex> pinging_mutex_;
    std::vector<std::condition_variable> pinging_cv_;

    ns3::Time next_tcp_start_;                      // do not establish all TCP connections at once, they fail in ns-3

    void updateUploadedBytes(ns3::Ptr<const ns3::Packet> packet, const ns3::Address &robot_address,
                             const ns3::Address &server_address, const ns3::SeqTsSizeHeader &header);
    void updateDownloadedBytes(ns3::Ptr<const ns3::Packet> packet, const ns3::Address &server_address,
                               const ns3::Address &robot_address, const ns3::SeqTsSizeHeader &header);
    void saveRtt(const ns3::Ptr<ns3::Node> &robot, const ns3::Time &time);

protected:
    int getRobotId(const ns3::Address &address);
    static void addMobility(const ns3::NodeContainer &nodes);
    static void addInternetStack(const ns3::NodeContainer &nodes);
    static void setNodePosition(const ns3::Ptr<ns3::Node> &node, const ns3::Vector &position);

    ns3::NodeContainer robots() const;
    ns3::Ptr<ns3::Node> mecServer() const;
    ns3::Ptr<ns3::Node> baseStation() const;
    ns3::NodeContainer congestingNodes() const;

public:
    WirelessNetwork(int n_robots, int n_congesting_nodes);
    virtual ~WirelessNetwork();

    virtual void createNetwork() = 0;
    virtual void createApplications();
    void createWarehouse(const ns3::Box &boundaries, int n_rooms_x, int n_rooms_y);
    void createCongestion(const std::pair<ns3::Time,ns3::Time> &on_time_range,
                          const std::pair<ns3::Time,ns3::Time> &off_time_range,
                          int data_rate);

    void simulate();
    int upload(int robot_id, int n_bytes, const ns3::Time &max_duration);
    int download(int robot_id, int n_bytes, const ns3::Time &max_duration);
    ns3::Time ping(int robot_id, const ns3::Time &max_rtt);

    int nRobots() const;
    std::pair<int,int> getRobotRoom(int robot_id) const;
    void setRobotPosition(int robot_id, const ns3::Vector &position);
    void setBaseStationPosition(const ns3::Vector &position);
    void setCongestingNodePosition(int congesting_node_id, const ns3::Vector &position);
};
