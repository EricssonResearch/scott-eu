#include <thread>
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <turtlebot2i_edge/wifi.h>
#include <turtlebot2i_edge/Stamp.h>
#include <turtlebot2i_edge/Ping.h>

void updatePosition(const std::shared_ptr<WirelessNetwork> &network, int robot_id,
                    const nav_msgs::Odometry::ConstPtr &msg) {
    const geometry_msgs::Point& position = msg->pose.pose.position;
    network->setRobotPosition(robot_id, {position.x, position.y, position.z});
//    std::pair<int,int> room = network->getRobotRoom(robot_id);
//    ROS_INFO("Robot %d is in room {%d,%d}, position {%f,%f,%f}",
//             robot_id, room.first, room.second, position.x, position.y, position.z);
}

bool transfer(const std::shared_ptr<WirelessNetwork> &network, int robot_id, turtlebot2i_edge::Stamp::Request &request,
              turtlebot2i_edge::Stamp::Response &response, bool upload) {
    int to_transfer = request.to_stamp;
    int transferred;

    if (upload)
        transferred = network->upload(robot_id, to_transfer, ns3::Seconds(request.max_duration.toSec()));
    else
        transferred = network->download(robot_id, to_transfer, ns3::Seconds(request.max_duration.toSec()));

    response.header.stamp = ros::Time::now();
    response.stamped = transferred;

    double percentage = 100 * static_cast<double>(transferred) / to_transfer;
    if (upload)
        ROS_INFO("Robot %d transferred %d bytes out of %d (%f%%) to MEC server",
                 robot_id, transferred, to_transfer, percentage);
    else
        ROS_INFO("MEC server transferred %d bytes out of %d (%f%%) to robot %d",
                 transferred, to_transfer, percentage, robot_id);
    return true;
}

bool ping(const std::shared_ptr<WirelessNetwork> &network, int robot_id, turtlebot2i_edge::Ping::Request &request,
          turtlebot2i_edge::Ping::Response &response) {
    ns3::Time rtt = network->ping(robot_id, ns3::Seconds(request.max_rtt.toSec()));
    response.rtt.fromSec(rtt.GetSeconds());
    ROS_INFO("Robot %d measured RTT: %f ms", robot_id, rtt.GetSeconds() * 1e3);
    return true;
}

void runRosNode(const std::shared_ptr<WirelessNetwork> &network) {
    ros::NodeHandle node_handle;
    int n_robots = network->nRobots();

    ROS_INFO("Updating positions...");
    std::vector<ros::Subscriber> subscribers;
    for (int i=0; i<n_robots; i++) {
        std::string topic = "odom_" + std::to_string(i);
        auto callback = [network, i](const nav_msgs::Odometry::ConstPtr &msg) { updatePosition(network, i, msg); };
        ros::Subscriber subscriber = node_handle.subscribe<nav_msgs::Odometry>(topic, 1, callback);
        subscribers.push_back(subscriber);
    }

    std::vector<ros::ServiceServer> service_servers;
    for (int i=0; i<n_robots; i++) {
        std::string service = "upload_" + std::to_string(i);
        auto callback = [network, i](turtlebot2i_edge::Stamp::Request &request,
                                     turtlebot2i_edge::Stamp::Response &response) {
            return transfer(network, i, request, response, true);
        };
        ros::ServiceServer service_server = node_handle.advertiseService
                <turtlebot2i_edge::Stamp::Request, turtlebot2i_edge::Stamp::Response>(service, callback);
        service_servers.push_back(service_server);
    }
    ROS_INFO("ROS services to upload ready");

    for (int i=0; i<n_robots; i++) {
        std::string service = "download_" + std::to_string(i);
        auto callback = [network, i](turtlebot2i_edge::Stamp::Request &request,
                                     turtlebot2i_edge::Stamp::Response &response) {
            return transfer(network, i, request, response, false);
        };
        ros::ServiceServer service_server = node_handle.advertiseService
                <turtlebot2i_edge::Stamp::Request, turtlebot2i_edge::Stamp::Response>(service, callback);
        service_servers.push_back(service_server);
    }
    ROS_INFO("ROS services to download ready");

    for (int i=0; i<n_robots; i++) {
        std::string service = "ping_" + std::to_string(i);
        auto callback = [network, i](turtlebot2i_edge::Ping::Request &request,
                                     turtlebot2i_edge::Ping::Response &response) {
            return ping(network, i, request, response);
        };
        ros::ServiceServer service_server = node_handle.advertiseService
                <turtlebot2i_edge::Ping::Request, turtlebot2i_edge::Ping::Response>(service, callback);
        service_servers.push_back(service_server);
    }
    ROS_INFO("ROS services to ping ready");

    // 2 threads per robot, one for updating the position and one for using the network
    ros::MultiThreadedSpinner spinner(2 * n_robots);
    spinner.spin();
}

ns3::Vector xmlrpc_to_position(const XmlRpc::XmlRpcValue &position) {
    const XmlRpc::XmlRpcValue &x = position[0];
    const XmlRpc::XmlRpcValue &y = position[1];
    const XmlRpc::XmlRpcValue &z = position[2];
    auto x_ = x.getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(x) : static_cast<int>(x);
    auto y_ = y.getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(y) : static_cast<int>(y);
    auto z_ = z.getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(z) : static_cast<int>(z);
    return {x_, y_, z_};
}

int main(int argc, char **argv) {
    std::string network_type;
    std::vector<double> warehouse_boundaries, bs_position;
    XmlRpc::XmlRpcValue congesting_positions;
    int n_robots, n_rooms_x, n_rooms_y, n_congesting_nodes, congesting_data_rate;
    double congesting_min_switch_time, congesting_max_switch_time;

    ros::init(argc, argv, "network_simulation");

    ROS_INFO("Getting parameters from parameter server...");
    if (!ros::param::get("/network/type", network_type) ||
        !ros::param::get("/network/base_station/position", bs_position) ||
        !ros::param::get("/network/robots/n", n_robots) ||
        !ros::param::get("/network/congestion/n", n_congesting_nodes) ||
        !ros::param::get("/network/congestion/data_rate", congesting_data_rate) ||
        !ros::param::get("/network/congestion/switch_time/min", congesting_min_switch_time) ||
        !ros::param::get("/network/congestion/switch_time/max", congesting_max_switch_time) ||
        !ros::param::get("/network/congestion/position", congesting_positions) ||
        !ros::param::get("/network/warehouse/boundaries", warehouse_boundaries) ||
        !ros::param::get("/network/warehouse/rooms/n_x", n_rooms_x) ||
        !ros::param::get("/network/warehouse/rooms/n_y", n_rooms_y)) {
        ROS_ERROR("ROS parameter server does not contain the necessary parameters");
        return -1;
    }
    ns3::Vector bs_position_(bs_position[0], bs_position[1], bs_position[2]);
    ns3::Box warehouse_boundaries_(warehouse_boundaries[0], warehouse_boundaries[1],
                                   warehouse_boundaries[2], warehouse_boundaries[3],
                                   warehouse_boundaries[4], warehouse_boundaries[5]);

    ROS_INFO("Network type: %s", network_type.c_str());
    ROS_INFO("Position of base station: {%f,%f,%f}", bs_position_.x, bs_position_.y, bs_position_.z);
    ROS_INFO("Number of robots: %d", n_robots);
    ROS_INFO("Grid of rooms in warehouse: %dx%d", n_rooms_y, n_rooms_x);
    ROS_INFO("Boundaries of warehouse: {%f,%f,%f,%f,%f,%f}",
             warehouse_boundaries_.xMin, warehouse_boundaries_.xMax,
             warehouse_boundaries_.yMin, warehouse_boundaries_.yMax,
             warehouse_boundaries_.zMin, warehouse_boundaries_.zMax);
    ROS_INFO("Number of congesting nodes: %d", n_congesting_nodes);
    ROS_INFO("Data rate of congesting nodes: %d bps", congesting_data_rate);
    ROS_INFO("Switch time of congesting nodes: [%f,%f] s", congesting_min_switch_time, congesting_max_switch_time);
    for (int i=0; i<n_congesting_nodes; i++) {
        ns3::Vector cp = xmlrpc_to_position(congesting_positions[i]);
        ROS_INFO("Position of congesting node %d: {%f,%f,%f}", i, cp.x, cp.y, cp.z);
    }

    ROS_INFO("Setting up network...");
    std::shared_ptr<WirelessNetwork> network;
    if (network_type == "wifi") {
        network = std::make_shared<WifiNetwork>(n_robots, n_congesting_nodes);
    } else {
        ROS_ERROR("Invalid network type: %s", network_type.c_str());
        return -1;
    }
    network->createNetwork();
    network->createApplications();                  // important: after creating network
    network->createWarehouse(warehouse_boundaries_, n_rooms_x, n_rooms_y);
    network->setBaseStationPosition(bs_position_);
    network->createCongestion(congesting_data_rate, ns3::Seconds(congesting_min_switch_time),
                              ns3::Seconds(congesting_max_switch_time));
    for (int i=0; i<n_congesting_nodes; i++) {
        ns3::Vector cp = xmlrpc_to_position(congesting_positions[i]);
        network->setCongestingNodePosition(i, cp);
    }

    ROS_INFO("Starting ROS node...");
    std::thread thread(runRosNode, network);        // on a second thread, because...

    ROS_INFO("Simulating network...");
    network->simulate();                            // ...this simulates the network

    thread.join();
    return 0;
}
