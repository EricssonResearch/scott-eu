#include <thread>
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <turtlebot2i_edge/5g.h>
#include <turtlebot2i_edge/wifi.h>

void updatePosition(const std::shared_ptr<WirelessNetwork> &network, int robot_id, const nav_msgs::Odometry::ConstPtr &msg) {
    const geometry_msgs::Point& position = msg->pose.pose.position;
    network->setRobotPosition(robot_id, {position.x, position.y, position.z});
    std::pair<int,int> room = network->getRobotRoom(robot_id);
    ROS_INFO("Robot %d is in room {%d,%d}, position {%f,%f,%f}", robot_id, room.second, room.first, position.x,
             position.y, position.z);
}

void updatePositions(const std::shared_ptr<WirelessNetwork> &network) {
    ros::NodeHandle node_handle;
    std::vector<ros::Subscriber> subscribers;
    for (int i=0; i< network->nRobots(); i++) {
        std::string topic = "odom_" + std::to_string(i);
        auto callback = [network, i](const nav_msgs::Odometry::ConstPtr &msg) { updatePosition(network, i, msg); };
        ros::Subscriber subscriber = node_handle.subscribe<nav_msgs::Odometry>(topic, 1, callback);
        subscribers.push_back(subscriber);
    }
    ros::spin();
}

int main(int argc, char **argv) {
    std::string network_type;
    ns3::Vector position;
    ns3::Box warehouse_boundaries;
    int n_robots, n_rooms_x, n_rooms_y;

    ros::init(argc, argv, "ns3_5g");

    ROS_INFO("Getting parameters from parameter server...");
    if (!ros::param::get("/network/type", network_type) ||
        !ros::param::get("/network/robots/n", n_robots) ||
        !ros::param::get("/warehouse/x_min", warehouse_boundaries.xMin) ||
        !ros::param::get("/warehouse/x_max", warehouse_boundaries.xMax) ||
        !ros::param::get("/warehouse/y_min", warehouse_boundaries.yMin) ||
        !ros::param::get("/warehouse/y_max", warehouse_boundaries.yMax) ||
        !ros::param::get("/warehouse/z_min", warehouse_boundaries.zMin) ||
        !ros::param::get("/warehouse/z_max", warehouse_boundaries.zMax) ||
        !ros::param::get("/warehouse/rooms/n_x", n_rooms_x) ||
        !ros::param::get("/warehouse/rooms/n_y", n_rooms_y)) {
        ROS_ERROR("ROS parameter server does not contain the necessary parameters");
        return -1;
    }
    ROS_INFO("Network type: %s", network_type.c_str());
    ROS_INFO("Number of robots: %d", n_robots);
    ROS_INFO("Boundaries of warehouse: {%f,%f,%f,%f,%f,%f}",
             warehouse_boundaries.xMin, warehouse_boundaries.xMax,
             warehouse_boundaries.yMin, warehouse_boundaries.yMax,
             warehouse_boundaries.zMin, warehouse_boundaries.zMax);
    ROS_INFO("Grid of rooms in warehouse: %dx%d", n_rooms_y, n_rooms_x);

    std::shared_ptr<WirelessNetwork> network;

    if (network_type == "5g") {
        if (!ros::param::get("/network/gnb/position/x", position.x) ||
            !ros::param::get("/network/gnb/position/y", position.y)) {
            ROS_ERROR("ROS parameter server does not contain the necessary parameters");
            return -1;
        }
        ROS_INFO("Position of gNB: {%f,%f}", position.x, position.y);

        std::shared_ptr<Nr5GNetwork> nr5g_network = std::make_shared<Nr5GNetwork>(n_robots);
        nr5g_network->setGnbPosition(position);
        network = nr5g_network;
    } else if (network_type == "wifi") {
        if (!ros::param::get("/network/mec_server/position/x", position.x) ||
            !ros::param::get("/network/mec_server/position/y", position.y)) {
            ROS_ERROR("ROS parameter server does not contain the necessary parameters");
            return -1;
        }
        ROS_INFO("Position of MEC server: {%f,%f}", position.x, position.y);

        std::shared_ptr<WifiNetwork> wifi_network = std::make_shared<WifiNetwork>(n_robots);
        wifi_network->setMecServerPosition(position);
        network = wifi_network;
    } else {
        ROS_ERROR("Invalid network type");
        return -1;
    }

    ROS_INFO("Setting up network...");
    network->createNetwork();
    network->createWarehouse(warehouse_boundaries, n_rooms_x, n_rooms_y);

    ROS_INFO("Starting to update positions...");
    std::thread thread(updatePositions, network);   // on a second thread, because...

    ROS_INFO("Simulating network...");
    network->simulate();                            // ...this simulates the network

    thread.join();
    return 0;
}
