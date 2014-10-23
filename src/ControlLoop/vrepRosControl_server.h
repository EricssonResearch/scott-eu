#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// API services:
#include "vrep_skeleton_msg_and_srv/displayText.h"

namespace MR
{
class MyRobot_vrepHW;
}

namespace controller_manager
{
class ControllerManager;
}

namespace ros
{
class CallbackQueue;
}

class ROS_server
{
public:
    static bool initialize();
    static void shutDown();

    static void instancePass();
    static void mainScriptAboutToBeCalled();

    static void simulationAboutToStart();
    static void simulationEnded();

private:
    ROS_server() {} // Make this class non-instantiable.

    static ros::NodeHandle* sm_node;

    static void spinOnce();

    // Services:
    static bool displayText_service(vrep_skeleton_msg_and_srv::displayText::Request &req,vrep_skeleton_msg_and_srv::displayText::Response &res);
    static ros::ServiceServer sm_displayText_server;

    // Publishers:
    static void streamAllData();
    static ros::Publisher sm_objectCount_publisher;

    // Subscribers:
    static void addStatusbarMessage_callback(const std_msgs::String::ConstPtr& msg);
    static ros::Subscriber sm_addStatusBarMessage_subscriber;

    // Control.
    static ros::CallbackQueue * sm_rosControlCallbackQueue;

    static MR::MyRobot_vrepHW * sm_myRobotHw;
    static controller_manager::ControllerManager * sm_ctrlManager;

    static ros::AsyncSpinner * sm_spinner;
};
