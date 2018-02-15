//PUT_VREP_ROSPLUGIN_COPYRIGHT_NOTICE_HERE

#ifndef V_REPEXTROS_H
#define V_REPEXTROS_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#define VREP_DLLEXPORT extern "C"

// The 3 required entry points of the V-REP plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

struct ScriptCallback
{
    int scriptId;
    std::string name;
};

struct Proxy
{
    bool destroyAfterSimulationStop;
};

#include <ros_msg_builtin_io.h>

struct SubscriberProxy : Proxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    ScriptCallback topicCallback;
    ros::Subscriber subscriber;
    image_transport::Subscriber imageTransportSubscriber;
    WriteOptions wr_opt;
};

struct PublisherProxy : Proxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    ros::Publisher publisher;
    image_transport::Publisher imageTransportPublisher;
    ReadOptions rd_opt;
};

struct ServiceClientProxy : Proxy
{
    int handle;
    std::string serviceName;
    std::string serviceType;
    ros::ServiceClient client;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

struct ServiceServerProxy : Proxy
{
    int handle;
    std::string serviceName;
    std::string serviceType;
    ScriptCallback serviceCallback;
    ros::ServiceServer server;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

#include <stubs.h>
#include <ros_msg_io.h>
#include <ros_srv_io.h>

#include <v_repLib.h>

#endif
