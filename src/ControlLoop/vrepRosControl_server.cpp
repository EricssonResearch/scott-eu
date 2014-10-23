#include "vrepRosControl_server.h"
#include "../v_repLib.h"

#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include "MyRobot_vrepHW.h"

ros::NodeHandle* ROS_server::sm_node = NULL;

// Services:
ros::ServiceServer ROS_server::sm_displayText_server;

// Publishers:
ros::Publisher ROS_server::sm_objectCount_publisher;

// Subscribers:
ros::Subscriber ROS_server::sm_addStatusBarMessage_subscriber;

// Control.
MR::MyRobot_vrepHW * ROS_server::sm_myRobotHw = 0;
controller_manager::ControllerManager * ROS_server::sm_ctrlManager = 0;

ros::CallbackQueue * ROS_server::sm_rosControlCallbackQueue = 0;
ros::AsyncSpinner * ROS_server::sm_spinner = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ROS_server::initialize()
{
	int argc = 0;
	char** argv = NULL;
    ros::init(argc,argv,"vrepRosControl");

	if(!ros::master::check())
		return(false);
	
    sm_node = new ros::NodeHandle;
    assert(sm_node);

    // Control.
    // NOTE: we create a callback queue only handling ros_control callbacks (not the standard callbacks which remain
    //       handled by the global callback queue). The ros_control callback queue is spinned in a thread of its own
    //       (but the standard messages remain handled by ros::spinOnce() in vrep's main thread).
    //       We adopt this architecture because:
    //          * by design ros_control needs to have the control update and the control callbacks handled in separate
    //            threads,
    //          * by design vrep needs to have the standard callbacks handled into the main thread,
    //          * by design vrep's regular API is not thread safe (and thus accessing the API from a secondary thread as
    //            well as from the main thread, may break things).
    sm_rosControlCallbackQueue = new ros::CallbackQueue();
    assert(sm_rosControlCallbackQueue);
    sm_node->setCallbackQueue(sm_rosControlCallbackQueue);

    sm_myRobotHw = new MR::MyRobot_vrepHW();
    assert(sm_myRobotHw);

    sm_ctrlManager = new controller_manager::ControllerManager(sm_myRobotHw, *sm_node);
    assert(sm_ctrlManager);

    sm_spinner = new ros::AsyncSpinner(1, sm_rosControlCallbackQueue); // The associated thread stops spinning when object AsyncSpinner is deleted.
    assert(sm_spinner);
    sm_spinner->start();

	// Enable the services:
    sm_displayText_server = sm_node->advertiseService("displayText",ROS_server::displayText_service);

	// Enable the publishers:
    sm_objectCount_publisher = sm_node->advertise<std_msgs::Int32>("objectCount",1);

	// Enable the subscribers:
    sm_addStatusBarMessage_subscriber = sm_node->subscribe("addStatusbarMessage",1,&ROS_server::addStatusbarMessage_callback);

	return(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::shutDown()
{
    // Control.
    assert(sm_spinner);
    assert(sm_ctrlManager);
    assert(sm_myRobotHw);
    assert(sm_rosControlCallbackQueue);

    delete sm_spinner;
    delete sm_ctrlManager;
    delete sm_myRobotHw;
    delete sm_rosControlCallbackQueue;

	// Disable the subscribers:
    sm_addStatusBarMessage_subscriber.shutdown();

	// Disable the publishers:
    sm_objectCount_publisher.shutdown();

	// Disable the services:
    sm_displayText_server.shutdown();

	// Shut down:
	ros::shutdown();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::instancePass()
{ // When simulation is not running, we "spinOnce" here:
    int simState=simGetSimulationState();
    if ((simState&sim_simulation_advancing)==0)
        spinOnce();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::mainScriptAboutToBeCalled()
{ // When simulation is running, we "spinOnce" here:
    spinOnce();

    assert(sm_myRobotHw);
    assert(sm_ctrlManager);

    // Update ros_control ControllerManager.
    float simulationTime_vrep = simGetSimulationTime();
    static float simulationTime_km1_vrep = simulationTime_vrep;

    bool isSimulationRunning = simGetSimulationState() == sim_simulation_advancing_running;

    if (simulationTime_km1_vrep != simulationTime_vrep && isSimulationRunning)
    {
        ros::Time simulationTime;     simulationTime.fromSec(simulationTime_vrep);
        ros::Time simulationTime_km1; simulationTime_km1.fromSec(simulationTime_km1_vrep);

        sm_myRobotHw->read();
        sm_ctrlManager->update(simulationTime, simulationTime - simulationTime_km1);
        sm_myRobotHw->write();
    }

    simulationTime_km1_vrep = simulationTime_vrep;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::simulationAboutToStart()
{
    assert(sm_myRobotHw);

    sm_myRobotHw->init();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::simulationEnded()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::spinOnce()
{
	// Disable error reporting (it is enabled in the service processing part, but we don't want error reporting for publishers/subscribers)
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

	//Process all requested services and topic subscriptions
    ros::spinOnce();

	//Handle all streaming (publishers)
    streamAllData();

	// Restore previous error report mode:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Services:
bool ROS_server::displayText_service(vrep_skeleton_msg_and_srv::displayText::Request &req,vrep_skeleton_msg_and_srv::displayText::Response &res)
{
	res.dialogHandle=simDisplayDialog("Message from a ROS node",req.textToDisplay.c_str(),sim_dlgstyle_message,NULL,NULL,NULL,NULL);
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Publishers:
void ROS_server::streamAllData()
{
	// Take care of publishers here (i.e. have them publish their data):
	std_msgs::Int32 objCnt;
	int index=0;
	int h=0;
	while (h>=0)
		h=simGetObjects(index++,sim_handle_all);
	objCnt.data=index-1;
    sm_objectCount_publisher.publish(objCnt);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subscribers:
void ROS_server::addStatusbarMessage_callback(const std_msgs::String::ConstPtr& msg)
{
	simAddStatusbarMessage(msg->data.c_str());
}
