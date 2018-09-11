#include "vrepRosControl_remoteApi.h"

std::string vrep_ip;
int vrep_port;
std::vector<std::string> joints;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_manager");

  if(!ros::master::check())
    return -1;

  ros::NodeHandle * sm_node = NULL;
  sm_node = new ros::NodeHandle;

  if (!sm_node->getParam("vrep_ip", vrep_ip)){
    ROS_ERROR("Parameter vrep_ip does not exist");
    return -1;
  }
  ROS_INFO("Parameter vrep_ip [%s]", vrep_ip.c_str());

  if (!sm_node->getParam("vrep_port", vrep_port)){
    ROS_ERROR("Parameter vrep_port does not exist");
    return -1;
  }
  ROS_INFO("Parameter vrep_port [%d]", vrep_port);

  if (!sm_node->getParam("joints", joints)){
    ROS_ERROR("Parameter vrep_port does not exist");
    return -1;
  }
  
  for (std::vector<std::string>::iterator it = joints.begin(); it != joints.end(); ++it) {
    ROS_INFO("Parameter joints [%s]", it->c_str());
  }
  

  MR::Phantom_vrepHW * sm_myRobot = NULL;
  sm_myRobot = new MR::Phantom_vrepHW(vrep_ip, vrep_port, &joints);
  sm_myRobot->init();
  
  ros::CallbackQueue * sm_rosControlCallbackQueue = NULL;
  sm_rosControlCallbackQueue = new ros::CallbackQueue();
  sm_node->setCallbackQueue(sm_rosControlCallbackQueue);

  controller_manager::ControllerManager * sm_ctrlManager = NULL;
  sm_ctrlManager = new controller_manager::ControllerManager(sm_myRobot, *sm_node);

  ros::AsyncSpinner * sm_spinner = NULL;
  sm_spinner = new ros::AsyncSpinner(1, sm_rosControlCallbackQueue);
  sm_spinner->start();

  ros::Time time_k1 = ros::Time::now();

  while (ros::ok()) {
    ros::Time time_now = ros::Time::now();
    if (time_now != time_k1) {
      sm_myRobot->read();
      sm_ctrlManager->update(time_now, time_now-time_k1);
      sm_myRobot->write();
    }
    time_k1 = time_now;
    
  }

  delete sm_spinner;
  delete sm_ctrlManager;
  delete sm_myRobot;
  delete sm_rosControlCallbackQueue;
  ROS_INFO("Delete pointers");
  ros::shutdown();
  return 0;
}
