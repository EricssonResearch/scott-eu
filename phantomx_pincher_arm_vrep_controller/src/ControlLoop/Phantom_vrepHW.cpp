#include "Phantom_vrepHW.h"

#include "../v_repLib.h"

#include <string>
#include <iostream>

extern "C" {
      #include "../extApi.h"
}

namespace MR
{
  std::string Phantom_vrepHW::sm_jointsNameUrdf[MR_JOINTS_NUM] = {
    "PhantomXPincher_joint1",
    "PhantomXPincher_joint2",
    "PhantomXPincher_joint3",
    "PhantomXPincher_joint4",
    "PhantomXPincher_gripperCenter_joint",
    "PhantomXPincher_gripperClose_joint"
  };


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Phantom_vrepHW::Phantom_vrepHW(std::string aIp, int aPort, std::vector<std::string> * joints) :
    hardware_interface::RobotHW()
{
  port = aPort;
  ip = aIp;
  int i = 0;
  for (std::vector<std::string>::iterator it = joints->begin(); it != joints->end(); ++it) {
    if (i < MR_JOINTS_NUM){
      sm_jointsNameVrep[i] = it->c_str();
      i++ ;
    }
  }
  
    // Init arrays m_cmd[], m_pos[], m_vel[], m_eff[].
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        m_pos_cmd[i] = 0.0;
        m_vel_cmd[i] = 0.0;	
        m_pos[i] = 0.0;
        m_vel[i] = 0.0;
        m_eff[i] = 0.0;
    }

    // Init and get handles of the joints to control.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
        m_vrepJointsHandle[i] = -1;

    // Register joint interfaces.
    registerHardwareInterfaces();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool Phantom_vrepHW::init_vrep(){
    vrepClientId = simxStart(ip.c_str(),port,1,1,2000,5);
    if (vrepClientId == -1) {
      ROS_ERROR("Remote Api not connected at [%s] port [%d]",ip.c_str(),port);
      return false;
    }

    // Get joint handles.
    for (int i = 0; i < MR_JOINTS_NUM; ++i){
      int result = simxGetObjectHandle(vrepClientId,sm_jointsNameVrep[i].c_str(), &m_vrepJointsHandle[i], simx_oneshot_wait);
      if (result != 0) {
	ROS_ERROR_STREAM("MR robot interface not able to get handle for '" << sm_jointsNameVrep[i].c_str() << "'." << std::endl << "Error code: " << result << std::endl);
	return false;
      }        
    }
    read_blocking();
    return true;
    //
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Phantom_vrepHW::init()
{
  if (!init_vrep()){
    return false;
  }
    
  for (int i=0; i < MR_JOINTS_NUM; ++i){
    m_pos_cmd[i] = m_pos[i];
  }
  ROS_ERROR_STREAM("Init" << std::endl);
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Phantom_vrepHW::registerHardwareInterfaces()
{
  ROS_ERROR_STREAM("Join numbers =" << MR_JOINTS_NUM << std::endl);
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        // Joint state interface.
        hardware_interface::JointStateHandle jointStateHandle(sm_jointsNameUrdf[i], &m_pos[i], &m_vel[i], &m_eff[i]);
        m_jointState_interface.registerHandle(jointStateHandle);

        // Joint velocity command interface 
        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &m_vel_cmd[i]);
        m_jointVelocity_interface.registerHandle(jointVelocityHandle);

	//Joint position command interface
	hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &m_pos_cmd[i]);
	m_jointPosition_interface.registerHandle(jointPositionHandle);
	  
    }

    registerInterface(&m_jointState_interface);
    registerInterface(&m_jointVelocity_interface);
    registerInterface(&m_jointPosition_interface);
    ROS_INFO("End of register");
}

 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool Phantom_vrepHW::read_blocking(){
// Really read joint states
    for (int i=0; i < MR_JOINTS_NUM; i++){
      float pos,
	vel,
	eff;
      int r1,
	r2,
	r3;
      r1 = simxGetJointPosition(vrepClientId,m_vrepJointsHandle[i], &pos, simx_opmode_blocking);

      r2 = simxGetObjectFloatParameter(vrepClientId, m_vrepJointsHandle[i], 2012, &vel, simx_opmode_blocking);

      r3 = simxGetJointForce(vrepClientId, m_vrepJointsHandle[i], &eff, simx_opmode_blocking);
	
      if (r1 > 2 || r2 > 2 || r3 > 2) {
	ROS_ERROR("MR robot interface not able to get state for [%s]'. Returned [%d] for position, [%d] for velocity and [%d] for force.",sm_jointsNameVrep[i].c_str(),r1,r2,r3);
	return false;
      }

      m_pos[i] = pos;
      m_vel[i] = vel;
      m_eff[i] = eff;

      // init streamming for further reading
      r1 = simxGetJointPosition(vrepClientId,m_vrepJointsHandle[i], &pos, simx_opmode_streaming);
      r2 = simxGetObjectFloatParameter(vrepClientId, m_vrepJointsHandle[i], 2012, &vel, simx_opmode_streaming);
      r3 = simxGetJointForce(vrepClientId, m_vrepJointsHandle[i], &eff, simx_opmode_streaming);
    }

    return true;

  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Phantom_vrepHW::read()
{
  if (vrepClientId == -1){
    ROS_ERROR("Vrep client error");
    return false;
  }
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
      float pos,
	vel,
	eff;
      int r1,
	r2,
	r3;
      r1 = simxGetJointPosition(vrepClientId,m_vrepJointsHandle[i], &pos, simx_opmode_buffer);

      r2 = simxGetObjectFloatParameter(vrepClientId, m_vrepJointsHandle[i], 2012, &vel, simx_opmode_buffer);

      r3 = simxGetJointForce(vrepClientId, m_vrepJointsHandle[i], &eff, simx_opmode_buffer);
	
      if (r1 > 2 || r2 > 2 || r3 > 2) {
	ROS_ERROR("MR robot interface not able to get state for [%s]'. Returned [%d] for position, [%d] for velocity and [%d] for force.",sm_jointsNameVrep[i].c_str(),r1,r2,r3);
	return false;
      }
    
      m_pos[i] = pos;
      m_vel[i] = vel;
      m_eff[i] = eff;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Phantom_vrepHW::write()
{
  if (vrepClientId == -1){
    ROS_ERROR("Vrep client error");
    return false;
  }
    for (int i = 0; i < MR_JOINTS_NUM; ++i){
      //====== Velocity Control =========
      
      /*if (simSetJointTargetVelocity(m_vrepJointsHandle[i], m_vel_cmd[i]) == -1) {
	ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);
	return false;
      }
      */

      //===== Position Control =======
      if (simxSetJointTargetPosition(vrepClientId, m_vrepJointsHandle[i], m_pos_cmd[i], simx_opmode_oneshot) > 1) {
	ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsNameVrep[i] << "'." << std::endl);
	return false;
      }	
    }
    return true;
}

} // namespace MR.
