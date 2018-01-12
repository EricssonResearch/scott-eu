#include "Phantom_vrepHW.h"

#include "../v_repLib.h"

#include <string>
#include <iostream>


namespace MR
{


std::string Phantom_vrepHW::sm_jointsName[MR_JOINTS_NUM] = {
  "PhantomXPincher_joint1",
  "PhantomXPincher_joint2",
  "PhantomXPincher_joint3",
  "PhantomXPincher_joint4",
  "PhantomXPincher_gripperCenter_joint",
  "PhantomXPincher_gripperClose_joint",
  "PhantomXPincher_joint1_1",
  "PhantomXPincher_joint2_1",
  "PhantomXPincher_joint3_1",
  "PhantomXPincher_joint4_1",
  "PhantomXPincher_gripperCenter_joint_1",
  "PhantomXPincher_gripperClose_joint_1"
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Phantom_vrepHW::Phantom_vrepHW() :
    hardware_interface::RobotHW()
{
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
bool Phantom_vrepHW::init()
{
    // Get joint handles.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        int vrepJointsHandle = simGetObjectHandle(sm_jointsName[i].c_str());

        if (vrepJointsHandle == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get handle for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }

        m_vrepJointsHandle[i] = vrepJointsHandle;
    }
    read();
    
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
        hardware_interface::JointStateHandle jointStateHandle(sm_jointsName[i], &m_pos[i], &m_vel[i], &m_eff[i]);
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
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Phantom_vrepHW::read()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        float pos,
              vel,
              eff;

        if (simGetJointPosition(m_vrepJointsHandle[i], &pos) == -1 ||
            simGetObjectFloatParameter(m_vrepJointsHandle[i], 2012, &vel) == -1 || // Velocity.
            simGetJointForce(m_vrepJointsHandle[i], &eff) == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);

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
    for (int i = 0; i < MR_JOINTS_NUM; ++i){
      //====== Velocity Control =========
      
      /*if (simSetJointTargetVelocity(m_vrepJointsHandle[i], m_vel_cmd[i]) == -1) {
	ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);
	return false;
      }
      */

      //===== Position Control =======
      if (simSetJointTargetPosition(m_vrepJointsHandle[i], m_pos_cmd[i]) == -1) {
	ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);
	return false;
      }	
    }
    return true;
}

} // namespace MR.
