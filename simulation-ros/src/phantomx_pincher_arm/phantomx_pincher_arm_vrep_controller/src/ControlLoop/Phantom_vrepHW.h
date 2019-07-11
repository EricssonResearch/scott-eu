#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <unistd.h>

namespace MR
{


enum MrJointsEnum
{
    PHANTOMXPINCHER_JOINT1 = 0,
    PHANTOMXPINCHER_JOINT2,
    PHANTOMXPINCHER_JOINT3,
    PHANTOMXPINCHER_JOINT4,
    PHANTOMXPINCHER_GRIPPERCENTER_JOINT,
    PHANTOMXPINCHER_GRIPPERCLOSE_JOINT,
    MR_JOINTS_NUM
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This is the hardware interface for MyRobot simulated in vrep.
class Phantom_vrepHW : public hardware_interface::RobotHW
{
public:
  Phantom_vrepHW(std::string aIp, int aPort, std::vector<std::string> * joints);
  
    bool init();

    bool read();
    bool write();

protected:
    std::string sm_jointsNameVrep[MR_JOINTS_NUM];
    static std::string sm_jointsNameUrdf[MR_JOINTS_NUM];

    bool read_blocking();
    
    // Vrep handles.
    int m_vrepJointsHandle[MR_JOINTS_NUM];

    // Interfaces.
    double m_pos_cmd[MR_JOINTS_NUM];
    double m_vel_cmd[MR_JOINTS_NUM];    
    double m_pos[MR_JOINTS_NUM];
    double m_vel[MR_JOINTS_NUM];
    double m_eff[MR_JOINTS_NUM];

    //Vrep Remote Api
    std::string ip;
    int port;
    int vrepClientId;

    hardware_interface::JointStateInterface m_jointState_interface;
    hardware_interface::VelocityJointInterface m_jointVelocity_interface;
    hardware_interface::PositionJointInterface m_jointPosition_interface;

    void registerHardwareInterfaces();
    bool init_vrep();
};


} // namespace MR.
