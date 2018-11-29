#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace MR
{


enum MrJointsEnum
{
    PHANTOMXPINCHER_JOINT1 = 0,
    PHANTOMXPINCHER_JOINT2,
    PHANTOMXPINCHER_JOINT3,
    PHANTOMXPINCHER_JOINT4,

    MR_JOINTS_NUM
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This is the hardware interface for MyRobot simulated in vrep.
class MyRobot_vrepHW : public hardware_interface::RobotHW
{
public:
    MyRobot_vrepHW();

    bool init();

    bool read();
    bool write();

protected:
    static std::string sm_jointsName[MR_JOINTS_NUM];

    // Vrep handles.
    int m_vrepJointsHandle[MR_JOINTS_NUM];

    // Interfaces.
    double m_pos_cmd[MR_JOINTS_NUM];
    double m_vel_cmd[MR_JOINTS_NUM];    
    double m_pos[MR_JOINTS_NUM];
    double m_vel[MR_JOINTS_NUM];
    double m_eff[MR_JOINTS_NUM];

    hardware_interface::JointStateInterface m_jointState_interface;
    hardware_interface::VelocityJointInterface m_jointVelocity_interface;
    hardware_interface::PositionJointInterface m_jointPosition_interface;

    void registerHardwareInterfaces();
};


} // namespace MR.
