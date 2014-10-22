#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace LBC
{


enum LpcJointsEnum
{
    FRONT_LEFT_WHEEL_JOINT = 0,
    BACK_LEFT_WHEEL_JOINT,
    BACK_RIGHT_WHEEL_JOINT,
    FRONT_RIGHT_WHEEL_JOINT,

    LPC_JOINTS_NUM
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This is the hardware interface for LPC simulated in vrep.
class MyRobot_vrepHW : public hardware_interface::RobotHW
{
public:
    MyRobot_vrepHW();

    bool init();

    bool read();
    bool write();

protected:
    static std::string sm_jointsName[LPC_JOINTS_NUM];

    // Vrep handles.
    int m_vrepJointsHandle[LPC_JOINTS_NUM];

    // Interfaces.
    double m_cmd[LPC_JOINTS_NUM];
    double m_pos[LPC_JOINTS_NUM];
    double m_vel[LPC_JOINTS_NUM];
    double m_eff[LPC_JOINTS_NUM];

    hardware_interface::JointStateInterface m_jointState_interface;
    hardware_interface::VelocityJointInterface m_jointVelocity_interface;

    void registerHardwareInterfaces();
};


} // namespace LBC.
