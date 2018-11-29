#!/usr/bin/env python
""" 
	This will be the source code for spped limit calculation (ROS node)
	Now the input is manually set, but later will be loaded from V-rep Simulator
"""
import rospy

def speed_calcu():
        
	m_payload=0.7  # manually set now
	m_robot=6.3+m_payload
	m_H=75
	u= 1/(1/m_robot+1/m_H)#two-body-system

	K_upper_leg=50
	Vmax_upper=440*440/sqrt(K_upper_leg*u)


	K_lower_leg=60
	Vmax_lower=260*260/sqrt(K_lower_leg*u)

	speedlimit=min(Vmax_upper,Vmax_lower)
	return speedlimit

if __name__ == '__main__':
    try:
	speedlimit=speed_calcu()
    except rospy.ROSInterruptException:
        rospy.loginfo("Speed calcu error.")
