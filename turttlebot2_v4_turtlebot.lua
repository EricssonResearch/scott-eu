function setVels_cb(msg)
   -- not sure if a scale factor must be applied
   local linVel = msg.linear.x/2 -- in m/s
   local rotVel = msg.angular.z*interWheelDistance/2 -- in rad/s
   velocityRight = linVel+rotVel
   velocityLeft = linVel-rotVel
end

-- TODO
function setMotor_cb(msg)
    -- Disable motor
    if msg.state == 0 then
        sim.setJointTargetVelocity(leftJoint, 0)
        sim.setJointTargetVelocity(rightJoint, 0)
    -- Enable motor
    --else

    end
end

if (sim_call_type==sim.childscriptcall_initialization) then 
    objHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    modelBaseName = sim.getObjectName(objHandle)

    mainBodyHandle = sim.getObjectHandle("turtlebot_body_visual")
    
    leftJoint=sim.getObjectHandle("turtlebot_leftWheelJoint_")
    rightJoint=sim.getObjectHandle("turtlebot_rightWheelJoint_")
    simulationIsKinematic=false -- we want a dynamic simulation here!
    velocityLeft = 0
    velocityRight = 0
    linVel = 0
    rotVel = 0

    t_frontBumper = sim.getObjectHandle('bumper_front_joint')
    t_rightBumper = sim.getObjectHandle('bumper_right_joint')
    t_leftBumper  = sim.getObjectHandle('bumper_left_joint')
    
    f_cliff_handle = sim.getObjectHandle('cliff_sensor_front')
    r_cliff_handle = sim.getObjectHandle('cliff_sensor_right')
    l_cliff_handle = sim.getObjectHandle('cliff_sensor_left')

    -- Odometry variables
    r_linear_velocity, r_angular_velocity = {0,0,0},{0,0,0}
    originMatrix = sim.getObjectMatrix(mainBodyHandle,-1)
    invOriginMatrix = simGetInvertedMatrix(originMatrix)
    oldTransformation = originMatrix
    lastTime = simGetSimulationTime()
    ----------------------------- ROS STUFF --------------------------------
    -- Bumper
	pubBumper = simROS.advertise(modelBaseName..'/events/bumper','kobuki_msgs/BumperEvent')
	--simROS.publisherTreatUInt8ArrayAsString(pubBumper)
    -- Cliff
	pubCliff = simROS.advertise(modelBaseName..'/events/cliff','kobuki_msgs/CliffEvent')
    -- Odometry
    pubPose = simROS.advertise(modelBaseName..'/pose','nav_msgs/Odometry')
    simROS.publisherTreatUInt8ArrayAsString(pubPose)

    -- Commands
    subCmdVel = simROS.subscribe(modelBaseName..'/cmd_vel','geometry_msgs/Twist','setVels_cb')
    subCmdMotor = simROS.subscribe(modelBaseName..'/motor_power','kobuki_msgs/MotorPower','setMotor_cb')
end 


if (sim_call_type == sim.childscriptcall_sensing) then 
    -- Bumper
    local bumper_id = 255
    local bumper_pressed = 0
    -- Cliff
    local cliff_sensor = 255
    local cliff_sensor_activated = 0

    --[[ BUMPER SENSING ]]--
    -- Front Bumper
    front_bumper_pos = sim.getJointPosition(t_frontBumper)
    if(front_bumper_pos < -0.001) then
       -- print("F. COLLISION!")
        front_collision=true
        bumperCenterState = 1
        bumper_id = 1
        bumper_pressed = 1
    else
       -- print("F. No Collision")
        front_collision=false
        bumperCenterState = 0
    end
    
    -- Right Bumper
    right_bumper_pos = sim.getJointPosition(t_rightBumper)
    if(right_bumper_pos < -0.001) then
       -- print("R. COLLISION!")
        right_collision=true
        bumperRightState = 1
        bumper_id = 2
        bumper_pressed = 1
    else
        --print("R. No Collision")
        right_collision=false
        bumperRightState = 0
    end
    
    -- Left Bumper
    left_bumper_pos = sim.getJointPosition(t_leftBumper)
    if(left_bumper_pos < -0.001) then
      --  print("L. COLLISION!")
        left_collision=true
        bumperLeftState = 1
        bumper_id = 1
        bumper_pressed = 1
    else
        --print("L. No Collision")
        left_collision=false
        bumperLeftState = 0
    end

    -- Bumper ROS message 
    ros_kobuki_bumper_event = {}
    ros_kobuki_bumper_event["bumper"] = bumper_id 
    ros_kobuki_bumper_event["state"] = bumper_pressed
	simROS.publish(pubBumper, ros_kobuki_bumper_event)
    
    --[[ CLIFF SENSING ]]--
    -- Front Cliff
    res, front_cliff_dist = simCheckProximitySensor(f_cliff_handle, sim_handle_all)
    -- Left Cliff
    res, left_cliff_dist = simCheckProximitySensor(l_cliff_handle, sim_handle_all)
    -- Right Cliff
    res, right_cliff_dist = simCheckProximitySensor(r_cliff_handle, sim_handle_all)
   
    if (front_cliff_dist == nil) then
        cliff_sensor = 1
        cliff_sensor_activated = 1
    end

    if (left_cliff_dist == nil) then
        cliff_sensor = 2
        cliff_sensor_activated = 1
    end
    
    if (right_cliff_dist == nil) then
        cliff_sensor = 3
        cliff_sensor_activated = 1
    end

    local ros_cliff_event = {}
    ros_cliff_event["sensor"] = cliff_sensor
    ros_cliff_event["state"] = cliff_sensor_activated
    ros_cliff_event["bottom"] = left_bumper_pos
	simROS.publish(pubCliff, ros_cliff_event)

    -- Odometry
    local transformNow = sim.getObjectMatrix(mainBodyHandle,-1)
    local pose_orientationNow = sim.multiplyMatrices(invOriginMatrix, transformNow)
    local r_quaternion = simGetQuaternionFromMatrix(pose_orientationNow)
    local r_position = {pose_orientationNow[4], pose_orientationNow[8], pose_orientationNow[12]}
    local r_linear_velocity, r_angular_velocity = 0,0
    r_linear_velocity, r_angular_velocity = simGetObjectVelocity(mainBodyHandle)

    -- ROSing
    local ros_pose = {}
    ros_pose['header'] = {seq=0,stamp=simROS.getTime(), frame_id="/robot"..modelBaseName}
    local cov = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    local quaternion_ros = {}
    quaternion_ros["x"] = r_quaternion[1]
    quaternion_ros["y"] = r_quaternion[2]
    quaternion_ros["z"] = r_quaternion[3]
    quaternion_ros["w"] = r_quaternion[4]
    local position_ros = {}
    position_ros["x"] = r_position[1]
    position_ros["y"] = r_position[2]
    position_ros["z"] = r_position[3]
    local pose_r = {position=position_ros, orientation=quaternion_ros}
    ros_pose['pose'] = {pose=pose_r, covariance = cov}
    local linear_speed = {}
    linear_speed["x"] = r_linear_velocity[1]
    linear_speed["y"] = r_linear_velocity[2]
    linear_speed["z"] = r_linear_velocity[3]
    local angular_speed = {}
    angular_speed["x"] = r_angular_velocity[1]
    angular_speed["y"] = r_angular_velocity[2]
    angular_speed["z"] = r_angular_velocity[3]
    ros_pose['twist'] = {twist={linear=linear_speed, angular=angular_speed}, covariance=cov}
    ros_pose['child_frame_id'] = "kinect"
    simROS.publish(pubPose, ros_pose)     
end 

if (sim_call_type==sim.childscriptcall_cleanup) then 
    -- ROS Shutdown
    simROS.shutdownPublisher(pubPose)
    simROS.shutdownPublisher(pubBumper)
    simROS.shutdownSubscriber(subCmdVel)
end 

if (sim_call_type==sim.childscriptcall_actuation) then 
    s=sim.getObjectSizeFactor(objHandle) -- make sure that if we scale the robot during simulation, other values are scaled too!
    v0=0.4*s
    wheelDiameter=0.085*s
    interWheelDistance=0.137*s
    noDetectionDistance=0.4*s
    if simulationIsKinematic then
        -- Simulation is kinematic
        p=sim.boolOr32(sim.getModelProperty(objHandle),sim.modelproperty_not_dynamic)
        sim.setModelProperty(objHandle,p)
        dt=sim.getSimulationTimeStep()
        p=sim.getJointPosition(leftJoint)
        sim.setJointPosition(leftJoint,p+velocityLeft*dt*2/wheelDiameter)
        p=sim.getJointPosition(rightJoint)
        sim.setJointPosition(rightJoint,p+velocityRight*dt*2/wheelDiameter)
        linMov=dt*(velocityLeft+velocityRight)/2.0
        rotMov=dt*math.atan((velocityRight-velocityLeft)/interWheelDistance)
        position=sim.getObjectPosition(objHandle,sim.handle_parent)
        orientation=sim.getObjectOrientation(objHandle,sim.handle_parent)
        xDir={math.cos(orientation[3]),math.sin(orientation[3]),0.0}
        position[1]=position[1]+xDir[1]*linMov
        position[2]=position[2]+xDir[2]*linMov
        orientation[3]=orientation[3]+rotMov
        sim.setObjectPosition(objHandle,sim.handle_parent,position)
        sim.setObjectOrientation(objHandle,sim.handle_parent,orientation)
    else
        -- Simulation is dynamic
        p=sim.boolOr32(sim.getModelProperty(objHandle),sim.modelproperty_not_dynamic)-sim.modelproperty_not_dynamic
        sim.setModelProperty(objHandle,p)
        --velocityRight = linVel + rotVel
        --velocityLeft = linVel - rotVel
        sim.setJointTargetVelocity(leftJoint,velocityLeft*2/wheelDiameter)
        sim.setJointTargetVelocity(rightJoint,velocityRight*2/wheelDiameter)
    end 
end 

