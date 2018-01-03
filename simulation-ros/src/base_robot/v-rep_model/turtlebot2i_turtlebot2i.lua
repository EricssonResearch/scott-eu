function setVels_cb(msg)
   -- not sure if a scale factor must be applied
   local linVel = msg.linear.x/2 -- in m/s
   local rotVel = msg.angular.z*interWheelDistance/2 -- in rad/s
   
   --  Check if motor is enabled 
   if (motor_power == 1) then
       velocityRight = linVel+rotVel
       velocityLeft  = linVel-rotVel
   else
       velocityRight = 0 
       velocityLeft  = 0 
   end
end

-- Enable/disable motors
function setMotor_cb(msg)
    motor_power = msg.state
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

    motor_power = 1 --Enable motors by default

    t_frontBumper = sim.getObjectHandle('bumper_front_joint')
    t_leftBumper  = sim.getObjectHandle('bumper_left_joint')
    t_rightBumper = sim.getObjectHandle('bumper_right_joint')
    
    f_cliff_handle = sim.getObjectHandle('cliff_sensor_front')
    l_cliff_handle = sim.getObjectHandle('cliff_sensor_left')
    r_cliff_handle = sim.getObjectHandle('cliff_sensor_right')

    l_wheel_drop_handle = sim.getObjectHandle('wheel_drop_sensor_left')
    r_wheel_drop_handle = sim.getObjectHandle('wheel_drop_sensor_right')

    docking_station_ir_handle = sim.getObjectHandle('docking_station_ir_sensor')
    docking_station_handle = sim.getObjectHandle('DockStation')

    -- Odometry variables
    r_linear_velocity, r_angular_velocity = {0,0,0},{0,0,0}
    originMatrix = sim.getObjectMatrix(mainBodyHandle,-1)
    invOriginMatrix = simGetInvertedMatrix(originMatrix)
    oldTransformation = originMatrix
    lastTime = simGetSimulationTime()

    bumper_id = 255
    cliff_sensor = 255
    wheel_drop_sensor = 255

    ----------------------------- ROS STUFF --------------------------------
    -- Bumper
	pubBumper = simROS.advertise(modelBaseName..'/events/bumper', 'kobuki_msgs/BumperEvent')
	--simROS.publisherTreatUInt8ArrayAsString(pubBumper)
    -- Cliff
	pubCliff = simROS.advertise(modelBaseName..'/events/cliff', 'kobuki_msgs/CliffEvent')
    -- Wheel Drop
	pubWheelDrop = simROS.advertise(modelBaseName..'/events/wheel_drop', 'kobuki_msgs/WheelDropEvent')
	--  Docking IR
    pubDockIR = simROS.advertise(modelBaseName..'/sensors/dock_ir', 'kobuki_msgs/DockInfraRed')
	simROS.publisherTreatUInt8ArrayAsString(pubDockIR)
    -- Odometry
    pubPose = simROS.advertise(modelBaseName..'/odom', 'nav_msgs/Odometry')
    simROS.publisherTreatUInt8ArrayAsString(pubPose)

    -- Commands
    subCmdVel = simROS.subscribe(modelBaseName..'/commands/velocity','geometry_msgs/Twist','setVels_cb')
    subCmdMotor = simROS.subscribe(modelBaseName..'/commands/motor_power','kobuki_msgs/MotorPower','setMotor_cb')

end 

if (sim_call_type == sim.childscriptcall_sensing) then 
    -- Bumper
    local bumper_pressed = 0
    -- Cliff
    local cliff_sensor_activated = 0
    -- Wheel Drop
    local wheel_drop_sensor_activated = 0
    -- Docking IR
    local dock_ir_proximity = 255
    local dock_ir_orientation = 255

    ---- BUMPER SENSING ----
    -- Front Bumper
    front_bumper_pos = sim.getJointPosition(t_frontBumper)
    if(front_bumper_pos < -0.001) then
        front_collision=true
        bumperCenterState = 1
        bumper_id = 1
        bumper_pressed = 1
    else
        front_collision=false
        bumperCenterState = 0
    end
    
    -- Right Bumper
    right_bumper_pos = sim.getJointPosition(t_rightBumper)
    if(right_bumper_pos < -0.001) then
        right_collision=true
        bumperRightState = 1
        bumper_id = 2
        bumper_pressed = 1
    else
        right_collision=false
        bumperRightState = 0
    end
    
    -- Left Bumper
    left_bumper_pos = sim.getJointPosition(t_leftBumper)
    if(left_bumper_pos < -0.001) then
        left_collision=true
        bumperLeftState = 1
        bumper_id = 0
        bumper_pressed = 1
    else
        left_collision=false
        bumperLeftState = 0
    end

    -- Bumper ROS message 
    ros_kobuki_bumper_event = {}
    ros_kobuki_bumper_event["bumper"] = bumper_id 
    ros_kobuki_bumper_event["state"] = bumper_pressed
	simROS.publish(pubBumper, ros_kobuki_bumper_event)
    
    ---- CLIFF SENSING ----
    -- Left Cliff
    res, left_cliff_dist = simCheckProximitySensor(l_cliff_handle, sim_handle_all)
    -- Front Cliff
    res, front_cliff_dist = simCheckProximitySensor(f_cliff_handle, sim_handle_all)
    -- Right Cliff
    res, right_cliff_dist = simCheckProximitySensor(r_cliff_handle, sim_handle_all)

    if (left_cliff_dist == nil) then
        cliff_sensor = 0
        cliff_sensor_activated = 1
    end
   
    if (front_cliff_dist == nil) then
        cliff_sensor = 1
        cliff_sensor_activated = 1
    end
    
    if (right_cliff_dist == nil) then
        cliff_sensor = 2
        cliff_sensor_activated = 1
    end

    if (cliff_sensor == nil) then
        cliff_sensor = 255
    end

    local ros_cliff_event = {}
    ros_cliff_event["sensor"] = cliff_sensor
    ros_cliff_event["state"] = cliff_sensor_activated
    ros_cliff_event["bottom"] = left_bumper_pos
	simROS.publish(pubCliff, ros_cliff_event)

    ---- WHEEL DROP ----
    res, left_wheel_drop  = simCheckProximitySensor(l_wheel_drop_handle, sim_handle_all)
    res, right_wheel_drop = simCheckProximitySensor(r_wheel_drop_handle, sim_handle_all)
    if (left_wheel_drop == nil) then
        wheel_drop_sensor = 0
        wheel_drop_sensor_activated = 1
    end
    
    if (right_wheel_drop == nil) then
        wheel_drop_sensor = 1
        wheel_drop_sensor_activated = 1
    end

    if (wheel_drop_sensor == nil) then
        wheel_drop_sensor = 255
    end

    local ros_wheel_drop_event = {}
    ros_wheel_drop_event["wheel"] = wheel_drop_sensor
    ros_wheel_drop_event["state"] = wheel_drop_sensor_activated
    simROS.publish(pubWheelDrop, ros_wheel_drop_event)

    ---- DOCK IR ----
    local max_range = 0.8 -- check a way to get this value from VREP
    local detect_angle = 0
    local detect_proximity = 0
    --local res, detect_dist, detect_point = simCheckProximitySensor(docking_station_ir_handle, docking_station_handle)
    res, detect_dist, detect_point = simCheckProximitySensor(docking_station_ir_handle, sim_handle_all)

    if (detect_dist ~= nil) then
        detect_angle = math.atan2(detect_point[3], detect_point[1]) * 180/3.14
        detect_proximity = detect_dist/max_range

        -- dock is near
        if (detect_proximity < 0.5) then
            dock_ir_proximity = 0
        -- dock is far
        else
            dock_ir_proximity = 1
        end

        -- dock is left
        if (detect_angle < 75) then
            dock_ir_orientation = 1
        -- dock is front
        elseif (detect_angle >= 75 and detect_angle < 105) then
            dock_ir_orientation = 2
        -- dock is right
        else
            dock_ir_orientation = 4
        end
    end

    local ros_dock_ir = {}
    ros_dock_ir["header"] = {seq=0,stamp=simROS.getTime(), frame_id='/dock_ir'}
    ros_dock_ir["data"] = string.char(dock_ir_orientation)

    if (dock_ir_proximity == 1) then
        ros_dock_ir["data"] = string.char(8 * dock_ir_orientation)
    end

    -- print(string.byte(ros_dock_ir["data"]))

    simROS.publish(pubDockIR, ros_dock_ir)     
    
    -- Odometry
    local transformNow = sim.getObjectMatrix(mainBodyHandle,-1)
    local pose_orientationNow = sim.multiplyMatrices(invOriginMatrix, transformNow)
    local r_quaternion = simGetQuaternionFromMatrix(pose_orientationNow)
    local r_position = {pose_orientationNow[4], pose_orientationNow[8], pose_orientationNow[12]}
    local r_linear_velocity, r_angular_velocity = 0,0
    r_linear_velocity, r_angular_velocity = simGetObjectVelocity(mainBodyHandle)

    -- ROSing
    local ros_pose = {}
    ros_pose['header'] = {seq=0,stamp=simROS.getTime(), frame_id="/odom"}
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

    -- Publish TF
    pose_tf = {
        header = {
            stamp = simROS.getTime(),
            frame_id = '/odom'
        },
        child_frame_id = '/base_footprint',
        transform = {
            translation = position_ros,
            rotation = quaternion_ros 
        }
    }

    simROS.sendTransform(pose_tf)
end 

if (sim_call_type==sim.childscriptcall_cleanup) then 
    -- ROS Shutdown
    simROS.shutdownPublisher(pubPose)
    simROS.shutdownPublisher(pubBumper)
    simROS.shutdownPublisher(pubCliff)
    simROS.shutdownPublisher(pubWheelDrop)
    simROS.shutdownPublisher(pubDockIR)
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

