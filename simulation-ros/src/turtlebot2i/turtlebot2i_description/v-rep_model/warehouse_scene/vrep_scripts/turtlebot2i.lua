-- Adjust circle size
function setCirlceSize_cb(msg)
    --Since 'setObjectSize' doesn't work, we have to use 'Scale', so we need to record the previous scale.
    --BUG: if the size is not changed (or just slightly changed), the following code will waste calculation power.
    sim.scaleObject(zoneRed_handle,msg.critical_zone_radius/previous_critical_zone_radius,msg.critical_zone_radius/previous_critical_zone_radius,0,0)  
    sim.scaleObject(zoneYellow_handle,msg.warning_zone_radius/previous_warning_zone_radius,msg.warning_zone_radius/previous_warning_zone_radius,0,0)
    sim.scaleObject(zoneGreen_handle,msg.clear_zone_radius/previous_clear_zone_radius,msg.clear_zone_radius/previous_clear_zone_radius,0,0)
    --sim.scaleObject(obj_handle,scale,scale,0,0) 
    printf("New circle size received (seq): %d",msg.header.seq)
    previous_clear_zone_radius = msg.clear_zone_radius
    previous_warning_zone_radius = msg.warning_zone_radius
    previous_critical_zone_radius = msg.critical_zone_radius
end
--- Adjust robot speed
function setVels_scale_cb(msg)
   --velScale = msg.data--.scale  --a number: 0-2
   --print("setVels_scale:")
   rightVelScale = msg.right_vel_scale
   leftVelScale = msg.left_vel_scale
   --print("This should not be triggered.")
   --printf("setVels_scale:%d",msg.data)
end

function setVels_cb(msg)
   -- not sure if a scale factor must be applied
   local linVel = msg.linear.x-- in m/s
   local rotVel = msg.angular.z*interWheelDistance -- in rad/s
   
   --  Check if motor is enabled 
   if (motor_power == 1) then
       velocityRight = (linVel+rotVel)*rightVelScale
       velocityLeft  = (linVel-rotVel)*leftVelScale
       printf("linVel=%2.2f,rotVel=%2.2f | rightVelScale=%2.2f,leftVelScale=%2.2f",linVel,rotVel,rightVelScale,leftVelScale)--print(linVel)
       --if (velScale>1) then
           --print("speed up!")
       --end
       --if (velScale<1) then
           --print("slow down!")
       --end
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


    -- Build file name
    local sceneFullName = sim.getStringParameter(sim.stringparam_scene_name)
    local objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    root_name = sim.getObjectName(objectHandle)
   
    -- Set robot name to enable multiple robots in the scene
    robot_number, robot_name = sim.getNameSuffix(root_name)

    robot_id = robot_name .. ((robot_number >= 0) and "_" .. robot_number or "")
    sim.setStringSignal('robot_id', robot_id)
    sim.setStringSignal('root_name', robot_name)
    sim.setStringSignal('robot_name', robot_name)
    sim.setIntegerSignal('robot_number', robot_number)



    objHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    -- docs.ros.org/indigo/api/kobuki_msgs/html/msg/SensorState.html
    --DISCHARGING      = 0
    --DOCKING_CHARGED  = 2
    --DOCKING_CHARGING = 6
    --ADAPTER_CHARGED  = 18
    --ADAPTER_CHARGING = 22
    sim.setIntegerSignal(robot_name .. '_charger_state', 0)  -- specify whether robot is charging


    -- Get proximity sensor handlers
    object_cliff_sensor_front = sim.getObjectHandle('cliff_sensor_front')
    object_cliff_sensor_left = sim.getObjectHandle('cliff_sensor_left')
    object_cliff_sensor_right = sim.getObjectHandle('cliff_sensor_right')
    object_dock_station_ir_sensor = sim.getObjectHandle('dock_station_ir_sensor')
    object_wheel_drop_sensor_left = sim.getObjectHandle('wheel_drop_sensor_left')
    object_wheel_drop_sensor_right = sim.getObjectHandle('wheel_drop_sensor_right')


    -- Disable proximity sensors (Comment the lines below if want to enable some sensor)
    sim.setExplicitHandling(object_cliff_sensor_front, 1)   
    sim.setExplicitHandling(object_cliff_sensor_left, 1) 
    sim.setExplicitHandling(object_cliff_sensor_right, 1)  
    --sim.setExplicitHandling(object_dock_station_ir_sensor, 1) 
    sim.setExplicitHandling(object_wheel_drop_sensor_left, 1) 
sim.setExplicitHandling(object_wheel_drop_sensor_right, 1) 


    robot_id = sim.getStringSignal('robot_id')
    robot_name = sim.getStringSignal('robot_name')

    sim.setIntegerSignal(robot_name .. '_charger_state', 0)  -- Battery is discharging

    mainBodyHandle = sim.getObjectHandle("turtlebot_reference")

    leftJoint = sim.getObjectHandle("turtlebot_leftWheelJoint_")
    rightJoint = sim.getObjectHandle("turtlebot_rightWheelJoint_")
    simulationIsKinematic = false -- we want a dynamic simulation here!
    velocityLeft = 0
    velocityRight = 0
    linVel = 0
    rotVel = 0
    motor_power = 1 --Enable motors by default
    velScale = 1 -- Scale is 1 by default
    leftVelScale = 1  -- Scale is 1 by default
    rightVelScale = 1 -- Scale is 1 by default
    previous_clear_zone_radius = 1.0
    previous_warning_zone_radius = 1.0
    previous_critical_zone_radius = 1.0


    t_frontBumper = sim.getObjectHandle('bumper_front_joint')
    t_leftBumper  = sim.getObjectHandle('bumper_left_joint')
    t_rightBumper = sim.getObjectHandle('bumper_right_joint')
    
    f_cliff_handle = sim.getObjectHandle('cliff_sensor_front')
    l_cliff_handle = sim.getObjectHandle('cliff_sensor_left')
    r_cliff_handle = sim.getObjectHandle('cliff_sensor_right')

    l_wheel_drop_handle = sim.getObjectHandle('wheel_drop_sensor_left')
    r_wheel_drop_handle = sim.getObjectHandle('wheel_drop_sensor_right')

    dock_station_ir_handle = sim.getObjectHandle('dock_station_ir_sensor')
    dock_station_handle = sim.getObjectHandle('dockstation')

    dock_station_ir_emitter_collection_handle = sim.getCollectionHandle('dock_station_ir_emitters')
    ----------------------------------------------
    -- Adjust circle size
    zoneRed_handle = sim.getObjectHandle('critical_zone')
    zoneYellow_handle = sim.getObjectHandle('warning_zone')
    zoneGreen_handle = sim.getObjectHandle('clear_zone')

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
	pubBumper = simROS.advertise(robot_id..'/events/bumper', 'kobuki_msgs/BumperEvent')
    -- Cliff
	pubCliff = simROS.advertise(robot_id..'/events/cliff', 'kobuki_msgs/CliffEvent')
    -- Wheel Drop
	pubWheelDrop = simROS.advertise(robot_id..'/events/wheel_drop', 'kobuki_msgs/WheelDropEvent')
	--  Docking IR
    pubDockIR = simROS.advertise(robot_id..'/sensors/dock_ir', 'kobuki_msgs/DockInfraRed')
	simROS.publisherTreatUInt8ArrayAsString(pubDockIR)
    -- Odometry
    pubPose = simROS.advertise(robot_id..'/odom', 'nav_msgs/Odometry')
    simROS.publisherTreatUInt8ArrayAsString(pubPose)
    -- Sensor State
    pubState = simROS.advertise(robot_id..'/sensors/core', 'kobuki_msgs/SensorState')
    simROS.publisherTreatUInt8ArrayAsString(pubState)

    -- Commands
    subCmdVel = simROS.subscribe(robot_id..'/commands/velocity','geometry_msgs/Twist','setVels_cb')
    subCmdMotor = simROS.subscribe(robot_id..'/commands/motor_power','kobuki_msgs/MotorPower','setMotor_cb')
    subCmdVelScale = simROS.subscribe(robot_id..'/safety/vel_scale','turtlebot2i_safety/VelocityScale','setVels_scale_cb')
    subCmdCircleSize = simROS.subscribe(robot_id..'/safety/safety_zone','turtlebot2i_safety/SafetyZone','setCirlceSize_cb')  
    --------------------END ------------------------------------
end 

if (sim_call_type == sim.childscriptcall_sensing) then 
    -- Bumper
    local bumper_pressed = 0  -- used in BumperEvent msg
    local bumper_state = 0    -- used in SensorState msg
    -- Cliff
    local cliff_sensor_activated = 0
    local cliff_sensor_state = 0
    -- Wheel Drop
    local wheel_drop_sensor_activated = 0
    local wheel_drop_sensor_state = 0
    -- Docking IR
    local dock_ir_proximity = 255
    local dock_ir_orientation = 255

    ---- BUMPER SENSING ----
    -- Front Bumper
    --front_bumper_pos = sim.getJointPosition(t_frontBumper)
    front_bumper_force = sim.getJointForce(t_frontBumper) -- Works better with force instead of position
    if(front_bumper_force < -1) then
        front_collision=true
        bumperCenterState = 1
        bumper_id = 1
        bumper_pressed = 1
        bumper_state = bumper_state + 2
    else
        front_collision=false
        bumperCenterState = 0
    end
    
    -- Right Bumper
    right_bumper_force = sim.getJointForce(t_rightBumper)
    if(right_bumper_force < -1) then
        right_collision=true
        bumperRightState = 1
        bumper_id = 2
        bumper_pressed = 1
        bumper_state = bumper_state + 1
    else
        right_collision=false
        bumperRightState = 0
    end
    
    -- Left Bumper
    left_bumper_force = sim.getJointForce(t_leftBumper)
    if(left_bumper_force < -1) then
        left_collision=true
        bumperLeftState = 1
        bumper_id = 0
        bumper_pressed = 1
        bumper_state = bumper_state + 4
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
        cliff_sensor_state = cliff_sensor_state + 4
    end
   
    if (front_cliff_dist == nil) then
        cliff_sensor = 1
        cliff_sensor_activated = 1
        cliff_sensor_state = cliff_sensor_state + 2
    end
    
    if (right_cliff_dist == nil) then
        cliff_sensor = 2
        cliff_sensor_activated = 1
        cliff_sensor_state = cliff_sensor_state + 1
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
        wheel_drop_sensor_state = wheel_drop_sensor + 2
    end
    
    if (right_wheel_drop == nil) then
        wheel_drop_sensor = 1
        wheel_drop_sensor_activated = 1
        wheel_drop_sensor_state = wheel_drop_sensor + 1
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

    -- TODO: change to http://www.coppeliarobotics.com/helpFiles/en/regularApi/simCheckProximitySensorEx.htm
    --res, detect_dist, detect_point = simCheckProximitySensor(dock_station_ir_handle, sim_handle_all)
    detection_mode = 12      -- fast dection + max detected angle
    detection_threshold = 10
    detection_max_angle = 0.174  -- 10 degrees
    res, detect_dist, detect_point, detect_obj_handle, detect_surf = simCheckProximitySensorEx(dock_station_ir_handle, dock_station_ir_emitter_collection_handle, detection_mode, detection_threshold, detection_max_angle)

    dock_ir_data = {0, 0, 0}
    
    charger_state = sim.getIntegerSignal(robot_name .. '_charger_state')
    
    if (detect_dist ~= nil) then

        ir_emitter_name = simGetObjectName(detect_obj_handle)
        idx = string.find(ir_emitter_name, '_')
        ir_emitter_pos = string.sub(ir_emitter_name, 1, idx-1)

        --print('emitter:'..ir_emitter_pos)

        if ir_emitter_pos == 'left' then
            ir_emitter_pos_code = 1
        elseif ir_emitter_pos == 'center' then
            ir_emitter_pos_code = 2
        else
            ir_emitter_pos_code = 3
        end

        detect_angle = math.atan2(detect_point[3], detect_point[1]) * 180/3.14
        detect_proximity = detect_dist/max_range

        -- dock is near
        if (detect_proximity < 0.5) then
            -- dock is near left
            if (detect_angle < 10) then
                dock_ir_dist_ori = 1
            -- dock is near center
            elseif (detect_angle >= 85 and detect_angle < 95) then
                dock_ir_dist_ori = 2
            -- dock is near right
            elseif (detect_angle > 170) then
                dock_ir_dist_ori = 4
            end
        -- dock is far
        else
            -- dock is far left
            if (detect_angle < 10) then
                dock_ir_dist_ori = 16
            -- dock is far front
            elseif (detect_angle >= 85 and detect_angle < 95) then
                dock_ir_dist_ori = 8
            -- dock is far right
            elseif (detect_angle > 170) then
                dock_ir_dist_ori = 32
            end
        end

        -- Set state to "Charging" if the robot is very close and in front of the docking station
        if (detect_proximity < 0.10 and ir_emitter_pos_code == 2 and dock_ir_dist_ori == 2 and charger_state == 0) then
            charger_state = 6  -- DOCKING_CHARGING 
        end

        dock_ir_data[ir_emitter_pos_code] = dock_ir_dist_ori
    end

    sim.setIntegerSignal(robot_name .. '_charger_state', charger_state)

    local ros_dock_ir = {}
        ros_dock_ir["header"] = {seq = 0,stamp = simROS.getTime(), frame_id = robot_id..'/dock_ir'}
    --ros_dock_ir["data"] = string.char(dock_ir_dist_ori)
    ros_dock_ir["data"] = simPackUInt8Table(dock_ir_data)
    --print(dock_ir_data)

    --if (dock_ir_proximity == 1) then
    --    ros_dock_ir["data"] = string.char(8 * dock_ir_dist_ori)
    --end

    simROS.publish(pubDockIR, ros_dock_ir)     
    
    -- Odometry
    local transformNow = sim.getObjectMatrix(mainBodyHandle,-1)
    local pose_orientationNow = sim.multiplyMatrices(invOriginMatrix, transformNow)
    --local r_quaternion = simGetQuaternionFromMatrix(pose_orientationNow)
    local r_quaternion = simGetObjectQuaternion(mainBodyHandle, -1)        -- uses absolute orientation
    --local r_position = {pose_orientationNow[4], pose_orientationNow[8], pose_orientationNow[12]}
    local r_position = simGetObjectPosition(mainBodyHandle, -1)                                    -- uses absolute pose
    local r_linear_velocity, r_angular_velocity = 0,0
    r_linear_velocity, r_angular_velocity = simGetObjectVelocity(mainBodyHandle)

    -- Static pose Map --> Odom
    local r_map_odom_quaternion = simGetQuaternionFromMatrix(originMatrix)
    local r_map_odom_position = {originMatrix[4], originMatrix[8], originMatrix[12]}

    -- ROSing
    local ros_pose = {}
    ros_pose['header'] = {seq = 0,stamp=simROS.getTime(), frame_id = robot_id..'/odom'}
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
    
    local quaternion_map_odom_ros = {}
    quaternion_map_odom_ros["x"] = r_map_odom_quaternion[1]
    quaternion_map_odom_ros["y"] = r_map_odom_quaternion[2]
    quaternion_map_odom_ros["z"] = r_map_odom_quaternion[3]
    quaternion_map_odom_ros["w"] = r_map_odom_quaternion[4]
    
    local position_map_odom_ros = {}
    position_map_odom_ros["x"] = r_map_odom_position[1]
    position_map_odom_ros["y"] = r_map_odom_position[2]
    position_map_odom_ros["z"] = r_map_odom_position[3]
    
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
    ros_pose['twist'] = {twist = {linear = linear_speed, angular = angular_speed}, covariance = cov}
    ros_pose['child_frame_id'] = robot_id..'/base_footprint'
    simROS.publish(pubPose, ros_pose)     

    time = simROS.getTime()

    -- Publish TF
    pose_tf = {
        header = {
            stamp = time,
            frame_id = robot_id..'/odom'
        },
        child_frame_id = robot_id..'/base_footprint',
        transform = {
            translation = position_ros,
            rotation = quaternion_ros 
        }
    }

    -- Publish Static TF (/map --> /odom)
--    static_tf = {
--        header = {
--            stamp = time,
--            frame_id = '/map'
--        },
--        child_frame_id = robot_id..'/odom',
--        transform = {
--            translation = position_map_odom_ros,
--            rotation = quaternion_map_odom_ros
--        }
--    }

    simROS.sendTransform(pose_tf)
--    simROS.sendTransform(static_tf)


    -- Sensor State

    ros_sensor_state = {
        header = {
            stamp = time,
            frame_id = robot_id..'/base_link'
        },
        
        time_stamp = time,
        bumper = bumper_state,
        wheel_drop = wheel_drop_sensor_state,
        cliff = cliff_sensor_state,
        left_encoder = 0,
        right_encoder = 0,
        left_pwm = 0,
        right_pwm = 0,
        buttons = 0,
        charger = charger_state,
        battery = 0,
        bottom = {0,0,0},
        current = simPackUInt8Table({0,0}),
        over_current = 0,
        digital_input = 0,
        analog_input = {0,0,0,0}
    }

    simROS.publish(pubState, ros_sensor_state)

end 

if (sim_call_type == sim.childscriptcall_cleanup) then 
    -- ROS Shutdown
    simROS.shutdownPublisher(pubPose)
    simROS.shutdownPublisher(pubBumper)
    simROS.shutdownPublisher(pubCliff)
    simROS.shutdownPublisher(pubWheelDrop)
    simROS.shutdownPublisher(pubDockIR)
    simROS.shutdownSubscriber(subCmdVel)
end 

if (sim_call_type == sim.childscriptcall_actuation) then 
    s = sim.getObjectSizeFactor(objHandle) -- make sure that if we scale the robot during simulation, other values are scaled too!
    v0 = 0.4*s
    wheelDiameter = 0.085*s
    interWheelDistance = 0.137*s
    noDetectionDistance = 0.4*s

    if simulationIsKinematic then
        -- Simulation is kinematic
        p = sim.boolOr32(sim.getModelProperty(objHandle),sim.modelproperty_not_dynamic)
        sim.setModelProperty(objHandle,p)
        dt = sim.getSimulationTimeStep()
 
        p = sim.getJointPosition(leftJoint)
        sim.setJointPosition(leftJoint,p+velocityLeft*dt*2/wheelDiameter)
        
        p = sim.getJointPosition(rightJoint)
        sim.setJointPosition(rightJoint,p+velocityRight*dt*2/wheelDiameter)
        
        linMov = dt*(velocityLeft+velocityRight)/2.0
        rotMov = dt*math.atan((velocityRight-velocityLeft)/interWheelDistance)
        
        position = sim.getObjectPosition(objHandle, sim.handle_parent)
        orientation = sim.getObjectOrientation(objHandle, sim.handle_parent)
        
        xDir = {math.cos(orientation[3]), math.sin(orientation[3]), 0.0}
        
        position[1] = position[1] +xDir[1]*linMov
        position[2] = position[2] + xDir[2]*linMov
        orientation[3] = orientation[3] + rotMov
        
        sim.setObjectPosition(objHandle, sim.handle_parent, position)
        sim.setObjectOrientation(objHandle, sim.handle_parent, orientation)
    else
        -- Simulation is dynamic
        p = sim.boolOr32(sim.getModelProperty(objHandle), sim.modelproperty_not_dynamic) - sim.modelproperty_not_dynamic
        sim.setModelProperty(objHandle, p)
        --velocityRight = linVel + rotVel
        --velocityLeft = linVel - rotVel
        sim.setJointTargetVelocity(leftJoint, velocityLeft*2/wheelDiameter)
        sim.setJointTargetVelocity(rightJoint, velocityRight*2/wheelDiameter)
    end
end 
