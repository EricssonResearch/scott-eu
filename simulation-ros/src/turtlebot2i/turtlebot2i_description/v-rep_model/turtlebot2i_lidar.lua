-- This is a ROS enabled Hokuyo_04LX_UG01 model (although it can be used as a generic 
-- ROS enabled laser scanner), based on the existing Hokuyo model. It performs instantaneous
-- scans and publishes ROS Laserscan msgs, along with the sensor's tf.

if (sim_call_type == sim.childscriptcall_initialization) then

    modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    object_name = sim.getObjectName(modelHandle)
    sensor_number, sensor_name = sim.getNameSuffix(object_name)

    robot_id = sim.getStringSignal("robot_id")

    laserHandle = sim.getObjectHandle("lidar_sensor")
    jointHandle = sim.getObjectHandle("lidar_joint")

    scanRange = 180 * math.pi/180 --You can change the scan range. Angle_min=-scanRange/2, Angle_max=scanRange/2-stepSize
    stepSize = 2 * math.pi/1024
    pts = math.floor(scanRange/stepSize)

    ----------------------------- ROS STUFF --------------------------------

    pubScan = simROS.advertise(robot_id..'/'..sensor_name..'/scan', 'sensor_msgs/LaserScan')
    simROS.publisherTreatUInt8ArrayAsString(pubScan)

end 

if (sim_call_type == sim.childscriptcall_cleanup) then 

end 

if (sim_call_type == sim.childscriptcall_sensing) then 
    local dists = {}
    angle =- scanRange*0.5

    sim.setJointPosition(jointHandle, angle)
    jointPos = angle
    
    for ind = 1, pts, 1 do
    
        r, dist, pt = sim.handleProximitySensor(laserHandle) -- pt is relative to the laser ray! (rotating!)

        if r > 0 then
            dists[ind] = dist
        else
            dists[ind] = 0
        end
    
        ind = ind + 1
        angle = angle + stepSize
        jointPos = jointPos + stepSize
        sim.setJointPosition(jointHandle, jointPos)
    end

    -- Now send the data:
    
    local ros_laser_data = {}
    ros_laser_data["header"] = {seq=0,stamp=simROS.getTime(), frame_id = robot_id..'/'..sensor_name.."/scan"}
    ros_laser_data["angle_min"] = -scanRange*0.5 
    ros_laser_data["angle_max"] = scanRange*0.5-stepSize
    ros_laser_data["angle_increment"] = stepSize
    ros_laser_data["range_min"] = 0 
    ros_laser_data["range_max"] = 50

    ros_laser_data["ranges"] = dists

    simROS.publish(pubScan, ros_laser_data) 

end 

