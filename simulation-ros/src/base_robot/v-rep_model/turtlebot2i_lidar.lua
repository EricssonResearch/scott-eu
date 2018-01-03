-- This is a ROS enabled Hokuyo_04LX_UG01 model (although it can be used as a generic 
-- ROS enabled laser scanner), based on the existing Hokuyo model. It performs instantaneous
-- scans and publishes ROS Laserscan msgs, along with the sensor's tf.

if (sim_call_type == sim.childscriptcall_initialization) then

    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objName=sim.getObjectName(modelHandle)

    modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    parentHandle = simGetObjectParent(modelHandle)

    sensorName = sim.getObjectName(modelHandle)
    sensorName = string.gsub(sensorName,"#","")

    if parentHandle ~= -1 then
        modelBaseName = sim.getObjectName(parentHandle).."/"..sensorName
     else
        modelBaseName = sensorName
    end

    laserHandle = sim.getObjectHandle("lidar_sensor")
    jointHandle = sim.getObjectHandle("lidar_joint")

    scanRange = 180*math.pi/180 --You can change the scan range. Angle_min=-scanRange/2, Angle_max=scanRange/2-stepSize
    stepSize = 2*math.pi/1024
    pts = math.floor(scanRange/stepSize)

    ----------------------------- ROS STUFF --------------------------------

    pubScan = simROS.advertise(modelBaseName..'/scan', 'sensor_msgs/LaserScan')
    simROS.publisherTreatUInt8ArrayAsString(pubScan)

    --parentTf=sim.getObjectHandle("parentTf#")  --get handle to parent object in tf tree. Change this to your needs
    --tfname=simExtROS_enablePublisher('tf',1,simros_strmcmd_get_transform ,modelHandle,parentTf,'') --publish the tf
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
    ros_laser_data["header"] = {seq=0,stamp=simROS.getTime(), frame_id="/scan"}
    ros_laser_data["angle_min"] = -scanRange*0.5
    ros_laser_data["angle_max"] = scanRange*0.5-stepSize
    ros_laser_data["angle_increment"] = stepSize
    --ros_laser_data["time_increment"] = 
    --ros_laser_data["scan_time"] = 
    ros_laser_data["range_min"] = 0 
    ros_laser_data["range_max"] = 50

    ros_laser_data["ranges"] = dists

    simROS.publish(pubScan, ros_laser_data) 

end 

