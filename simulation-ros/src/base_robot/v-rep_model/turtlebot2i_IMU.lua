
-- This function retrieves the stamped transform for a specific object
function getTransformStamped(objHandle, name, relTo, relToName)
    
    t = sim.getSystemTime()
    p = sim.getObjectPosition(objHandle,relTo)
    o = sim.getObjectQuaternion(objHandle,relTo)
    
    return {
        header = {
            stamp = t,
            frame_id = relToName
        },
        child_frame_id = name,
        transform = {
            translation = {x=p[1],y=p[2],z=p[3]},
            rotation = {x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

if (sim_call_type == sim.childscriptcall_initialization) then 

    objHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    modelBaseName = sim.getObjectName(sim.getObjectParent(sim.getObjectParent(objHandle)))

    -- Initialize Gyro sensor
    gyro_object = sim.getObjectHandle('GyroSensor_reference')
    oldTransformationMatrix = sim.getObjectMatrix(gyro_object, -1)

    -- Initialize Accelerometer
    acc_mass_object = sim.getObjectHandle('Accelerometer_mass')
    acc_force_object = sim.getObjectHandle('Accelerometer_forceSensor')
    result,mass = sim.getObjectFloatParameter(acc_mass_object, sim.shapefloatparam_mass)

    -- Initialize GPS
    gps_object = sim.getObjectHandle('GPS_reference')

    xShiftAmplitude = 0
    yShiftAmplitude = 0
    zShiftAmplitude = 0
    xShift = 0
    yShift = 0
    zShift = 0

    lastTime = sim.getSimulationTime()
	
    -- ROS publisher 
    pubIMU = simROS.advertise(modelBaseName..'/sensors/imu', 'sensor_msgs/Imu')
    pubGlobalPose = simROS.advertise(modelBaseName..'/sensors/global_pose', 'geometry_msgs/PoseStamped')
--    pubIMU = simROS.advertise('/vrep_ros_interface/turtlebot/sensors/imu', 'sensor_msgs/Imu')
end 

if (sim_call_type == sim.childscriptcall_sensing) then

    -- Get gyro sensor data
    local transformationMatrix = sim.getObjectMatrix(gyro_object, -1)
    local orientation = sim.getQuaternionFromMatrix(transformationMatrix)
    local oldInverse = simGetInvertedMatrix(oldTransformationMatrix)
    local m = sim.multiplyMatrices(oldInverse, transformationMatrix)
    local euler = sim.getEulerAnglesFromMatrix(m)
    local currentTime = sim.getSimulationTime()
    local gyroData = {0, 0, 0}
    local dt = currentTime-lastTime

    if (dt ~= 0) then
        gyroData[1] = euler[1]/dt -- X
        gyroData[2] = euler[2]/dt -- Y
        gyroData[3] = euler[3]/dt -- Z
    end

    oldTransformationMatrix = sim.copyMatrix(transformationMatrix)
    lastTime = currentTime

    -- Get accelerometer data
    result,force = sim.readForceSensor(acc_force_object)

    if (result > 0) then
        accel = {force[1]/mass, force[2]/mass, force[3]/mass}  -- accel[1] -> X, accel[1] -> Y, accel[1] -> Z
    end

    -- Get GPS data
    
    objectAbsolutePosition = sim.getObjectPosition(gps_object, -1)

    global_pose = {objectAbsolutePosition[1], objectAbsolutePosition[2], objectAbsolutePosition[3]}


    -- Create ROS message
    local imu_msg = {}

    imu_msg['header'] = {seq = 0,
                         stamp = simROS.getTime(), 
                         frame_id = "/imu"}

    imu_msg['orientation'] = { x = orientation[1], 
                               y = orientation[2], 
                               z = orientation[3], 
                               w = orientation[4] } 

    imu_msg['angular_velocity'] = { x = gyroData[1], 
                                    y = gyroData[2], 
                                    z = gyroData[3] } 

    imu_msg['linear_acceleration'] = { x = accel[1], 
                                       y = accel[2], 
                                       z = accel[3] }

    imu_msg['angular_velocity_covariance'] = {}
    imu_msg['linear_acceleration_covariance'] = {}

    local pose_msg = {}

    pose_msg['header'] = {seq = 0,
                         stamp = simROS.getTime(), 
                         frame_id = "/odom"}
    local cov = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    local position_ros = {}
    position_ros['x'] = global_pose[1]
    position_ros['y'] = global_pose[2]
    position_ros['z'] = global_pose[3]

    local quaternion_ros = {}
    quaternion_ros['x'] = orientation[1]
    quaternion_ros['y'] = orientation[2]
    quaternion_ros['z'] = orientation[3]
    quaternion_ros['w'] = orientation[4]

    pose_msg['pose'] = {position=position_ros, orientation=quaternion_ros}

    -- Publish ROS message
    simROS.publish(pubIMU, imu_msg)    
    simROS.publish(pubGlobalPose, pose_msg) 
    
    -- Publish TF
--    pose_tf = {
--        header = {
--            stamp = sim.getSystemTime(),
--            frame_id = '/odom'
--        },
--        child_frame_id = '/base_footprint',
--        transform = {
--            translation = position_ros,
--            rotation = quaternion_ros 
--        }
--    }
--
--    simROS.sendTransform(pose_tf)

end 

if (sim_call_type == sim.childscriptcall_cleanup) then 
 
    simROS.shutdownPublisher(pubIMU)

end 

