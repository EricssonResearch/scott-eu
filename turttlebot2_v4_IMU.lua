

if (sim_call_type == sim.childscriptcall_initialization) then 

--    objHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
--    modelBaseName = sim.getObjectName(objHandle)

    -- Initialize Gyro sensor
    ref = sim.getObjectHandle('GyroSensor_reference')
    oldTransformationMatrix = sim.getObjectMatrix(ref, -1)

    -- Initialize Accelerometer
    massObject = sim.getObjectHandle('Accelerometer_mass')
    sensor = sim.getObjectHandle('Accelerometer_forceSensor')
    result,mass = sim.getObjectFloatParameter(massObject, sim.shapefloatparam_mass)

    lastTime = sim.getSimulationTime()
	
    -- ROS publisher 
--    pubIMU = simROS.advertise(modelBaseName..'/sensors/imu', 'sensor_msgs/Imu')
    pubIMU = simROS.advertise('/vrep_ros_interface/turtlebot/sensors/imu', 'sensor_msgs/Imu')
end 

if (sim_call_type == sim.childscriptcall_sensing) then

    -- Get gyro sensor data
    local transformationMatrix = sim.getObjectMatrix(ref, -1)
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
    result,force = sim.readForceSensor(sensor)

    if (result > 0) then
        accel = {force[1]/mass, force[2]/mass, force[3]/mass}  -- accel[1] -> X, accel[1] -> Y, accel[1] -> Z
    end

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

    -- Publlish ROS message
    simROS.publish(pubIMU, imu_msg)     

end 

if (sim_call_type == sim.childscriptcall_cleanup) then 
 
    simROS.shutdownPublisher(pubIMU)

end 

