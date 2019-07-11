-- This is a ROS enabled Hokuyo_04LX_UG01 model (although it can be used as a generic 
-- ROS enabled laser scanner), based on the existing Hokuyo model. It performs instantaneous
-- scans and publishes ROS Laserscan msgs, along with the sensor's tf.
-- This script was modified to support the Hokuyo_04LX_UG01_Fast sensor

if (sim_call_type == sim.childscriptcall_initialization) then
	if(simROS==nil)then
		print('Warning: ' .. sim.getObjectName(sim.getObjectAssociatedWithScript(sim.handle_self)) .. ' cannot find simROS.')
	end
    -- Disable camera sensor (comment the lines below to enable)
    object_fastHokuyo_sensor1 = sim.getObjectHandle('fastHokuyo_sensor1')
    -- sim.setExplicitHandling(object_fastHokuyo_sensor1, 1)

    object_fastHokuyo_sensor2 = sim.getObjectHandle('fastHokuyo_sensor2')
    -- sim.setExplicitHandling(object_fastHokuyo_sensor2, 1)

    modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    object_name = sim.getObjectName(modelHandle)
    sensor_number, sensor_name = sim.getNameSuffix(object_name)

    robot_id = sim.getStringSignal("robot_id")
    
    visionSensor1Handle = sim.getObjectHandle("fastHokuyo_sensor1")
    visionSensor2Handle = sim.getObjectHandle("fastHokuyo_sensor2")
    
    maxScanDistance = sim.getScriptSimulationParameter(sim.handle_self, 'maxScanDistance')
    
    if maxScanDistance > 1000 then 
        maxScanDistance = 1000 
    end
    
    if maxScanDistance < 0.1 then 
        maxScanDistance = 0.1 
    end
    
    maxScanDistance_ = maxScanDistance * 0.9999
    
    scanRange = sim.getScriptSimulationParameter(sim.handle_self, 'scanAngle')

    if scanRange > 240 then 
        scanRange = 240 
    end
    
    if scanRange < 2 then 
        scanRange = 2 
    end

    minScanDistance_ = 0.12

    scanRange = scanRange * math.pi/180

    ----------------------------- ROS STUFF --------------------------------
	if(not(simROS==nil))then
		pubScan = simROS.advertise(robot_id..'/'..sensor_name..'/scan', 'sensor_msgs/LaserScan')
		simROS.publisherTreatUInt8ArrayAsString(pubScan)
	end
end

if (sim_call_type == sim.childscriptcall_cleanup) then 

end 

if (sim_call_type == sim.childscriptcall_sensing) then 
	if(not(simROS==nil))then
		ranges = {}
		
		if notFirstHere then
			r,t1,u1 = sim.readVisionSensor(visionSensor1Handle)
			r,t2,u2 = sim.readVisionSensor(visionSensor2Handle)
		
			if u1 then
				for i = 0, u1[1]-1, 1 do
					w = 2+4*i
					dist = u1[w+4]

					if (dist < maxScanDistance_ and dist > minScanDistance_) then
						table.insert(ranges, dist)
					else
						table.insert(ranges, math.huge)
					end
				end
			end

			if u2 then
				for i = 0, u2[1]-1, 1 do
					w = 2+4*i
					dist = u2[w+4]

					if (dist < maxScanDistance_ and dist > minScanDistance_) then
						table.insert(ranges, dist)
					else
						table.insert(ranges, math.huge)
					end

				end
			end

			-- Now send the data:
		   
			stepSize = scanRange / table.getn(ranges)
			local ros_laser_data = {}
			ros_laser_data["header"] = {seq=0, stamp=simROS.getTime(), frame_id = robot_id..'/'..sensor_name.."/scan"}
			ros_laser_data["angle_min"] = -scanRange * 0.5 
			ros_laser_data["angle_max"] =  scanRange * 0.5 - stepSize
			ros_laser_data["angle_increment"] = stepSize
			ros_laser_data["range_min"] = 0 
			ros_laser_data["range_max"] = maxScanDistance
		
			ros_laser_data["ranges"] = ranges
		
			simROS.publish(pubScan, ros_laser_data) 

		end
		notFirstHere=true
		
		-- measuredData now contains all the points that are closer than the sensor range
		-- For each point there is the x, y and z coordinate (i.e. 3 number for each point)
		-- Coordinates are expressed relative to the sensor frame.
		-- You can access this data from outside via various mechanisms. The best is to first
		-- pack the data, then to send it as a string. For example:
		--
		-- 
		-- data=sim.packFloatTable(measuredData)
		-- sim.setStringSignal("measuredDataAtThisTime",data)
		--
		-- Then in a different location:
		-- data=sim.getStringSignal("measuredDataAtThisTime")
		-- measuredData=sim.unpackFloatTable(data)
		--
		--
		-- Of course you can also send the data via tubes, wireless (sim.tubeOpen, etc., sim.sendData, etc.)
		--
		-- Also, if you send the data via string signals, if you you cannot read the data in each simulation
		-- step, then always append the data to an already existing signal data, e.g.
		--
		-- 
		-- data=sim.packFloatTable(measuredData)
		-- existingData=sim.getStringSignal("measuredDataAtThisTime")
		-- if existingData then
		--     data=existingData..data
		-- end
		-- sim.setStringSignal("measuredDataAtThisTime",data)
	end
end
