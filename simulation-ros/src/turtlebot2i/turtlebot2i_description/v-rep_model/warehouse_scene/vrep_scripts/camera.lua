function pointCloud()

    local header = {seq = 0, stamp = simROS.getTime(), frame_id = robot_id..'/'..sensor_name.."_depth_optical_frame"}
    local data = {}
    local height = 160 
    local width  = 120
    local fields = {{ name='x', offset=0, datatype= 7, count=1 },
                    { name='y', offset=4, datatype= 7, count=1 },
                    { name='z', offset=8, datatype= 7, count=1 }}

    r,t1,u1 = simReadVisionSensor(depthCam)

    if u1 then
        for j = 0, u1[2]-1, 1 do
            for i = 0, u1[1]-1, 1 do
                local w = 2+4*(j*u1[1]+i)
                local p = {u1[w+1], u1[w+2], u1[w+3]}
                table.insert(data, p[1])
                table.insert(data, p[2])
                table.insert(data, p[3])
            end
        end
    end

    local point_cloud = {}
    point_cloud["header"] = header
    point_cloud["fields"] = fields
    point_cloud["data"] = simPackFloats(data)
    point_cloud["height"] = height
    point_cloud["width"]  = width
    point_cloud["is_dense"] = true
    point_cloud["row_step"] = width * 12
    point_cloud["point_step"] = 12
    point_cloud["is_bigendian"] = false

    simROS.publish(pubKinectCloud, point_cloud)
end

if (sim_call_type==sim.childscriptcall_initialization) then 
	if(simROS==nil)then
		print('Warning: ' .. sim.getObjectName(sim.getObjectAssociatedWithScript(sim.handle_self)) .. ' cannot find simROS.')
	end
	
	rgb_enabled=true
	depth_enabled=true
	
	camera_handle=sim.getObjectAssociatedWithScript(sim.handle_self)
	camera_name=sim.getObjectName(camera_handle)
	
	
	
    object_camera_rgb = sim.getObjectHandle(camera_name..'_rgb')
    object_camera_depth = sim.getObjectHandle(camera_name..'_depth')
	

	robot_id = sim.getStringSignal("robot_id")
	-- Get object handler
	modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
	object_name = sim.getObjectName(modelHandle)
	sensor_number, sensor_name = sim.getNameSuffix(object_name)
	
	if(depth_enabled)then 
		-- Depth camera handler
		depthCam = sim.getObjectHandle(sensor_name..'_depth')
		depthView = sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
		sim.adjustView(depthView,depthCam,64)
		-- ROS Stuff
		if(not(simROS==nil))then
			pubKinectDepth = simROS.advertise(robot_id..'/'..sensor_name..'/depth/raw_image','sensor_msgs/Image')
			simROS.publisherTreatUInt8ArrayAsString(pubKinectDepth)
			pubKinectCloud = simROS.advertise(robot_id..'/'..sensor_name..'/cloud','sensor_msgs/PointCloud2')
			simROS.publisherTreatUInt8ArrayAsString(pubKinectCloud)			
		end
	else
		result2=sim.setExplicitHandling(object_camera_depth, 1) -- disable camera depth
	end
	
	if(rgb_enabled)then  
		-- Color camera handler
		colorCam = sim.getObjectHandle(sensor_name..'_rgb')
		colorView = sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
		sim.adjustView(colorView,colorCam,64)
		-- ROS Stuff
		if(not(simROS==nil))then
			pubKinectRgb = simROS.advertise(robot_id..'/'..sensor_name..'/rgb/raw_image','sensor_msgs/Image')
			simROS.publisherTreatUInt8ArrayAsString(pubKinectRgb) 	
		end
	else
		result1=sim.setExplicitHandling(object_camera_rgb, 1) -- disable camera rgb
	end
		
		

	
    
end 

if (sim_call_type==sim.childscriptcall_cleanup) then 
	if(not(simROS==nil))then
		simROS.shutdownPublisher(pubKinectRgb)
		simROS.shutdownPublisher(pubKinectDepth)
		simROS.shutdownPublisher(pubKinectCloud)
	end
end 

if (sim_call_type==sim.childscriptcall_sensing) then

	current_time = simROS.getTime()
	if(sim.getExplicitHandling(object_camera_rgb) == 0) then
        local data,w,h = sim.getVisionSensorCharImage(colorCam)

        -- Publish camera RGB image to ROS
	    d = {}
        d['header'] = {seq=0, stamp=current_time, frame_id = robot_id..'/'..sensor_name.."_rgb_optical_frame"}
	    d['height'] = h
	    d['width'] = w
	    d['encoding'] = 'rgb8'
	    d['is_bigendian'] = 1
	    d['step'] = w*3
	    d['data'] = data
	    simROS.publish(pubKinectRgb,d)
	end
	if(sim.getExplicitHandling(object_camera_depth) == 0) then
        -- Publish camera depth image to ROS
        data,w,h = sim.getVisionSensorCharImage(depthCam)
	    d = {}
        d['header'] = {seq=0, stamp=current_time, frame_id = robot_id..'/'..sensor_name.."_depth_optical_frame"}
	    d['height'] = h
	    d['width'] = w
	    d['encoding'] = '32FC1'--'16UC1' 
	    d['is_bigendian'] = 1 --1
	    d['step'] = w*4
	    d['data'] = data
	    simROS.publish(pubKinectDepth,d)

        -- Publish camera point cloud to ROS
        pointCloud()
        
    end      
end

--[[
header: 
  seq: 0
  stamp: 
    secs: 1527861646
    nsecs: 997300811
  frame_id: "camera_depth_optical_frame"
height: 480
width: 640
encoding: "16UC1"
is_bigendian: 0
step: 1280
data: [0, 0, ... ] # array is 614'400 long, i.e. 2 entries per pixel--]]