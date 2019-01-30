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

    robot_id = sim.getStringSignal("robot_id")

    -- Disable camera sensor (comment the lines below to enable) 
    object_camera_rgb = sim.getObjectHandle('camera_rgb')
    object_camera_depth = sim.getObjectHandle('camera_depth')
    sim.setExplicitHandling(object_camera_rgb, 1) -- disable camera rgb
    sim.setExplicitHandling(object_camera_depth, 1) -- disable camera depth

    -- Get object handler
    modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    object_name = sim.getObjectName(modelHandle)
    sensor_number, sensor_name = sim.getNameSuffix(object_name)

    -- Depth camera handler
    depthCam = sim.getObjectHandle(sensor_name..'_depth')
    depthView = sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    sim.adjustView(depthView,depthCam,64)

    -- Color camera handler
    colorCam = sim.getObjectHandle(sensor_name..'_rgb')
    colorView = sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
    sim.adjustView(colorView,colorCam,64)
    
    -- ROS Stuff
	pubKinectRgb = simROS.advertise(robot_id..'/'..sensor_name..'/rgb/raw_image','sensor_msgs/Image')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectRgb) 
	pubKinectDepth = simROS.advertise(robot_id..'/'..sensor_name..'/depth/raw_image','sensor_msgs/Image')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectDepth)
    pubKinectCloud = simROS.advertise(robot_id..'/'..sensor_name..'/cloud','sensor_msgs/PointCloud2')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectCloud)
end 

if (sim_call_type==sim.childscriptcall_cleanup) then 
    simROS.shutdownPublisher(pubKinectRgb)
    simROS.shutdownPublisher(pubKinectDepth)
    simROS.shutdownPublisher(pubKinectCloud)
end 

if (sim_call_type==sim.childscriptcall_sensing) then
    if(sim.getBoolParameter(sim.boolparam_vision_sensor_handling_enabled) == true) then

        local data,w,h = sim.getVisionSensorCharImage(colorCam)

        -- Publish camera RGB image to ROS
	    d = {}
        d['header'] = {seq=0, stamp=simROS.getTime(), frame_id = robot_id..'/'..sensor_name.."_rgb_optical_frame"}
	    d['height'] = h
	    d['width'] = w
	    d['encoding'] = 'rgb8'
	    d['is_bigendian'] = 1
	    d['step'] = w*3
	    d['data'] = data
	    simROS.publish(pubKinectRgb,d)

        -- Publish camera depth image to ROS
        data,w,h = sim.getVisionSensorCharImage(depthCam)
	    d = {}
        d['header'] = {seq=0, stamp=simROS.getTime(), frame_id = robot_id..'/'..sensor_name.."_depth_optical_frame"}
	    d['height'] = h
	    d['width'] = w
	    d['encoding'] = 'rgb8'
	    d['is_bigendian'] = 1
	    d['step'] = w*3
	    d['data'] = data
	    simROS.publish(pubKinectDepth,d)

        -- Publish camera point cloud to ROS
        pointCloud()
        
    end      
end
