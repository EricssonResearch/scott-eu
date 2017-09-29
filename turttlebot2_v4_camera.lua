function pointCloud()

--    id_res_x = sim_visionintparam_resolution_x
--    id_res_y = sim_visionintparam_resolution_y
--    r,res_x = simGetObjectInt32Parameter(depthCam, id_res_x, 0)
--    r,res_y = simGetObjectInt32Parameter(depthCam, id_res_y, 0)
--    local height = res_y 
--    local width  = res_x

    local header = {seq = 0, stamp = simROS.getTime(), frame_id = sensorName.."_link"}
    local data = {}
    local height = 160 
    local width  = 120
--    local height = 320 
--    local width  = 240
--    local height = 480
--    local width  = 640
    local fields = {{ name='x', offset=0, datatype= 7, count=1 },
                    { name='y', offset=4, datatype= 7, count=1 },
                    { name='z', offset=8, datatype= 7, count=1 }}

--    m1 = sim.getObjectMatrix(depthCam, -1)

    r,t1,u1 = simReadVisionSensor(depthCam)
--    r,t1,u1 = simReadVisionSensor(depthCam)

    if u1 then
        for j = 0, u1[2]-1, 1 do
            for i = 0, u1[1]-1, 1 do
                local w = 2+4*(j*u1[1]+i)
                local p = {u1[w+1], u1[w+2], u1[w+3]}
--                p = simMultiplyVector(m1, p)
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

    -- Depth camera handler
    depthCam = sim.getObjectHandle('camera_depth')
    depthView = sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    sim.adjustView(depthView,depthCam,64)

    -- Color camera handler
    colorCam = sim.getObjectHandle('camera_rgb')
    colorView = sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
    sim.adjustView(colorView,colorCam,64)

    objHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    parentHandle = simGetObjectParent(objHandle)

    sensorName = sim.getObjectName(objHandle)
    sensorName = string.gsub(sensorName,"#","")

    if parentHandle ~= -1 then
        modelBaseName = sim.getObjectName(parentHandle).."/"..sensorName
     else
        modelBaseName = sensorName
    end

--    modelBaseName = string.gsub(modelBaseName,"#","_")
    
    -- ROS Stuff
	pubKinectRgb = simROS.advertise(modelBaseName..'/rgb/raw_image','sensor_msgs/Image')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectRgb) 
	pubKinectDepth = simROS.advertise(modelBaseName..'/depth/raw_image','sensor_msgs/Image')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectDepth)
    pubKinectCloud = simROS.advertise(modelBaseName..'/cloud','sensor_msgs/PointCloud2')
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
        d['header'] = {seq=0, stamp=simROS.getTime(), frame_id = sensorName.."_link_optical"}
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
        d['header'] = {seq=0, stamp=simROS.getTime(), frame_id = sensorName.."_link_optical"}
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
