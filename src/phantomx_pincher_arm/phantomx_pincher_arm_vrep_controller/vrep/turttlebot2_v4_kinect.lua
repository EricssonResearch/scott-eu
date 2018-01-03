function pointCloud()
    local depthBuffer=sim.getVisionSensorDepthBuffer(depthCam)
    local header = {seq=0,stamp=simROS.getTime(), frame_id="k"}
    local points = {}
    --local point = {x=0,y=0,z=0}
    --local ch = {name="distance",values={}}
    local channel = {}
    for i=1,64,1 do
        local xAngle=((32-i-0.5)/32)*camXHalfAngle
        for j=1,48,1 do
            local yAngle=((j-24+0.5)/24)*camYHalfAngle
            local depthValue=depthBuffer[i+(j-1)*64]
            local zCoord=nearClippingPlane+depthAmplitude*depthValue
            local xCoord=math.tan(xAngle)*zCoord
            local yCoord=math.tan(yAngle)*zCoord
            local dist=math.sqrt(xCoord*xCoord+yCoord*yCoord+zCoord*zCoord)
            local point = {x=xCoord,y=yCoord,z=zCoord}
            --point["x"] = xCoord
            --point["y"] = yCoord
            --point["z"] = zCoord
	    points[i+(j-1)*64] = point

            local ch = {name="distance",values={dist}}
	    --ch["values"] = {dist}
            channel[i+(j-1)*64] = ch 
        end
    end
    local point_cloud = {}
    point_cloud["header"] = header
    point_cloud["points"] = points
    point_cloud["channels"] = channel
    --print(#points)
    --print(#channel)
    simROS.publish(pubKinectCloud,point_cloud)
end

if (sim_call_type==sim.childscriptcall_initialization) then 
    depthCam=sim.getObjectHandle('kinect_depth')
    depthView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    sim.adjustView(depthView,depthCam,64)

    colorCam=sim.getObjectHandle('kinect_rgb')
    print(sim.getObjectName(colorCam))
    colorView=sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
    sim.adjustView(colorView,colorCam,64)

    -- camera resolution to 64 to improve performance
    camXAngleInDegrees = 67
    camXResolution = 64
    camYResolution = 48
    camXHalfAngle=camXAngleInDegrees*0.5*math.pi/180
    camYHalfAngle=(camXAngleInDegrees*0.5*math.pi/180)*48/64
    nearClippingPlane=0.2
    depthAmplitude=3.3

    objHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    parentHandle = simGetObjectParent(objHandle)

    if parentHandle ~= -1 then
        modelBaseName = sim.getObjectName(parentHandle).."/"..sim.getObjectName(objHandle)
     else
        modelBaseName = sim.getObjectName(objHandle)
    end
    modelBaseName = string.gsub(modelBaseName,"#","_")
    -- ROS Stuff
	pubKinectRgb = simROS.advertise(modelBaseName..'/rgb/raw_image','sensor_msgs/Image')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectRgb) 
	pubKinectDepth = simROS.advertise(modelBaseName..'/depth/raw_image','sensor_msgs/Image')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectDepth)
        pubKinectCloud = simROS.advertise(modelBaseName..'/cloud','sensor_msgs/PointCloud')
	simROS.publisherTreatUInt8ArrayAsString(pubKinectCloud)
end 

if (sim_call_type==sim.childscriptcall_cleanup) then 
    simROS.shutdownPublisher(pubKinectRgb)
    simROS.shutdownPublisher(pubKinectDepth)
    simROS.shutdownPublisher(pubKinectCloud)
end 

if (sim_call_type==sim.childscriptcall_sensing) then
    -- ROS Kinect
    if(sim.getBoolParameter(sim.boolparam_vision_sensor_handling_enabled) == true) then
        local data,w,h = sim.getVisionSensorCharImage(colorCam)
	    d = {}
        d['header'] = {seq=0,stamp=simROS.getTime(), frame_id="k"}
	    d['height'] = h
	    d['width'] = w
	    d['encoding'] = 'rgb8'
	    d['is_bigendian'] = 1
	    d['step'] = w*3
	    d['data'] = data
	    simROS.publish(pubKinectRgb,d)

            data,w,h = sim.getVisionSensorCharImage(depthCam)
	    d = {}
            d['header'] = {seq=0,stamp=simROS.getTime(), frame_id="k"}
	    d['height'] = h
	    d['width'] = w
	    d['encoding'] = 'rgb8'
	    d['is_bigendian'] = 1
	    d['step'] = w*3
	    d['data'] = data
	    simROS.publish(pubKinectDepth,d)	
            -- point cloud calc
            pointCloud()
        

    end      
end
