if (sim_call_type==sim.childscriptcall_initialization) then 
    depthCam=sim.getObjectHandle('kinect_depth')
    depthView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    sim.adjustView(depthView,depthCam,64)

    colorCam=sim.getObjectHandle('kinect_rgb')
    print(sim.getObjectName(colorCam))
    colorView=sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
    sim.adjustView(colorView,colorCam,64)

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
end 

if (sim_call_type==sim.childscriptcall_cleanup) then 
    simROS.shutdownPublisher(pubKinectRgb)
    simROS.shutdownPublisher(pubKinectDepth)
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
    end      
end
