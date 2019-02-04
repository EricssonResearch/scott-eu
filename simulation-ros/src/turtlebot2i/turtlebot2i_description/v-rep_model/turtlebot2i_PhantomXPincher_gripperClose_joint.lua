function sysCall_init()
    -- do some initialization here:

end

function sysCall_actuation()
    -- put your actuation code here
 
end

function sysCall_sensing()
    -- put your sensing code here
    self_handle=sim.getObjectAssociatedWithScript(sim.handle_self)

    joint5_pos=sim.getJointPosition(self_handle)

    joint5_max=0.032--math.pi/2
    joint5_min=-0.05---math.pi/2

    gripleft_max=0.015 --1.500e-2
    gripleft_min=0

    -- g=((joint5_pos-joint5_min)/(joint5_max-joint5_min))*(gripleft_max-gripleft_min)
    g = sim.getJointPosition(self_handle)

    handle=sim.getObjectHandle('PhantomXPincher_gripperCenter_joint0')
    handle0=sim.getObjectHandle('PhantomXPincher_gripperCenter_joint')
    arm_attach_point_handle=sim.getObjectHandle('arm_attach_point')

    --f=sim.getJointForce(handle)
    sim.setJointTargetPosition(handle,g)
    sim.setJointTargetPosition(handle0,0.008)

    prox_sense_left_handle=sim.getObjectHandle('Proximity_sensor_left')
    prox_sense_right_handle=sim.getObjectHandle('Proximity_sensor_right')

    local res_l,dist_l,pt_l,obj_l=sim.handleProximitySensor(prox_sense_left_handle)
    local res_r,dist_r,pt_r,obj_r=sim.handleProximitySensor(prox_sense_right_handle)

    --print('-------------')
    --print('res_l: '..res_l)
   -- print('res_r: '..res_r)
    if (not(obj_l==nil) and not(obj_r==nil))then
        result=sim.setObjectParent(obj_l, arm_attach_point_handle,true)
    else
        if(not(obj_l==nil))then
        result=sim.setObjectParent(obj_l, -1,false)
        end
        if(not(obj_r==nil))then
        result=sim.setObjectParent(obj_r, -1,false)
        end
    end



    
--    print('prox_sense_handle: ',sim.getObjectName(prox_sense_handle))
 --   result=sim.handleProximitySensor(prox_sense_handle)
--print('-----')
--print('result: ',result)
--print('detectedObjectHandle: ',detectedObjectHandle)


end

function sysCall_cleanup()
    -- do some clean-up here
end


    -- Make sure you read the section on "Accessing general-type objects programmatically"
    -- For instance, if you wish to retrieve the handle of a scene object, use following instruction:
    --
    -- handle=sim.getObjectHandle('sceneObjectName')
    -- 
    -- Above instruction retrieves the handle of 'sceneObjectName' if this script's name has no '#' in it
    --
    -- If this script's name contains a '#' (e.g. 'someName#4'), then above instruction retrieves the handle of object 'sceneObjectName#4'
    -- This mechanism of handle retrieval is very convenient, since you don't need to adjust any code when a model is duplicated!
    -- So if the script's name (or rather the name of the object associated with this script) is:
    --
    -- 'someName', then the handle of 'sceneObjectName' is retrieved
    -- 'someName#0', then the handle of 'sceneObjectName#0' is retrieved
    -- 'someName#1', then the handle of 'sceneObjectName#1' is retrieved
    -- ...
    --
    -- If you always want to retrieve the same object's handle, no matter what, specify its full name, including a '#':
    --
    -- handle=sim.getObjectHandle('sceneObjectName#') always retrieves the handle of object 'sceneObjectName' 
    -- handle=sim.getObjectHandle('sceneObjectName#0') always retrieves the handle of object 'sceneObjectName#0' 
    -- handle=sim.getObjectHandle('sceneObjectName#1') always retrieves the handle of object 'sceneObjectName#1'
    -- ...
    --
    -- Refer also to sim.getCollisionhandle, sim.getDistanceHandle, sim.getIkGroupHandle, etc.

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end
--]]


