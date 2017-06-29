--**************************
-- Tube communication to YouBot
-- @author Klaus Raizer
-- @author Ricardo Souza
-- @date 24-02-2017
--
-- Description: Functions for tube communications between YouBot and external controller
--**************************
if (sim_call_type==sim_childscriptcall_initialization) then

	-- Put some initialization code here

	-- Make sure you read the section on "Accessing general-type objects programmatically"
	-- For instance, if you wish to retrieve the handle of a scene object, use following instruction:
	--
	-- handle=simGetObjectHandle('sceneObjectName')
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
	-- handle=simGetObjectHandle('sceneObjectName#') always retrieves the handle of object 'sceneObjectName'
	-- handle=simGetObjectHandle('sceneObjectName#0') always retrieves the handle of object 'sceneObjectName#0'
	-- handle=simGetObjectHandle('sceneObjectName#1') always retrieves the handle of object 'sceneObjectName#1'
	-- ...
	--
	-- Refer also to simGetCollisionhandle, simGetDistanceHandle, simGetIkGroupHandle, etc.
	--
	-- Following 2 instructions might also be useful: simGetNameSuffix and simSetNameSuffix

	vehicleReference = simGetObjectHandle('youBot_vehicleReference')
	vehicleTarget = simGetObjectHandle('youBot_vehicleTargetPosition')

	robotPlatSensor1 = simGetObjectHandle('platformSensor1')
	robotPlatSensor2 = simGetObjectHandle('platformSensor2')
	robotPlatSensor3 = simGetObjectHandle('platformSensor3')

	actionTube=simTubeOpen(1, 'armAction'..simGetNameSuffix(nil),1)
	actionStatusTube=simTubeOpen(1, 'armActionStatus'..simGetNameSuffix(nil),1)
	statusTube=simTubeOpen(1, 'status'..simGetNameSuffix(nil),1)
	moveStatusTube=simTubeOpen(1, 'moveStatus'..simGetNameSuffix(nil), 1)

	--robot states
	stop=0
	follow=1
	replan=2

end


if (sim_call_type==sim_childscriptcall_actuation) then

	-- Put your main ACTUATION code here

	-- For example:
	--
	-- local position=simGetObjectPosition(handle,-1)
	-- position[1]=position[1]+0.001
	-- simSetObjectPosition(handle,-1,position)

end


if (sim_call_type==sim_childscriptcall_sensing) then

	-- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

	-- Put some restoration code here

end


-- Verify if robot is moving
checkRobotMovement=function()
    --p1=simGetObjectPosition(vehicleTarget,-1)
    --p2=simGetObjectPosition(vehicleReference,-1)
    --p={p2[1]-p1[1],p2[2]-p1[2]}
    --pError=math.sqrt(p[1]*p[1]+p[2]*p[2])
    --oError=math.abs(simGetObjectOrientation(vehicleReference,vehicleTarget)[3])
		cs, cr, cw = simTubeStatus(moveStatusTube)
    if cs ==1 and cr == 1 then
        tubeData = simTubeRead(moveStatusTube)
        if tonumber(tubeData) == stop then
            return {1}, {}, {"True"}, ""
        end
    end
    return {0}, {}, {"False"}, ""

    --if(pError<0.001)and(oError<0.1*math.pi/180) then
        --return {1}, {}, {"True"}, ""
    --else
    --    return {0}, {}, {"False"}, ""
    --end
end

-- Verify if arm is moving / under operation
checkArmMovement=function()
    cs, cr, cw = simTubeStatus(actionStatusTube)
    if cs ==1 and cr == 1 then
        tubeData = simTubeRead(actionStatusTube)
				print(tubeData)
        if(tubeData == 'done') then
            return {0}, {}, {}, ""
        end
    end
    return {1}, {}, {}, ""
end

-- Pick Up command
pickup=function(inInts, inFloats, inStrings, inBuffer)
    if #inStrings ~= 1 then
        return {0}, {}, {"Wrong parameters"}, ""
    end
    objectHandle = simGetObjectHandle(inStrings[1])
    if objectHandle == -1 then
        return {0}, {}, {"Object not found"}, ""
    end
    objectName = inStrings[1]
    simTubeWrite(actionTube, 'pickup:'..objectName)
    print( "pickup: ", objectName )
    return {1}, {}, {"True"}, ""
end

-- Drop command - receives objectID and place
dropAtPlace=function(inInts, inFloats, inStrings, inBuffer)
    if #inStrings ~= 2 then
        return {0}, {}, {"Wrong paratemter"}, ""
    end
    objectHandle = simGetObjectHandle(inStrings[1])
    placeHandle = simGetObjectHandle(inStrings[2])
    if objectHandle == -1 or placeHandle == -1 then
        return {0}, {}, {"object not found"}, ""
    end
    objectName = inStrings[1]
    placeName = inStrings[2]
    simTubeWrite(actionTube, 'drop:'..objectName..":"..placeName)
    return {1}, {}, {"True"}, ""
end

-- Set robot target
moveRobot=function(inInts,inFloats,inStrings,inBuffer)
    if #inStrings >= 1 then
				if string.match(inStrings[1], '#') == nil then
					target = inStrings[1]..'#'
				else
					target = inStrings[1]
				end
        placeHandle = simGetObjectHandle(target)
        if placeHandle == -1 then
            return {0}, {}, {"place not found"}, ""
        end
        placeName = target
        simTubeWrite(actionTube, 'move:'..placeName)
        return {1}, {}, {"True"}, ""
    end
    return {0}, {}, {"parameters error"}, ""
end

-- Get robot position
-- TODO: put pose reading on sensing loop
getRobotPosition=function(inInts, inFloats, inStrings, inBuffer)
    robotPosition = simGetObjectPosition(vehicleReference, -1)
    robotOrientation = simGetObjectOrientation(vehicleReference, -1)[3]
    return {1, robotPosition[1], robotPosition[2], robotOrientation}, {}, {}, ""
end

-- util - string split
function mysplit(inString,sep)
    if sep ==nil then sep="%s"
    end
    local t,i={},1
    for str in string.gmatch(inString, "([^"..sep.."]+)") do
        t[i]=str
        i=i+1
    end
    return t
end

--Get information of robot's cargo
cargoInfo=function(inInts, inFloats, inStrings, inBuff)
    -- first sensor
    local res, dist, pt, obj = simHandleProximitySensor(robotPlatSensor1)
            if obj then
                nm1 = simGetObjectName(obj)
            else
                nm1 = 'free'
            end
    -- second sensor
    local res, dist, pt, obj = simHandleProximitySensor(robotPlatSensor2)
        if obj then
                nm2 = simGetObjectName(obj)
            else
                nm2 = 'free'
            end
    -- third sensor
    local res, dist, pt, obj = simHandleProximitySensor(robotPlatSensor3)
        if obj then
                nm3 = simGetObjectName(obj)
            else
                nm3 = 'free'
            end

    return {1}, {}, {nm1, nm2, nm3}, ""
end

-- recharge function
recharge=function(inInts, inFloats, inStrings, inBuffer)
    if #inStrings >= 1 then
        operation = inStrings[1]
        if operation == 'start' then
            placeHandle = simGetObjectHandle(inStrings[2])
            if placeHandle == -1 then
                return {0}, {}, {"recharger does not exist"}, ""
            end
            placeName = inStrings[2]
            simTubeWrite(actionTube, 'recharge:'..operation..':'..placeName)
            return {1}, {}, {"True"}, ""
        elseif operation == 'stop' then
            simTubeWrite(actionTube, 'recharge:'..operation)
            return {1}, {}, {"True"}, ""
        end
        return {0}, {}, {"Unkown command"}, ""
    end
    return {0}, {}, {"parameters errors"}, ""
end
