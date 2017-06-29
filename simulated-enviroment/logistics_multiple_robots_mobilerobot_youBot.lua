--**************************
-- Actions for the YouBot Kuka Robot
-- @author Klaus Raizer
-- @author Ricardo Souza
-- @date 19-04-2016
--
-- Description: High level actions to control this Kuka Robot
--**************************
threadFunction=function()
-- Put some initialization code here:

simSetThreadSwitchTiming(100) -- Default timing for automatic thread switching

--open internal communication tube
statusTube=simTubeOpen(1, 'status'..simGetNameSuffix(nil),1)
actionStatusTube=simTubeOpen(1, 'armActionStatus'..simGetNameSuffix(nil),1)
actionTube=simTubeOpen(1, 'armAction'..simGetNameSuffix(nil),1)

print('robot waiting for arm command')

--- AUXILIARY FUNCTIONS ---
setIkMode=function(withOrientation)
    simSetThreadAutomaticSwitch(false) -- Don't get interrupted for this function
    if (ikMode==false) then
        simSetObjectPosition(gripperTarget,-1,simGetObjectPosition(gripperTip,-1))
    end
    if (withOrientation) then
        simSetExplicitHandling(ikWithOrientation1,0)
        simSetExplicitHandling(ikWithOrientation2,0)
    else
        simSetExplicitHandling(ik1,0)
        simSetExplicitHandling(ik2,0)
    end
    for i=1,5,1 do
        simSetJointMode(armJoints[i],sim_jointmode_ik,1)
    end
    ikMode=true -- [Why this?]
    simSetThreadAutomaticSwitch(true)
end

setFkMode=function()
    simSetThreadAutomaticSwitch(false) -- Don't get interrupted for this function
    simSetExplicitHandling(ik1,1)
    simSetExplicitHandling(ik2,1)
    simSetExplicitHandling(ikWithOrientation1,1)
    simSetExplicitHandling(ikWithOrientation2,1)

    for i=1,5,1 do
        simSetJointMode(armJoints[i],sim_jointmode_force,0)
    end
    ikMode=false
    simSetThreadAutomaticSwitch(true)
end
-- Used to see if the robot has arrived at destination when picking up or droping products
waitToReachVehicleTargetPositionAndOrientation=function()
    repeat
        simSwitchThread() -- don't waste your time waiting!
        p1=simGetObjectPosition(goalConfigurationHandle,-1)
        p2=simGetObjectPosition(youBotCenterHandle,-1)
        p={p2[1]-p1[1],p2[2]-p1[2]}
        pError=math.sqrt(p[1]*p[1]+p[2]*p[2])
        oError=math.abs(simGetObjectOrientation(youBotCenterHandle,goalConfigurationHandle)[3])
    --until (pError<0.001)and(oError<0.1*math.pi/180) -- This solution disregards oscilations
  until (pError<0.05)and(oError<5*math.pi/180) -- This solution disregards oscilations

end

waitUntilRobotIsAtGoalPosition=function()
-- Wait until robot is at goal position
	repeat
        simSwitchThread() -- don't waste your time waiting!
		rgp=simGetObjectPosition(mobilerobotHandle,goalConfigurationHandle)
		distance_to_goal= math.sqrt(rgp[1]*rgp[1]+rgp[2]*rgp[2])--2D
		--print("distance_to_goal: "..distance_to_goal)
	until (distance_to_goal<=minimum_distance_to_waypoint) 
end

getBoxAdjustedMatrixAndFacingAngle=function(boxHandle)
    p2=simGetObjectPosition(boxHandle,-1)
    p1=simGetObjectPosition(vehicleReference,-1)
    p={p2[1]-p1[1],p2[2]-p1[2],p2[3]-p1[3]}
    pl=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
    p[1]=p[1]/pl
    p[2]=p[2]/pl
    p[3]=p[3]/pl
    m=simGetObjectMatrix(boxHandle,-1)
    -- m is a matrix: table of 12 numbers (the last row of the 4x4 matrix (0,0,0,1) is not returned), or nil in case of an error. Table values in Lua are indexed from 1, not 0!

    matchingScore=0
    for i=1,3,1 do
        v={m[0+i],m[4+i],m[8+i]}
        score=v[1]*p[1]+v[2]*p[2]+v[3]*p[3]
        if (math.abs(score)>matchingScore) then
            s=1
            if (score<0) then s=-1 end
            matchingScore=math.abs(score)
            bestMatch={v[1]*s,v[2]*s,v[3]*s}
        end
    end
    angle=math.atan2(bestMatch[2],bestMatch[1])
    m=simBuildMatrix(p2,{0,0,angle})
    return m, angle-math.pi/2
end


--- GENERAL VARIABLES ---
--maximum_pickup_and_drop_distance=1 -- 1 meter minimum distance to inititate a pickup or drop action

--suf = simGetNameSuffix(nil)
--if (suf ~= -1) then
--	suf = '#'..suf
--else
--	suf = ''
--end

gripperTarget=simGetObjectHandle('youBot_gripperPositionTarget')
gripperTip=simGetObjectHandle('youBot_gripperPositionTip')
vehicleReference=simGetObjectHandle('youBot_vehicleReference')
youBotCenterHandle=simGetObjectHandle('youBot_Center')
vehicleTarget=simGetObjectHandle('youBot_vehicleTargetPosition')
--goalCylinderHandle=simGetObjectHandle('mobilerobot_goalPosCylinder#')
goalConfigurationHandle=simGetObjectHandle('youBotGoalConfiguration')
youBotGoalHandle=simGetObjectHandle('youBotGoal')
mobilerobotHandle=simGetObjectHandle('mobilerobot_youBot')

armJoints={-1,-1,-1,-1,-1}
for i=0,4,1 do
    armJoints[i+1]=simGetObjectHandle('youBotArmJoint'..i)
end
ik1=simGetIkGroupHandle('youBotUndamped_group')
ik2=simGetIkGroupHandle('youBotDamped_group')
ikWithOrientation1=simGetIkGroupHandle('youBotPosAndOrientUndamped_group')
ikWithOrientation2=simGetIkGroupHandle('youBotPosAndOrientDamped_group')
gripperCommunicationTube=simTubeOpen(0,'youBotGripperState'..simGetNameSuffix(nil),1)

ikSpeed={360,360,360,360}
ikAccel={20,20,20,20}
ikJerk={20,20,20,20}
fkSpeed={360,360,360,360,360}
fkAccel={20,20,20,20,20}
fkJerk={20,20,20,20,20}

--- ROBOT VARIABLES ---
cargoHold={"free","free","free"}

backPickupPosition={0,-0*math.pi/180,90*math.pi/180,0*math.pi/180,-0*math.pi/180}
frontalPickupPosition={180*math.pi/180,-0*math.pi/180,90*math.pi/180,60*math.pi/180,-0*math.pi/180}
intermediatePickupPosition={90*math.pi/180,-0*math.pi/180,90*math.pi/180,60*math.pi/180,-0*math.pi/180}-- makes sure it turns counter-clock wise (because for some uncanny reason, in frontalPickupPosition, youBot gets confused between 180 and -180)

cargoArmPosition={15*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180}
cargoArmPosition0={0,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180}
cargoArmPosition1={-15*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180}

dropDistance=0.05 -- Distance in the z direction of the place (local frame, not global) to drop the product
previous_waypoint_handle=nil -- holds the last waypoint's handle the robot was at
minimum_distance_to_waypoint=0.01 --The minimum distance the robot must be from a given waypoint to consider it as being there
distanceFromPlace=0.15  -- A safety distance to be kept from an object the robot is trying to approach
maximum_allowed_angle_error=0.01 -- A maximum allowed angle error when judging if a joint has arrived at its expected location
recharge_approach_distance=0.15 -- Distance necessary for the recharging mechanism to engage

--- ROBOT FUNCTIONS ---
openGripper=function()
    simTubeWrite(gripperCommunicationTube,simPackInts({1}))
    simWait(0.8)
end

closeGripper=function()
    simTubeWrite(gripperCommunicationTube,simPackInts({0}))
    simWait(0.8)
end

-- Returns the robots' current aypoint location. Returns nil if too far away from any waypoint
getCurrentWaypoint=function()
	listOfShapesInTheScene=simGetObjectsInTree(sim_handle_scene ,sim_object_shape_type, 0) --table
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."--- getCurrentWaypoint ---")
	--temphandle=simGetObjectHandle("Waypoint_SH#0")
	--temptype=simGetObjectType(temphandle)
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."temptype: "..temptype)
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."sim_object_shape_type: "..sim_object_shape_type)
	
	-- Get list of waypoints in the scene --TODO: turn this into a sepparated function? 
	list_of_waypoints={}
	count=1
	for i=1,#listOfShapesInTheScene,1 do
	  if (string.match(simGetObjectName(listOfShapesInTheScene[i]), "Waypoint"))then
	--	print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] "..simGetObjectName(listOfShapesInTheScene[i]))
		list_of_waypoints[count]=listOfShapesInTheScene[i]
	--	print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."list_of_waypoints["..count.."]: "..list_of_waypoints[count])
		count=count+1
	  end
    end
	
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."---2-----")

	
	-- Get closest waypoint'
	closest_distance=math.huge
	closest_waypoint_handle=nil
	--print("#list_of_waypoints: "..#list_of_waypoints)
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."]")
	for i=1,#list_of_waypoints,1 do
			
			
		if(closest_waypoint_handle==nil)then
		--	print("closest_waypoint_handle: ",closest_waypoint_handle)
		--	print("closest_waypoint_name: nil")
		--	print("list_of_waypoints["..i.."]: ",list_of_waypoints[i])
			
			closest_waypoint_handle=list_of_waypoints[i]
		else
		--	print("closest_waypoint_handle: ",closest_waypoint_handle)
		--	print("closest_waypoint_name: ",simGetObjectName(closest_waypoint_handle))
		--	print("list_of_waypoints["..i.."]: ",list_of_waypoints[i])
			
			rp=simGetObjectPosition(youBotGoalHandle,list_of_waypoints[i])
			d=math.sqrt(rp[1]*rp[1]+rp[2]*rp[2])-- NOTE: 2d space
		--	print("d: "..d)
			if(d<=closest_distance)then 
				closest_distance=d
				closest_waypoint_handle=list_of_waypoints[i]
			end
		end
	end
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."---3--------")
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."closest_waypoint_name: "..simGetObjectName(closest_waypoint_handle))
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."closest_distance: "..closest_distance)
	-- If closest waypoint is close enough, return it. Else, return nil.
	if(closest_distance<=minimum_distance_to_waypoint)then 
		current_waypoint=closest_waypoint_handle
		--print("current_waypoint=closest_waypoint_handle")
	else
		current_waypoint=nil
	end
	--print("[youBot Name: "..simGetObjectName(youBotGoalHandle).."] ".."------")
	return current_waypoint
	
end

-- Move the robot from where it is to the given waypoint
moveToWaypoint=function(targetPlaceName)
    if (not string.match(targetPlaceName, "Waypoint"))then
        print('Error. The string passed to move function is not a waypoint.')
        simTubeWrite(actionStatusTube, "fail")
        return false
    else
        -- Get positions of place and vehicle
        --print('targetPlaceName: ',targetPlaceName)
        placeHandle = simGetObjectHandle(targetPlaceName)
        xyzOfTargetWaypoint=simGetObjectPosition(placeHandle,-1)


        --xyzOfVehicle=simGetObjectPosition(vehicleReference,-1)
        xyzOfGoal=simGetObjectPosition(goalConfigurationHandle,-1)

        anglesOfTargetWaypoint=simGetObjectOrientation(placeHandle,-1)
        --Tell vehicle to go to given place
        simSetObjectPosition(goalConfigurationHandle,-1,{xyzOfTargetWaypoint[1],xyzOfTargetWaypoint[2],xyzOfGoal[3]}) --Keep z of vehicle
        simSetObjectOrientation(goalConfigurationHandle,-1,anglesOfTargetWaypoint)

        --Wait for the movement to finish
        --waitToReachVehicleTargetPositionAndOrientation()
        --If finished, update variable holding the last waypoint the robot was at
        previous_waypoint_name=targetPlaceName
        return true
    end
end

-- Picks up an object with the gripper and stores it in its cargo hold
pickup=function(objectName) --TODO Should we change it to "pickupProductAtShelf"?
    
	--print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".."is picking up "..objectName)
	targetProductHandle=simGetObjectHandle(objectName)
	
    if(targetProductHandle==-1)then
        --print("The object ", objectName, " does not exist.") -- Should not happen if the controller is careful
        simTubeWrite(actionStatusTube, "fail")
        return false
    end

    if((cargoHold[1] ~= "free")and(cargoHold[3] ~= "free")and(cargoHold[3] ~= "free"))then -- Checks for available cargo space
       -- print("All cargo slots are being used.")
        simTubeWrite(actionStatusTube, "fail")
        return false
    end
	
	
	current_waypoint=getCurrentWaypoint()-- Check if it is at a shelf waypoint
	--print("Robot ".. simGetObjectName(mobilerobotHandle).." current_waypoint: ",current_waypoint)
	current_waypoint_name=simGetObjectName(current_waypoint)
	--print("Robot ".. simGetObjectName(mobilerobotHandle).." current_waypoint_name: ",current_waypoint_name)
	
	--TODO: IMPORTANT, this should not happen (current_waypoint==nil)
	if( (current_waypoint==nil) or (not string.match(simGetObjectName(current_waypoint), "_SH")) )then
		--print("Robot ".. simGetObjectName(mobilerobotHandle).." is not at a shelf waypoint.")
        simTubeWrite(actionStatusTube, "fail")
        return false
	end

	--TODO verify if desired product is at the current shelf
	
	
	--print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".."pickup is ok to go.")
	
    -- First, move the gripper to the frontal pickup position to make IK easier
	simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,intermediatePickupPosition,nil) -- makes sure it turns counter-clock wise
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,frontalPickupPosition,nil)
	
	--simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,frontalPickupPosition,nil)
	
    gripperPosAtFrontalPickupPosition=simGetObjectPosition(gripperTip,-1)
    simSetObjectPosition(gripperTarget,-1,gripperPosAtFrontalPickupPosition) -- Make sure the gripper target is at the grippers current position [TODO: isn't there a way to make this automatic?]

	-- Enable Inverse Kinematics
    setIkMode(true)
    -- Recover positions of the target product and its facing angles
    m,angle=getBoxAdjustedMatrixAndFacingAngle(targetProductHandle)
    p=simGetObjectPosition(gripperTarget,-1)
    p[1]=m[4]
    p[2]=m[8]
    p[3]=m[12]

	-- PROCEDURE: APPROACH SHELF
	--print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".." Adjust position of the robot.")
    -- Adjust position of the robot based on target product
	previousGoalConfigurationPosition=simGetObjectPosition(goalConfigurationHandle,-1)
	
	p=simGetObjectPosition(targetProductHandle,-1)
	r=simGetObjectPosition(goalConfigurationHandle,-1)

	result, goalXmaxSize=simGetObjectFloatParameter(youBotGoalHandle,sim_objfloatparam_objbbox_max_x )
	
	-- Move goal pgap in the propper direction
	pgap=0.1 --specific for this shelf TODO: make it find out by itself based on robot/product relative position
	
	theta=simGetObjectOrientation(youBotGoalHandle,-1)
	
	v={math.sin(theta[3])+r[1], math.cos(theta[3])+r[2]}
	
	direction_to_shelf={v[1]-r[1],v[2]-r[2]}
	
	direction_to_product={p[1]-r[1],p[2]-r[2]}
	
	direction=(direction_to_shelf[1]*direction_to_product[1]+direction_to_shelf[2]*direction_to_product[2])
	print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".."direction: ",direction)
	if(direction>0.01)then
		direction=1
	elseif(direction<-0.01)then
		direction=-1
	else
		direction=0
	end
	--print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".."norm direction: ",direction)
	
	
	simSetObjectPosition(goalConfigurationHandle,goalConfigurationHandle,{0,pgap*direction,0})
	
	-- Calculating gap
	
	
	g=simGetObjectPosition(goalConfigurationHandle,-1)
	
	distance_from_goal_to_product=math.sqrt( (g[1]-p[1])*(g[1]-p[1])+(g[2]-p[2])*(g[2]-p[2]) )

	gap=distance_from_goal_to_product-goalXmaxSize-distanceFromPlace
	
	c=gap/distance_from_goal_to_product
	
	-- Closing the gap taking into consideration the size of the robot's bounding box
	q=g
	q[1]=g[1]+(p[1]-g[1])*c
	q[2]=g[2]+(p[2]-g[2])*c
	
	simSetObjectPosition(goalConfigurationHandle,-1,{q[1],q[2],g[3]})
	
	--Now fix rotation
	--print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".." fix rotation.")
	product_orientation=simGetObjectOrientation(targetProductHandle,-1)
	o=simGetObjectOrientation(goalConfigurationHandle,-1)
	simSetObjectOrientation(goalConfigurationHandle,-1,{o[1],o[2],product_orientation[3]+math.pi})
	
	
	--m2,angle2=getBoxAdjustedMatrixAndFacingAngle(targetProductHandle)
	--print("m2[4], m2[8], m2[12]: "..m2[4]..", ".. m2[8]..", "..  m2[12])
	--print("angle2: "..angle2)
	
	
	--simSetObjectPosition(goalConfigurationHandle,-1,{m2[4]-m2[1]*distanceFromPlace,m2[8]-m2[5]*distanceFromPlace,0})
    --simSetObjectOrientation(goalConfigurationHandle,-1,{0,0,angle2})
	
	
	
	-- Verify if robot is at the pickup position
	waitUntilRobotIsAtGoalPosition()
	
	
	
	
	--PROCEDURE: PICKUP PRODUCT AND STORE IN CARGO
	
	
--[[
    previousVehicleTargetPosition=simGetObjectPosition(vehicleTarget,-1)
    previousVehicleTargetOrientation=simGetObjectOrientation(vehicleTarget,-1)
	
    m2,angle2=getBoxAdjustedMatrixAndFacingAngle(targetProductHandle)
    simSetObjectPosition(vehicleTarget,-1,{m2[4]-m2[1]*distanceFromPlace,m2[8]-m2[5]*distanceFromPlace,0})
    simSetObjectOrientation(vehicleTarget,-1,{0,0,angle2})
    waitToReachVehicleTargetPositionAndOrientation()
]]--
    -- Tell the gripper to move to the position where it can grab the target product
    p=simGetObjectPosition(targetProductHandle,-1)
    simRMLMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)

    -- Grab the product by closing the gripper
    closeGripper()

    -- Move the gripper to frontal pickup position
    setIkMode(false)--Need to do this first
    setFkMode()
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,frontalPickupPosition,nil)

    -- Move the gripper to back pickup position
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,backPickupPosition,nil)

	

	repeat
        simSwitchThread() -- don't waste your time waiting!
		--print("########")
		max_angle_error=0
		for i=1,#armJoints,1 do
			jp=simGetJointPosition(armJoints[i])
			
			angle_error=math.abs(backPickupPosition[i]-jp)
			--print("angle_error: ",angle_error)
			--print("max_angle_error: ",max_angle_error)
			if(angle_error>max_angle_error)then
				max_angle_error=angle_error
			end			
		end		
		--print("max_angle_error: "..max_angle_error)
	until (max_angle_error<=maximum_allowed_angle_error) 
	
	
	--PROCEDURE: STEP AWAY FROM SHELF
	-- Move back to shelf's waypoint
	--print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".." Move back to shelf's waypoint.")
	moveToWaypoint(simGetObjectName(current_waypoint))
	
	

    -- PROCEDURE: Dropping box at cargo
    if     cargoHold[1] == "free" then
            --dropAtPlace(objectName,"RobotsCargo")
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{15*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180},nil)
            openGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            cargoHold[1]=objectName
    elseif  cargoHold[2] == "free" then
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{0*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{0*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180},nil)
            openGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{0*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            cargoHold[2]=objectName
    elseif  cargoHold[3] == "free" then
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{-15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{-15*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180},nil)
            openGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{-15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            cargoHold[3]=objectName
    else    print("All cargo slots are being used.") -- Should not happen if we test it at the begining of the function
            simTubeWrite(actionStatusTube, "fail")
            return false
    end

    -- Move the gripper to back pickup position
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,backPickupPosition,nil)
    waitToReachVehicleTargetPositionAndOrientation()
    
		
	
	
	
	--Verify if robot is back at shelf's waypoint
	waitUntilRobotIsAtGoalPosition()
	
	
	--print("[Robot ".. simGetObjectName(mobilerobotHandle).."] ".." is done with pickup.")
	
	simTubeWrite(actionStatusTube, "done")
    return true

end





-- Drops a product at a place specified by a tag
dropAtPlace=function(objectName,placeName)
    -- Move the gripper to back pickup position
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,backPickupPosition,nil)

    if     cargoHold[1] == objectName then
            openGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{15*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180},nil)
            closeGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            cargoHold[1]="free"
    elseif  cargoHold[2] == objectName then
            openGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{0*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{0*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180},nil)
            closeGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{0*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            cargoHold[2]="free"
    elseif  cargoHold[3] == objectName then
            openGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{-15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{-15*math.pi/180,70*math.pi/180,75*math.pi/180,-55*math.pi/180,0*math.pi/180},nil)
            closeGripper()
            simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,{-15*math.pi/180,70*math.pi/180,75*math.pi/180,-90*math.pi/180,0*math.pi/180},nil)
            cargoHold[3]="free"
    else    print("The object ", objectName, " is not in the cargo hold.") -- Should not happen if we test it at the begining of the function
            simTubeWrite(actionStatusTube, "fail")
            return false
    end

    -- Move the gripper to frontal pickup position
    setIkMode(false)--Need to do this first
    setFkMode()
	simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,intermediatePickupPosition,nil) -- makes sure it turns counter-clock wise
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,frontalPickupPosition,nil)
    -- Enable Inverse Kinematics
    setIkMode(true)
	
	-- Recover positions of the target place
    targetPlace=simGetObjectHandle(placeName)
	dropingPlaceHandle=simCreateDummy(0.0000001,nil)
	result=simSetObjectPosition(dropingPlaceHandle,targetPlace,{0,0,dropDistance})
	-- simSetObjectOrientation(dummyHandle,-1,o)
	print("Result: ",result)
	
	--result, goalXmaxSize=simGetObjectFloatParameter(youBotGoalHandle,sim_objfloatparam_objbbox_max_x )
	p=simGetObjectPosition(dropingPlaceHandle,-1)
	g=simGetObjectPosition(goalConfigurationHandle,-1)
	r=simGetObjectPosition(mobilerobotHandle,-1)
	
	
	--Approach drop place
	--simSetObjectPosition(goalConfigurationHandle,-1,{r[1]+.5*(p[1]-r[1]),r[2]-.5*(p[2]-r[2]),g[3]})
	simSetObjectPosition(goalConfigurationHandle,dropingPlaceHandle,{-.8*(p[1]-r[1]),-.8*(p[2]-r[2]),g[3]})
	-- Wait until robot is at the drop position
	waitUntilRobotIsAtGoalPosition()
	
	--  PROCEDURE: Drop product at CB's dropping position
	
    --Move gripper to drop position
    p=simGetObjectPosition(dropingPlaceHandle,-1)
    simRMLMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    -- Drop the product
    openGripper()
    -- Move the gripper to frontal pickup position
    setIkMode(false)--Need to do this first
    setFkMode()
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,frontalPickupPosition,nil)
    -- Clears dummy
    simRemoveObject(dropingPlaceHandle)
	
	
	-- Return
	simSetObjectPosition(goalConfigurationHandle,-1,{g[1],g[2],g[3]})
	-- Wait until robot is back to where it started
	waitUntilRobotIsAtGoalPosition()
	
		
    simTubeWrite(actionStatusTube, "done")
    return true

end

-- Starts the recharging procedure. The robot must be at a waypoint adjacent to a recharge station.
-- The robot approaches the recharge station for the recharging to initiate.
startRecharge=function(RechargeStationName)
    -- Before recharging make sure the gripper is at back pickup position for safety
    simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,backPickupPosition,nil)
    setIkMode(false)--Need to do this first
    setFkMode()

    TagRechargerName = RechargeStationName
    TagRechargerName = TagRechargerName:gsub("RechargeStation", "TagRecharger")
    TagRechargerHandle=simGetObjectHandle(TagRechargerName)

    WaypointRechargerName = RechargeStationName
    WaypointRechargerName = WaypointRechargerName:gsub("RechargeStation", "Waypoint_RS")
    WaypointRechargerHandle=simGetObjectHandle(WaypointRechargerName)

    --Check if robot is at the waypoint related to the given recharger
    relative_position=simGetObjectPosition(vehicleReference,WaypointRechargerHandle)
    distance=math.sqrt(relative_position[1]*relative_position[1]+relative_position[2]*relative_position[2]+relative_position[3]*relative_position[3])

    if(distance<=minimum_distance_to_waypoint)then

    -- Adjust position of the robot based on target recharger
        m2,angle2=getBoxAdjustedMatrixAndFacingAngle(TagRechargerHandle)
        simSetObjectPosition(vehicleTarget,-1,{m2[4]-m2[1]*recharge_approach_distance,m2[8]-m2[5]*recharge_approach_distance,0})
        simSetObjectOrientation(vehicleTarget,-1,{0,0,angle2})
        waitToReachVehicleTargetPositionAndOrientation()
        previous_waypoint_handle=WaypointRechargerHandle
    else
        print('Robot is too far from correct waypoint to perform recharging at this recharge station.')
        simTubeWrite(actionStatusTube, "fail")
        return false
    end

end

stopRecharge=function()
    if(previous_waypoint_handle~=nil)then
        previous_waypoint_name=simGetObjectName(previous_waypoint_handle)
        moveToWaypoint(previous_waypoint_name)
    else
        print('Error: The robot can not stop recharging because it does not know its previous waypoint to go back to.')
        simTubeWrite(actionStatusTube, "fail")
        return false
    end
end

-- TODO: Saved for future implementation
getRobotStatus = function()

end

-- TODO: Saved for future implementation
setRobotStatus = function ()

end


end --End of threaded function

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


-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
	simAddStatusbarMessage('Lua runtime error: '..err)
end




-- Put some clean-up code here:
while (simGetSimulationState() ~= sim_simulation_advancing_abouttostop) do
    -- free thread if have comm
    if true then
        cs, cr, cw = simTubeStatus(actionTube)

        if cs ==1 and cr == 1 then
          tubeData = simTubeRead(actionTube)
          print(tubeData)
          command = mysplit(tubeData, ":")

          recharge = {
              ['start'] = function(x)startRecharge(x[3]) end,
              ['stop'] = function(x)stopRecharge() end
            }

            actions = {
              ['move'] = function (x) moveToWaypoint(x[2]) end,
              ['pickup'] = function(x) pickup(x[2]) end,
              ['drop'] = function(x) dropAtPlace(x[2], x[3]) end,
              ['recharge'] = function(x)recharge[x[2]](x) end
            }

            actions[command[1]](command)

        else
            simSwitchThread()
        end
    end
end
