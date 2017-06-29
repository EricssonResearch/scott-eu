--**************************
-- Path following script for the mobile robot
-- @author Klaus Raizer
-- @date 23-01-2017
--

moveStatusTube=simTubeOpen(1, 'moveStatus'..simGetNameSuffix(nil), 1)

visualizePath=function(path)
    if not _lineContainer then
        _lineContainer=simAddDrawingObject(sim_drawing_lines,3,0,-1,99999,{0.2,0.2,0.2})
    end
    simAddDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/3
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*3+1],path[(i-1)*3+2],initPos[3],path[i*3+1],path[i*3+2],initPos[3]}
            simAddDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

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

threadFunction=function()

 -- print('['..simGetObjectName(startConfigurationHandle)..'] ## Running thread function ##')


    while (simGetSimulationState()~=sim_simulation_advancing_abouttostop) do

		--print('#'..simGetObjectName(startConfigurationHandle)..'#')

        if(STATE==replan)then
           -- print('['..simGetObjectName(startConfigurationHandle)..'] '..'STATE==replan')
            initPos=simGetObjectPosition(startConfigurationHandle,-1)
            initOrient=simGetObjectOrientation(startConfigurationHandle,-1)
            t=simExtOMPL_createTask('t')
            ss={simExtOMPL_createStateSpace('2d',sim_ompl_statespacetype_pose2d,startConfigurationHandle,{-10,-10},{10,10},1)} --TODO: could optimize this
            simExtOMPL_setStateSpace(t,ss)
            simExtOMPL_setAlgorithm(t,sim_ompl_algorithm_RRTConnect)
            --simExtOMPL_setAlgorithm(t,sim_ompl_algorithm_RRTstar) -- better path but too slow

            --simExtOMPL_setCollisionPairs(t,{simGetObjectHandle('youBotStart'),sim_handle_all})
            -- If I use sim_handle_all, it doesn't work because it hits the robot. Needed to create a collection including everything but the robot
			
			--TODO: The collidable colection should not include the products being held in cargo!!
			-- http://www.coppeliarobotics.com/helpFiles/en/regularApi/simCreateCollection.htm
			-- http://www.coppeliarobotics.com/helpFiles/en/regularApi/simAddObjectToCollection.htm
			--http://www.coppeliarobotics.com/helpFiles/en/regularApi/simRemoveCollection.htm
			
			--splitVehicleName=mysplit(vehicleName,'#')
			--print("splitVehicleName [1]: ",splitVehicleName[1])
			--if(#splitVehicleName>1)then
			--print("splitVehicleName [2]: ",splitVehicleName[2])
			--else
			
			--vehicleName=simGetObjectName(vehicleBodyHandle)
			--colidableCollectionName="colidableCollection_"..vehicleName
			--print("colidableCollectionName: ",colidableCollectionName)
			--colidableCollectionHandle=simCreateCollection("",0)
			--numberOptions=0
			--result=simAddObjectToCollection(colidableCollectionHandle,sim_handle_all,sim_handle_all,numberOptions)
			--numberOptions=1
			--result=simAddObjectToCollection(colidableCollectionHandle,vehicleBodyHandle,sim_handle_all,numberOptions)
			------------------
		--[[	-- Set the sim_objectspecialproperty_detectable_all property:
local p=simGetObjectSpecialProperty(objectHandle)
p=simBoolOr32(p,sim_objectspecialproperty_detectable_all)
simSetObjectSpecialProperty(objectHandle,p)


-- Clear the sim_objectspecialproperty_detectable_all property:
local p=simGetObjectSpecialProperty(objectHandle)
p=simBoolOr32(p,sim_objectspecialproperty_detectable_all)-sim_objectspecialproperty_detectable_all
simSetObjectSpecialProperty(objectHandle,p)
			---------------------
			
			]]--
			

			
			
			
			--Inform path planning task of what is considered to be colidable
			simExtOMPL_setCollisionPairs(t,{simGetObjectHandle('youBotStart'),colidableCollectionHandle})
           
			-- Define start and goal positions
            startpos=simGetObjectPosition(startConfigurationHandle,-1)
            startorient=simGetObjectOrientation(startConfigurationHandle,-1)
            startpose={startpos[1],startpos[2],startorient[3]}
            simExtOMPL_setStartState(t,startpose)
            goalpos=simGetObjectPosition(goalConfigurationHandle,-1)
            goalorient=simGetObjectOrientation(goalConfigurationHandle,-1)
            goalpose={goalpos[1],goalpos[2],goalorient[3]}
            simExtOMPL_setGoalState(t,goalpose)

			
			--Check if goal box is interfering with something in the scene, in which case planning won't be possible
			-- http://www.coppeliarobotics.com/helpFiles/en/regularApi/simCheckCollision.htm
			collisionDetection=simCheckCollision(goalHandle,colidableCollectionHandle)
			--print("["..simGetObjectName(vehicleBodyHandle).."] ".."collisionDetection: "..collisionDetection)
			
			if(collisionDetection==1)then
				r=-1 -- 0 for failure, 1 for success and -1 for not performed				
			else
				-- The following command is what actually performs the path planning computation. 			
				r,path=simExtOMPL_compute(t,max_plan_time,-1,0) -- 0 is for standard number of points
			end
			

			--[[count=1
			repeat
				simSwitchThread()
			until (false)
			]]--
			

            --Transition eval
			if(r==0 or collisionDetection==1)then --failure
				print("")
				print("FAILED PLANNING [Robot "..simGetObjectName(vehicleBodyHandle).."] Possible reasons:")
				print("goal place is occupied: ",collisionDetection==1)
				print("couldn't find a path: ",r==0)
				print("... replaning.")
				
				STATE=replan
			else
				STATE=follow
			end

        elseif(STATE==follow)then
           -- print('['..simGetObjectName(startConfigurationHandle)..'] '..'STATE==follow')
            if(path)then --something wrong here. sometimes it thinks there is a path, but there isnt. Then it gets stuck at repeat
                visualizePath(path)
                finalPos={path[#path-2], path[#path-1]} -- x, y final position
                finalOri=path[#path] --  z final orientation
                --print('['..simGetObjectName(startConfigurationHandle)..'] '..'path['..#path..']: ',path[#path])
                -- Simply jump through the path points, no interpolation here:
                for i=1,#path-3,3 do
                    pos={path[i],path[i+1],initPos[3]}
                    orient={initOrient[1],initOrient[2],path[i+2]}
                    repeat
                        pvs = simGetObjectPosition(vehicleBodyHandle,startConfigurationHandle)-- vehicle position compared to current start configuration
                        dvs=math.sqrt(pvs[1]*pvs[1]+pvs[2]*pvs[2])
						--print('['..simGetObjectName(startConfigurationHandle)..'] '..'dvs['..i..']: '..dvs)
                    until(dvs<=min_d)-- should wait to see if the robot is able to keep up
					-- TODO: verify if stuck at repeat

                    simSetObjectPosition(startConfigurationHandle,-1,pos)
                    simSetObjectOrientation(startConfigurationHandle,-1,orient)

                    simSetObjectPosition(vehicleTarget,startConfigurationHandle,{0,0,0})
                    simSetObjectOrientation(vehicleTarget,startConfigurationHandle,{0,0,0})

                    pg=simGetObjectPosition(goalConfigurationHandle,-1)
                    rg=simGetObjectOrientation(goalConfigurationHandle,-1)


                    d=math.sqrt( (finalPos[1]-pg[1])*(finalPos[1]-pg[1]) + (finalPos[2]-pg[2])*(finalPos[2]-pg[2]) )
                    o=math.sqrt(math.pow(finalOri-rg[3],2))*180/math.pi -- angle difference in degrees between goal object and path last stat
                    --print('finalOri: ',finalOri,' rg[3]:',rg[3],' o: ',o)
                   --TODO: should also verify if there are new obstacles on this path
                    if(d>min_d or o>min_o)then -- this verifies if the goal was moved while following the path, in which case we should replan
                        STATE=replan
                        break
                    end

                    simSwitchThread()
                end -- end of for i=1,#path-3,3 do
				-- If #path-3 is not a multiple of 3, then the last position might be left out of the iteration. In which case we set the last posisiton and orientations manually here to be sure.
				pos={finalPos[1], finalPos[2],initPos[3]}
                orient={initOrient[1],initOrient[2],finalOri}
				simSetObjectPosition(startConfigurationHandle,-1,pos)
                simSetObjectOrientation(startConfigurationHandle,-1,orient)
                simSetObjectPosition(vehicleTarget,startConfigurationHandle,{0,0,0})
                simSetObjectOrientation(vehicleTarget,startConfigurationHandle,{0,0,0})

            else
                print('['..simGetObjectName(startConfigurationHandle)..']'..'No path to be followed.')
            end




            rp=simGetObjectPosition(startConfigurationHandle,goalConfigurationHandle)
            --ro=simGetObjectOrientation(startConfigurationHandle,goalConfigurationHandle)
            d=math.sqrt(rp[1]*rp[1]+rp[2]*rp[2])
            --o=ro[3]
           rg=simGetObjectOrientation(goalConfigurationHandle,-1)
           o=math.sqrt(math.pow(finalOri-rg[3],2))*180/math.pi -- angle difference in degrees between goal object and path last stat
           if(d<=min_d and o<=min_o)then --arrived
                STATE=stop
           else
                STATE=replan
            end

            --if(end of path != goal)then
            --    STATE=replan
            --end

        else -- (STATE==stop)
           -- print('['..simGetObjectName(startConfigurationHandle)..'] '..'STATE==stop')
            -- allign position and orientation? (TODO: only once), then wait

			--TODO: Investigate if we realy need to do the following:
			--simSetObjectPosition(startConfigurationHandle,goalConfigurationHandle,{0,0,0})
            --simSetObjectOrientation(startConfigurationHandle,goalConfigurationHandle,{0,0,0})

            rp=simGetObjectPosition(startConfigurationHandle,goalConfigurationHandle)
            ro=simGetObjectOrientation(startConfigurationHandle,goalConfigurationHandle)
            d=math.sqrt(rp[1]*rp[1]+rp[2]*rp[2])
            rg=simGetObjectOrientation(goalConfigurationHandle,-1)
            o=math.sqrt(math.pow(finalOri-rg[3],2))*180/math.pi -- angle difference in degrees between goal object and path last stat

            if(d>min_d or o>min_o)then --arrived
                STATE=replan
            end

        end

        simTubeWrite(moveStatusTube, STATE)
    end -- end of 'while (simGetSimulationState()'

end

function returnCollidableCollection()
	--colidableCollectionHandle=simCreateCollection("",0)
	collectionName="colidableCollection"..simGetObjectName(vehicleBodyHandle)
    colidableCollectionHandle=simGetCollectionHandle(collectionName)
	return colidableCollectionHandle
end


-- Put some initialization code here:
simSetThreadSwitchTiming(2) -- Default timing for automatic thread switching

startConfigurationHandle=simGetObjectHandle('youBotStartConfiguration')
goalConfigurationHandle=simGetObjectHandle('youBotGoalConfiguration')
goalHandle=simGetObjectHandle('youBotGoal')

vehicleTarget=simGetObjectHandle('youBot_vehicleTargetPosition')
vehicleBodyHandle=simGetObjectHandle('mobilerobot_youBot')

simSetObjectParent(startConfigurationHandle,-1,true)
simSetObjectParent(goalConfigurationHandle,-1,true)

min_d = 0.05 -- minimum_distance_from_robot_to_goal
min_o = 5*math.pi/180 -- minimum angle in degrees

max_plan_time=3 -- maximum number of seconds the planning algorithm can use when trying to find a path

stop=0
follow=1
replan=2
STATE=replan

vehicleName=simGetObjectName(vehicleBodyHandle)
colidableCollectionName="colidableCollection_"..vehicleName

--Dynamically create a collection for path planning. See details here: http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=2581&p=26260#p2626	
-- For some reason vrep does not erase collections created during runtime. Therefore, if we don't delete it manually, new collections would accumulate accross many runs.
-- Here we are creating a colletion with a standard name, and checking if it already exists. If it does, it is the remain of a previous run, then I would only need to clean it up before using it.
		
--If error there is no collection with this name yet, must create one
if pcall(returnCollidableCollection)then 
	colidableCollectionHandle=returnCollidableCollection()
else
	colidableCollectionHandle=simCreateCollection(collectionName,0)
end 

result=simEmptyCollection(colidableCollectionHandle)
result=simAddObjectToCollection(colidableCollectionHandle,sim_handle_all,sim_handle_all,0)
result=simAddObjectToCollection(colidableCollectionHandle,vehicleBodyHandle,sim_handle_tree,1)
			
-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
	simAddStatusbarMessage('Lua runtime error: '..err)
end
