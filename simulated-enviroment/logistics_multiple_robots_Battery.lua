--**************************
-- YouBot Kuka Robot's battery
-- @author Klaus Raizer
-- @date 24-02-2017
--
-- Description: Controls the battery of the YouBot Kuka Robot
--**************************

-- Returns a list with the handles of all the recharge stations in the scene
getListOfRechargeStations = function ()

    objects = simGetObjectsInTree(sim_handle_scene,0,0)
    numberOfObjectsOfTypeShape=table.getn(objects)

    rs=1
    recharge_station_list={}
    for i=1,numberOfObjectsOfTypeShape,1 do
        objectName=simGetObjectName(objects[i])
        if string.match(objectName, "RechargeStation") then
            recharge_station_list[rs]=objects[i]
            rs=rs+1
        end
    end

    return recharge_station_list
end

if (sim_call_type==sim_childscriptcall_initialization) then

    battery_initial_level=simGetScriptSimulationParameter(sim_handle_self,'batteryInitialLevel')  -- Initial battery value
    working_time=simGetScriptSimulationParameter(sim_handle_self,'workingTime')  -- Time the battery lasts in seconds of simulation time
    discharge_constant=-battery_initial_level/working_time
    battery_level=tonumber(battery_initial_level)         --Battery current level

    
    time_to_recharge=simGetScriptSimulationParameter(sim_handle_self,'timeToRecharge')  -- The time it takes to recharche a baterry from 0% to 100%
    recharge_constant=battery_initial_level/time_to_recharge
    
    first_at_recharge=true
    first_at_discharge=true

    -- http://www.coppeliarobotics.com/helpFiles/en/accessingGeneralObjects.htm
    batteryObject=simGetObjectHandle('Battery')   
    result=simSetShapeColor(batteryObject,nil,0,{0,1,0})
    
    vehicleReference=simGetObjectHandle('youBot_vehicleReference')

    minimumDistanceForRecharging=0.35 --If the robot is farther than this from the recharger, it won't recharge
    --print('minimumDistanceForRecharging: ',minimumDistanceForRecharging)
    recharge_station_list=getListOfRechargeStations()
    --print ("recharge_station_list: ",recharge_station_list[1]," , ",recharge_station_list[2]," , ",recharge_station_list[3],", ...")

end


if (sim_call_type==sim_childscriptcall_actuation) then
    -- TODO: should we stop the robot when battery level reaches zero?
    -- TODO: should we take into account also the orientation between recharger and robot?

    -- Find closest recharger
    if(recharge_station_list~=nil)and(recharge_station_list~={})then
        number_of_recharge_stations=table.getn(recharge_station_list)
    else
        number_of_recharge_stations=0
    end
        if(number_of_recharge_stations>0)then-- at least one recharge station in this scene
            closestRsHandle=recharge_station_list[1]
            relativePosition=simGetObjectPosition (closestRsHandle,vehicleReference)
            horizontalDistance=math.sqrt(relativePosition[1]^2+relativePosition[2]^2)
            closestDistance=horizontalDistance
            if(number_of_recharge_stations>1)then
                    for i=2,number_of_recharge_stations,1 do
                        relativePosition=simGetObjectPosition (recharge_station_list[i],vehicleReference)
                        horizontalDistance=math.sqrt(relativePosition[1]^2+relativePosition[2]^2)
                        if(horizontalDistance<closestDistance)then
                            closestDistance=horizontalDistance
                            closestRsHandle=recharge_station_list[i]
                        end
                    end
            end
        end
    
        if(battery_level~=nil)then
            t=simGetSimulationTime()
            --print('battery_level: ',battery_level)

            if((number_of_recharge_stations>0)and(closestDistance<=minimumDistanceForRecharging))then
                
                if(first_at_recharge)then
                    b = battery_level-recharge_constant*t
                    first_at_recharge=false
                    first_at_discharge=true
                end
                tempBat=recharge_constant*t+b
                if(tempBat<100) then
                    battery_level=tempBat
                else
                    battery_level=tonumber(battery_initial_level)
                end
            else
                if(first_at_discharge)then
                    b = battery_level-discharge_constant*t
                    first_at_recharge=true
                    first_at_discharge=false
                end
                tempBat=discharge_constant*t+b
                if(tempBat>0) then
                    battery_level=tempBat
                else
                    battery_level=0
                end
                
            end

            --Change color of battery to visually represent how charged it is
            if((battery_level)<(battery_initial_level/2))then
                g = (2/battery_initial_level)*battery_level
            else
                g=1
            end

            if(battery_level>(battery_initial_level/2))then
                r = -2*battery_level/battery_initial_level + 2
            else
                r = 1
            end

            result=simSetShapeColor(batteryObject,nil,0,{r,g,0})
        end

end

if (sim_call_type==sim_childscriptcall_sensing) then
	-- Put your main SENSING code here
end


if (sim_call_type==sim_childscriptcall_cleanup) then
	-- Put some restoration code here
end

--Returns Battery  parameters
getBatteryProperties = function()
    --battery_initial_level=tostring(simGetScriptSimulationParameter(sim_handle_self,'batteryInitialLevel'))  -- Initial battery value
    working_time=tostring(simGetScriptSimulationParameter(sim_handle_self,'workingTime'))  -- Time the battery lasts in seconds of simulation time    
    time_to_recharge=tostring(simGetScriptSimulationParameter(sim_handle_self,'timeToRecharge'))  -- The time it takes to recharche a baterry from 0% to 100%
 
    return {1}, {}, {working_time, time_to_recharge}, ""
end

--Sets Battery  parameters
setBatteryProperties = function(inInts, inFloats, inStrings, inBuffer)
    simSetScriptSimulationParameter(sim_handle_self,'workingTime',inInts[1])--working_time)
    working_time = inInts[1]
    simSetScriptSimulationParameter(sim_handle_self,'timeToRecharge',inInts[2])--time_to_recharge)
    time_to_discharge = inInts[2]
    discharge_constant=-battery_initial_level/working_time
    recharge_constant=battery_initial_level/time_to_recharge
    return {1}, {}, {}, ""
end
--Returns Battery status
getBatteryLevel = function(inInts, inFloats, inStrings, inBuffer)
    return {1, battery_level}, {}, {}, ""
end
