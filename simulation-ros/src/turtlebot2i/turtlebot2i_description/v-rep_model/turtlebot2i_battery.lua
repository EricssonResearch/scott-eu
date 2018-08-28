-- Returns a list with the handles of all the recharge stations in the scene

if (sim_call_type==sim.syscb_init) then

    --robot_id = sim.getStringSignal('robot_id')
    robot_name = sim.getStringSignal('robot_name')

    battery_initial_level=sim.getScriptSimulationParameter(sim.handle_self,'batteryInitialLevel')  -- Initial battery value
    working_time=sim.getScriptSimulationParameter(sim.handle_self,'workingTime')  -- Time the battery lasts in seconds of simulation time
    discharge_constant=-battery_initial_level/working_time
    battery_level=tonumber(battery_initial_level)         --Battery current level

    time_to_recharge=sim.getScriptSimulationParameter(sim.handle_self,'timeToRecharge')  -- The time it takes to recharche a baterry from 0% to 100%
    recharge_constant=battery_initial_level/time_to_recharge

    -- http://www.coppeliarobotics.com/helpFiles/en/accessingGeneralObjects.htm
    batteryObject=sim.getObjectHandle('battery')   
    result=sim.setShapeColor(batteryObject,nil,0,{0,1,0})

    minimumDistanceForRecharging=0.35 --If the robot is farther than this from the recharger, it won't recharge
    --print('minimumDistanceForRecharging: ',minimumDistanceForRecharging)

    charger_state = sim.getIntegerSignal(robot_name .. '_charger_state')
    --print('charger_state: '..charger_state)

    previous_timestamp = sim.getSimulationTime()

end


if (sim_call_type==sim.syscb_actuation) then
    -- TODO: should we stop the robot when battery level reaches zero?
    -- TODO: should we take into account also the orientation between recharger and robot?

    charger_state = sim.getIntegerSignal(robot_name .. '_charger_state')

    if(battery_level~=nil)then
        current_timestamp = sim.getSimulationTime()
        dt = current_timestamp - previous_timestamp

        if (charger_state == 0) then -- Battery is discharging
            if (battery_level > 0) then
                battery_level = battery_level + discharge_constant * dt
            else
                battery_level = 0
            end
        elseif (charger_state == 6) then -- Battery is charging
            if (battery_level < battery_initial_level) then
                battery_level = battery_level + recharge_constant * dt
            else
                battery_level = 100
                sim.setIntegerSignal(robot_name .. '_charger_state', 2)  -- DOCKING_CHARGED  
            end
        end

        --print('battery_level: '..battery_level)
        --print('charger_state: '..charger_state)
        previous_timestamp = current_timestamp

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

        result=sim.setShapeColor(batteryObject,nil,0,{r,g,0})
    end

end

if (sim_call_type==sim.syscb_sensing) then
	-- Put your main SENSING code here
end


if (sim_call_type==sim.syscb_cleanup) then
	-- Put some restoration code here
end

--Returns Battery  parameters
getBatteryProperties = function()
    --battery_initial_level=tostring(sim.getScriptSimulationParameter(sim.handle_self,'batteryInitialLevel'))  -- Initial battery value
    working_time=tostring(sim.getScriptSimulationParameter(sim.handle_self,'workingTime'))  -- Time the battery lasts in seconds of simulation time    
    time_to_recharge=tostring(sim.getScriptSimulationParameter(sim.handle_self,'timeToRecharge'))  -- The time it takes to recharche a baterry from 0% to 100%
 
    return {1}, {}, {working_time, time_to_recharge}, ""
end

--Sets Battery  parameters
setBatteryProperties = function(inInts, inFloats, inStrings, inBuffer)
    sim.setScriptSimulationParameter(sim.handle_self,'workingTime',inInts[1])--working_time)
    working_time = inInts[1]
    sim.setScriptSimulationParameter(sim.handle_self,'timeToRecharge',inInts[2])--time_to_recharge)
    time_to_discharge = inInts[2]
    discharge_constant=-battery_initial_level/working_time
    recharge_constant=battery_initial_level/time_to_recharge
    return {1}, {}, {}, ""
end
--Returns Battery status
getBatteryLevel = function(inInts, inFloats, inStrings, inBuffer)
    return {1, battery_level}, {}, {}, ""
end



