--## NOTE: This lua file should be inside the model file!
--**************************
-- Recharge Station
-- @author Klaus Raizer
-- @date 22-06-2016
-- 
-- Description: This script was added so that Recharge Stations's 
--names addhere to the automatic naming standard of adding "# plus number 
--of new object".
--
-- This script calls a lua script located at the same folder as this scene's with the name: logistics_multiple_robots_youBot.lua
-- Enyu Cao edited it to hide the error messages durning intialization.
--**************************

if not __notFirst__ then
    -- Build file name
    --local sceneFullName=sim.getStringParameter(sim.stringparam_scene_name)
    --sceneName = string.gsub(sceneFullName, ".ttt", "")

    robot_name = sim.getStringSignal('robot_name')
    if not robot_name==nil then
        local objectHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
        object_name = sim.getObjectName(objectHandle)

        local name_suffix, name_prefix = sim.getNameSuffix(object_name)
        local file = '/'..robot_name..'_'..name_prefix..'.lua'

        -- Retrieve scene path
        local scenePath = sim.getStringParameter(sim.stringparam_scene_path)
        -- Load script file
        __scriptCodeToRun__ = assert(loadfile(scenePath..file))
        if cmd then
            local tmp = assert(loadstring(cmd))
            if tmp then
                tmp()
            end
        end
    -- Make sure we only run this once
    __notFirst__ = true
    end 

end

if __scriptCodeToRun__ then
    --Actually call the script
    __scriptCodeToRun__()
end
