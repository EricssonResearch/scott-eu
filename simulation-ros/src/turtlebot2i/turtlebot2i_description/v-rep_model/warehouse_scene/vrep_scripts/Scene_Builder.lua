--**************************
-- Scene builder
-- @author Klaus Raizer
-- @date 2019-01-30
-- PROGRAMATICALLY ADD COMPONENTS TO YOUR SCENE
--**************************

function sysCall_init()
     
	-- Initialization ------------------------
	--sim.setBoolParameter(sim.boolparam_display_enabled,false)-- Disables rendering to increase simulation speed
	simRemoteApi.start(20000) -- Start the remote API
	simRemoteApi.start(20001) -- Start the remote API	
	
	-- Building the scene ------------------------
	
	addModel('Floor10x10m',{2,0,0},{0,0,0})
	--addModel('Floor10x10m',{10,0,0},{0,0,0})
	
	addModel('Shelf',{1.1,0.96485,0.063},{0,0,-math.pi/2}) -- TODO: check why shelf rotation here is not compatible with what happens in the sim
	addModel('Shelf',{1.1,-0.53515,0.063},{0,0,-math.pi/2})
	addModel('Shelf',{1.1,-2.0351,0.063},{0,0,-math.pi/2})
	

	addModel('ConveyorBelt',{4.3750,2.4649,0.113},{0,0,0}) -- TODO: check why we need to add these values in z
	addModel('ConveyorBelt',{4.3750,-0.53515,0.113},{0,0,0})
	addModel('ConveyorBelt',{4.3750,-3.5351,0.113},{0,0,0})
	

	addModel('dockstation',{0.77497,-2.9750,0.063},{0,0,math.pi/2})
	

	--addModel('Walking_Bill',{4,4,0},{0,0,math.pi/2})
	
	
	addModel('80cmHighWall1000cm',{7,0,.4},{0,0,0})
	addModel('80cmHighWall1000cm',{-3,0,.4},{0,0,0})
	addModel('80cmHighWall1000cm',{2,5,.4},{0,0,math.pi/2})
	addModel('80cmHighWall1000cm',{2,-5,.4},{0,0,math.pi/2})
	


	addModel('turtlebot2i',{2.2230,0.96485,0.063},{0,0,math.pi/2}) --TODO: check why the robot is moving by itself
	--addModel('Walking_Bill',{4,0,0},{0,0,0})
	
end

function addModel(name,position,orientation)
	scenePath = sim.getStringParameter(sim.stringparam_scene_path) -- retrieve scene path
	file = '/vrep_models/'..name..'.ttm'
    objectHandle=sim.loadModel(scenePath..file)
	result=sim.setObjectPosition(objectHandle,-1,position)
	result=sim.setObjectOrientation(objectHandle,-1,orientation)
	return objectHandle
end

function sysCall_actuation()
    -- put your actuation code here
    --
    -- For example:
    --
    -- local position=sim.getObjectPosition(handle,-1)
    -- position[1]=position[1]+0.001
    -- sim.setObjectPosition(handle,-1,position)
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end
