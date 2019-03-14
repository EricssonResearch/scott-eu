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

    addModel('Floor10x15m',{2,0,0},{0,0,0})
    --addModel('Floor10x10m',{2,0,0},{0,0,0})
    --addModel('Floor10x10m',{10,0,0},{0,0,0})

    addModel('Shelf',{-4.5, 0.96485,0.063},{0,0,-math.pi/2}) -- TODO: check why shelf rotation here is not compatible with what happens in the sim
    addModel('Shelf',{-4.5,-0.53515,0.063},{0,0,-math.pi/2})
    addModel('Shelf',{-4.5,-2.0351,0.063},{0,0,-math.pi/2})


    --addModel('ConveyorBelt',{5.8750,2.4649,0.113},{0,0,0}) -- TODO: check why we need to add these values in z
    addModel('ConveyorBelt',{4, 3.500,0.113},{0,0,math.pi})
    addModel('ConveyorBelt',{4, 1.000,0.113},{0,0,math.pi})
    addModel('ConveyorBelt',{4,-1.500,0.113},{0,0,math.pi})
    addModel('ConveyorBelt',{8.5, 1.000,0.113},{0,0,0})
    addModel('ConveyorBelt',{8.5,-1.500,0.113},{0,0,0})
    addModel('ConveyorBelt',{8.5,-3.500,0.113},{0,0,0})
    

    addModel('dockstation',{-5.0,-4.000,0.063},{0,0,math.pi/2})

    -- 2 static objects:
    addModel('sofa',{0.5, 0.0,0.25},{0,-math.pi/2,0})
    addModel('sofa',{2.5,-2.0,0.25},{0,-math.pi/2,0})
    
    -- 2 walking humans:
    --addModel('walkingBill_round',{ 0.0, 1.5,0},{0,0,math.pi/2})
    --addModel('walkingBill_round',{-0.5,-0.5,0},{0,0,0})

    --addModel('Walking_Bill',{4,4,0},{0,0,math.pi/2})
    --addModel('Working_Bill',{2.0,0.0,0},{0,0,math.pi/2})
    --addModel('indoorPlant',{2.5,-2.0,0},{0,0,0})
    --addModel('ConcreteBox',{2.0,-3.0,0.25},{0,0,0})

    addModel('80cmHighWall1000cm',{  9.5, 0,.4},{0,0,0})
    addModel('80cmHighWall1000cm',{ -5.5, 0,.4},{0,0,0})--dont have any effect on map
    addModel('80cmHighWall1000cm',{ -0.5, 5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall1000cm',{ -0.5,-5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall250cm', { 5.75,-5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall250cm', { 8.25,-5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall250cm', { 5.75, 5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall250cm', { 8.25, 5,.4},{0,0,math.pi/2})

    addModel('80cmHighWall250cm', {-2.0,  1.25,.4},{0,0,0})
    addModel('80cmHighWall250cm', {-2.0, -1.25,.4},{0,0,0})
    addModel('80cmHighWall250cm', {-2.0, -3.75,.4},{0,0,0})

    addModel('80cmHighWall250cm', { 3.0,  3.75,.4},{0,0,0})
    addModel('80cmHighWall250cm', { 3.0,  1.25,.4},{0,0,0})
    addModel('80cmHighWall250cm', { 3.0, -1.25,.4},{0,0,0})

    addModel('turtlebot2i',{-3.5,-4.0,0.063},{0,0,math.pi/2}) --TODO: check why the robot is moving by itself
    --addModel('turtlebot2i',{ 7.5, 3.0,0.063},{0,0,0}) --just to check the finish location
    --addModel('Walking_Bill',{3,-3.0,0},{0,0,0})
    --addModel('youbot',{2.0,-3.33515,0.063},{0,0,-math.pi/2})
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
