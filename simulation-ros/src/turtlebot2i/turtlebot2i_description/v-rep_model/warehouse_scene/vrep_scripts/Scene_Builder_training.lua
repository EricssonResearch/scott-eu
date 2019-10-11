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
    addModel('Floor15x20m',{0,0,0},{0,0,0})

    --Wall:
    addModel('80cmHighWall1500cm',{ 10.0, 0,.4},{0,0,0}) --right
    addModel('80cmHighWall1500cm',{-10.0, 0,.4},{0,0,0}) --left
    addModel('80cmHighWall2000cm',{  0.0, 7.5,.4},{0,0,math.pi/2}) --up
    addModel('80cmHighWall2000cm',{  0.0,-7.5,.4},{0,0,math.pi/2}) --down

    --addModel('80cmHighWall750cm',{ -4.25, 6.5,.4},{0,0,math.pi/2})
    --addModel('80cmHighWall500cm',{ -0.50, 6.5,.4},{0,0,0})
    --addModel('80cmHighWall750cm',{ -2.75, 9.0,.4},{0,0,math.pi/2})

    addModel('80cmHighWall500cm',{  4.0, 4.0,.4},{0,0,0})
    addModel('80cmHighWall500cm',{  6.5, 1.5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall500cm',{  9.0, 3.0,.4},{0,0,0})
    addModel('80cmHighWall500cm',{  5.5, 6.5,.4},{0,0,math.pi/2})

    addModel('80cmHighWall1000cm',{-3.0, 2.0,.4},{0,0,math.pi/2})
    addModel('80cmHighWall1500cm',{-0.5,-1.5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall500cm',{ -5.5,-4.0,.4},{0,0,0})
    addModel('80cmHighWall500cm',{  4.5,-4.0,.4},{0,0,0})
    addModel('80cmHighWall500cm',{ -0.5,-5.0,.4},{0,0,0})


    --addModel('turtlebot2i',{-6.5, 4.75,0.063},{0,0,-math.pi/2}) 
    --addModel('turtlebot2i',{ 0.0,  0.0,0.063},{0,0,0}) 
    --addModel('turtlebot2i_for_training',{ 0.0,  0.0,0.063},{0,0,0}) 
    addModel('turtlebot2i_with_zone',{ 0.0,  0.0,0.063},{0,0,0}) 
    addModel('dockstation',{-6.5, -2.0,0.063},{0,0,0})
    --

    --addModel('80cmHighPillar100cm_pattern',{-4.0,-3.2,0.4},{0,0,math.pi/2}) --TODO: check why the robot is moving by itself
    --addModel('turtlebot2i_simple',{-4.5,-4.0,0.063},{0,0,math.pi/2}) --TODO: check why the robot is moving by itself

    --addModel('Shelf_simple',{-3.5, 0.96485,0.063},{0,0,math.pi/2}) -- TODO: check why Shelf_simple rotation here is not compatible with what happens in the sim
    --addModel('Shelf_simple',{-3.5,-0.53515,0.063},{0,0,math.pi/2})
    --addModel('Shelf_simple',{-3.5,-2.03510,0.063},{0,0,math.pi/2})

    --
    addModel('ConveyorBelt_simple',{ 1.0, 1.0,0.113},{0,0,math.pi/2})
    addModel('ConveyorBelt_simple',{-1.0,-0.5,0.113},{0,0,-math.pi/2})
    addModel('ConveyorBelt_simple',{-3.0, 1.0,0.113},{0,0,math.pi/2})
    addModel('ConveyorBelt_simple',{-5.0,-0.5,0.113},{0,0,-math.pi/2})
    addModel('ConveyorBelt_simple',{-7.0,1.0,0.113},{0,0,math.pi/2})
    


    -- static objects:
    addModel('80cmHighPillar100cm',{2.0,-4.0,0.35},{0,0,0})
    addModel('80cmHighPillar100cm',{2.0,-6.5,0.35},{0,0,0})

    addModel('ConcreteBox',{ 7.0, 4.5,0.5},{0,0,0})
    addModel('ConcreteBox2',{ 1.0, 7.0,0.5},{0,0,0})
    addModel('ConcreteBox1',{-2.0,-3.0,0.5},{0,0,math.pi/2})
    addModel('ConcreteBox2',{-3.0,-4.5,0.5},{0,0,0})

    addModel('ConveyorBelt_simple',{-4.5,-6.0,0.113},{0,0,math.pi})
    addModel('ConveyorBelt_simple',{ 0.0,4.5,0.113},{0,0,0})
    addModel('ConveyorBelt_simple',{-9.0,5.5,0.113},{0,0,math.pi})
    addModel('ConveyorBelt_simple',{-9.0,-3.0,0.113},{0,0,math.pi})
    addModel('ConveyorBelt_simple',{-4.0, 5.0,0.113},{0,0,math.pi/2})
    addModel('ConveyorBelt_simple',{-7.0, 3.0,0.113},{0,0,-math.pi/2})

    addModel('ConcreteBox2',{ 5.5, 0.0,0.5},{0,0,0})
    addModel('ConcreteBox2',{ 5.5,-4.0,0.5},{0,0,0})
    addModel('ConcreteBox2',{ 7.0,-2.5,0.5},{0,0,0})
    addModel('ConcreteBox2',{ 7.25,-4.5,0.5},{0,0,0})
    addModel('ConcreteBox2',{ 9.0,-4.5,0.5},{0,0,0})
    addModel('ConcreteBox2',{ 7.5,-6.5,0.5},{0,0,0})
    addModel('ConcreteBox2',{ -7.0,-6.0,0.5},{0,0,0})
    
    -- dynamic objects:

    --addModel('walkingBill_round',{ 3.0, -2.5,0},{0,0,-math.pi/2})
    addModel('Walking_Bill',{ 3.0, -2.5,0},{0,0,-math.pi/2})
    addModel('Walking_Bill',{-6.5, 6.5,0},{0,0,-math.pi/2})
    addModel('Walking_Bill',{ 0.0, 3.0,0},{0,0, math.pi/2})
    --addModel('Walking_Bill',{ -4, 8.5,0},{0,0,-math.pi/2})
    --addModel('Working_Bill',{3.0,6.0,0},{0,0,math.pi/2})

    --addModel('80cmHighWall750cm',{  1.25, 0.00,.4},{0,0,math.pi/2})
    --addModel('80cmHighWall500cm',{  5.0,  0.50,.4},{0,0,0})
    
    --addModel('turtlebot2i',{ 7.5, 3.0,0.063},{0,0,0}) --just to check the finish location

    --[[Possible target point:
    addModel('Waypoint',{ -9.0, 6.5,.1},{0,0,0})
    addModel('Waypoint',{ -9.0, 3.0,.1},{0,0,0})
    --addModel('Waypoint',{ -6.5, 4.75,.1},{0,0,0})
    addModel('Waypoint',{ -4.0, 6.5,.1},{0,0,0})
    addModel('Waypoint',{ -4.0, 3.0,.1},{0,0,0})
    addModel('Waypoint',{ -0.5, 6.0,.1},{0,0,0})
    addModel('Waypoint',{  1.0, 3.0,.1},{0,0,0})

    addModel('Waypoint',{  5.0, 2.5,.1},{0,0,0})
    addModel('Waypoint',{  3.0, 0.0,.1},{0,0,0})
    addModel('Waypoint',{ -8.5, 0.0,.1},{0,0,0})
    addModel('Waypoint',{ -0.5,-2.0,.1},{0,0,0})

    addModel('Waypoint',{  9.0,-6.5,.1},{0,0,0})
    addModel('Waypoint',{  5.0,-2.0,.1},{0,0,0})
    addModel('Waypoint',{ -4.5,-2.5,.1},{0,0,0})
    addModel('Waypoint',{  8.5,-0.5,.1},{0,0,0})
    addModel('Waypoint',{ -9.0,-6.5,.1},{0,0,0}) --]]
    
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
