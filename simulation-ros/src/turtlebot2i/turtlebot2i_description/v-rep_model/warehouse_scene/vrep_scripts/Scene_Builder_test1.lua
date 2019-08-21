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


    addModel('dockstation',{-5.25,-4.000,0.063},{0,0,math.pi/2})
    addModel('turtlebot2i',{-4.5,-3.5,0.063},{0,0,math.pi/2}) --Position for testing the performance of the robot
    --addModel('turtlebot2i',{-1.5, 2.0,0.063},{0,0,0}) --Position for testing the mrcnn
    --addModel('turtlebot2i_with_zone',{-1.0, 2.5,0.063},{0,0,0}) --Position for taking screenshoot
    --addModel('Standing_Bill',{ 0.0,  2.0,0},{0,0,math.pi/2})
    --addModel('ConveyorBelt',{1.0,4.0,0.113},{0,0,math.pi/2})
    
    addModel('Shelf_simple',{-4.0, -4.0,0.063},{0,0,math.pi/2}) -- TODO: check why Shelf_simple rotation here is not compatible with what happens in the sim
    

    addModel('ConcreteBox2',{-3.5, 2.0,0.5},{0,0,0})
    addModel('ConcreteBox2',{-5.0,-0.5,0.5},{0,0,0})
    addModel('ConcreteBox2',{-3.5,-2.5,0.5},{0,0,0})

    addModel('ConveyorBelt_simple',{2.5,-2.50,0.113},{0,0,-math.pi/2})
    --addModel('ConveyorBelt_simple',{4.0,-4.00,0.113},{0,0,-math.pi/2})
    

    -- [[
    -- 8 static objects:
    addModel('ConcreteBox',{2.5, 3.5,0.5},{0,0,0})
    addModel('80cmHighPillar100cm',{-0.5, 3.0,0.25},{0,0,0})
    addModel('ConcreteBox1',{ 7.0, 1.5,0.5},{0,0,math.pi/2})
    addModel('ConcreteBox2',{ 6.0,-0.5,0.5},{0,0,0})
    addModel('ConcreteBox2',{ 8.0,-0.5,0.5},{0,0,0})
    addModel('80cmHighPillar100cm',{-0.5, -2.5,0.25},{0,0,0})
    addModel('ConcreteBox1',{ 1.0, -1.5,0.5},{0,0,0})
    addModel('ConcreteBox1',{ 1.0, -3.5,0.5},{0,0,0}) --]]
    
    -- [[
    -- 5 walking humans:
    --addModel('walkingBill_round',{-1.5, 1.0,0},{0,0,math.pi/2})
    --addModel('walkingBill_round',{ 1.0, 4.5,0},{0,0,0})
    addModel('Walking_Bill',{-1.5, 1.0,0},{0,0,math.pi/2})
    addModel('Walking_Bill',{ 1.0, 4.5,0},{0,0,0})
    addModel('Walking_Bill',{ 8.5, -1.5,0},{0,0, math.pi/2})
    addModel('Walking_Bill',{ 5.5,  1.5,0},{0,0, 0})
    addModel('Walking_Bill',{ 0.0, -1.0,0},{0,0, 0})
    addModel('Working_Bill',{ 8.5,  4.0,0},{0,0,math.pi/2}) 
    addModel('Working_Bill',{ 1.0,  1.0,0},{0,0,0})--]]

    -------------------------Wall section bellow-------------------------------

    addModel('80cmHighWall1000cm',{  9.5, 0,.4},{0,0,0})
    addModel('80cmHighWall1000cm',{ -5.5, 0,.4},{0,0,0})--dont have any effect on map
    addModel('80cmHighWall1500cm',{  2.0, 5,.4},{0,0,math.pi/2})
    addModel('80cmHighWall1500cm',{  2.0,-5,.4},{0,0,math.pi/2})


    addModel('80cmHighWall100cm',{ -3.0,  4.5,.4},{0,0,0})
    addModel('80cmHighWall750cm',{ -3.0, -1.25,.4},{0,0,0})
    addModel('80cmHighWall750cm',{  0.75, 0.00,.4},{0,0,math.pi/2})

    addModel('80cmHighWall100cm',{  6.0,  3.5,.4},{0,0,0})
    addModel('80cmHighWall100cm',{  4.5,  4.5,.4},{0,0,0})
    addModel('80cmHighWall500cm',{  4.5,  0.5,.4},{0,0,0})
    addModel('80cmHighWall200cm',{  4.5,  -4.0,.4},{0,0,0})

    addModel('80cmHighWall200cm',{  8.5,  3.0,.4},{0,0,math.pi/2})
    addModel('80cmHighWall200cm',{  5.5,  3.0,.4},{0,0,math.pi/2})
    addModel('80cmHighWall200cm',{  8.5, -2.0,.4},{0,0,math.pi/2})
    addModel('80cmHighWall200cm',{  5.5, -2.0,.4},{0,0,math.pi/2})

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
