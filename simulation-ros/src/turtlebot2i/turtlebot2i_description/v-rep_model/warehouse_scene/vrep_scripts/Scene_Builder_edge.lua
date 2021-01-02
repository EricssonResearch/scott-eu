--**************************
-- Scene builder
-- @author Klaus Raizer
-- @date 2019-01-30
-- PROGRAMATICALLY ADD COMPONENTS TO YOUR SCENE
--**************************

function sysCall_init()
    -- Remote API server
    -- Note that each client must connect to a different port (see https://forum.coppeliarobotics.com/viewtopic.php?t=1009)
    -- For the edge scene, we need one client on the robot and one client on the edge, so 2 clients per robot
    simRemoteApi.start(20000)
    simRemoteApi.start(20001)


    -- Floor
    addModel('Floor10x15m', {2,0,0}, {0,0,0})

    -- Walls
    addModel('80cmHighWall1000cm', {9.5,0,.4}, {0,0,0})
    addModel('80cmHighWall1000cm', {-5.5,0,.4}, {0,0,0})
    addModel('80cmHighWall1500cm', {2.0,5,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall1500cm', {2.0,-5,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall100cm', {-3.0,4.5,.4}, {0,0,0})
    addModel('80cmHighWall750cm', {-3.0,-1.25,.4}, {0,0,0})
    addModel('80cmHighWall750cm', {0.75,0.00,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall100cm', {6.0,3.5,.4}, {0,0,0})
    addModel('80cmHighWall100cm', {4.5,4.5,.4}, {0,0,0})
    addModel('80cmHighWall500cm', {4.5,0.5,.4}, {0,0,0})
    addModel('80cmHighWall200cm', {4.5,-4.0,.4}, {0,0,0})
    addModel('80cmHighWall200cm', {8.5,3.0,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall200cm', {5.5,3.0,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall200cm', {8.5,-2.0,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall200cm', {5.5,-2.0,.4}, {0,0,math.pi/2})

    -- Pillars
    addModel('80cmHighPillar100cm', {-0.5,3.0,0.25}, {0,0,0})
    addModel('80cmHighPillar100cm', {-0.5,-2.5,0.25}, {0,0,0})

    -- Warehouse objects
    addModel('Shelf_simple', {-4.0,-4.0,0.063}, {0,0,math.pi/2})
    addModel('ConveyorBelt_simple', {2.5,-2.50,0.113}, {0,0,-math.pi/2})
    addModel('ConcreteBox2', {-3.5,2.0,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-5.0,-0.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-3.5,-2.5,0.5}, {0,0,0})
    addModel('ConcreteBox', {2.5,3.5,0.5}, {0,0,0})
    addModel('ConcreteBox1', {7.0,1.5,0.5}, {0,0,math.pi/2})
    addModel('ConcreteBox2', {6.0,-0.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {8.0,-0.5,0.5}, {0,0,0})
    addModel('ConcreteBox1', {1.0,-1.5,0.5}, {0,0,0})
    addModel('ConcreteBox1', {1.0,-3.5,0.5}, {0,0,0})

    -- Dock stations (1 for each robot, otherwise there are errors!)
    addModel('dockstation', {-5.25,-4.000,0.063}, {0,0,math.pi/2})

    -- Robots
    addModel('turtlebot2i', {-4.8,-4,0.063}, {0,0,math.pi/2})

end

function addModel(name, position, orientation)
    scenePath = sim.getStringParameter(sim.stringparam_scene_path) -- retrieve scene path
    file = '/vrep_models/'..name..'.ttm'
    objectHandle=sim.loadModel(scenePath..file)
    result=sim.setObjectPosition(objectHandle, -1, position)
    result=sim.setObjectOrientation(objectHandle, -1, orientation)
    return objectHandle
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end
