--**************************
-- Scene builder
-- @author Klaus Raizer
-- @date 2019-01-30
-- PROGRAMATICALLY ADD COMPONENTS TO YOUR SCENE
--**************************

function sysCall_init()
    -- Remote API server
    -- Note that each client must connect to a different port (see https://forum.coppeliarobotics.com/viewtopic.php?t=1009)
    -- For the edge scene, we need 3 clients per robot:
    -- * 1 client on the scene_graph_generator node on the edge
    -- * 1 client on the scene_graph_generator node on the robot
    -- * 1 client on the task_offloading node (on the robot)
    simRemoteApi.start(20000)
    simRemoteApi.start(20001)
    simRemoteApi.start(20002)

    -- Floor
    addModel('Floor15x20m', {0,0,0}, {0,0,0})

    -- Walls
    addModel('80cmHighWall1500cm', {10.0,0,.4}, {0,0,0})
    addModel('80cmHighWall1500cm', {-10.0,0,.4}, {0,0,0})
    addModel('80cmHighWall2000cm', {0.0,7.5,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall2000cm', {0.0,-7.5,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall500cm', {4.0,4.0,.4}, {0,0,0})
    addModel('80cmHighWall500cm', {6.5,1.5,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall500cm', {9.0,3.0,.4}, {0,0,0})
    addModel('80cmHighWall500cm', {5.5,6.5,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall1000cm', {-3.0,2.0,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall1500cm', {-0.5,-1.5,.4}, {0,0,math.pi/2})
    addModel('80cmHighWall500cm', {-5.5,-4.0,.4}, {0,0,0})
    addModel('80cmHighWall500cm', {4.5,-4.0,.4}, {0,0,0})
    addModel('80cmHighWall500cm', {-0.5,-5.0,.4}, {0,0,0})

    -- Pillars
    addModel('80cmHighPillar100cm', {2.0,-4.0,0.35}, {0,0,0})
    addModel('80cmHighPillar100cm', {2.0,-6.5,0.35}, {0,0,0})
    
    -- Warehouse objects
    addModel('ConveyorBelt_simple', {1.0,1.0,0.113}, {0,0,math.pi/2})
    addModel('ConveyorBelt_simple', {-1.0,-0.5,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {-3.0,1.0,0.113}, {0,0,math.pi/2})
    addModel('ConveyorBelt_simple', {-5.0,-0.5,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {-7.0,1.0,0.113}, {0,0,math.pi/2})
    addModel('ConveyorBelt_simple', {-4.5,-6.0,0.113}, {0,0,math.pi})
    addModel('ConveyorBelt_simple', {0.0,4.5,0.113}, {0,0,0})
    addModel('ConveyorBelt_simple', {-9.0,5.5,0.113}, {0,0,math.pi})
    addModel('ConveyorBelt_simple', {-9.0,-3.0,0.113}, {0,0,math.pi})
    addModel('ConveyorBelt_simple', {-4.0,5.0,0.113}, {0,0,math.pi/2})
    addModel('ConveyorBelt_simple', {-7.0,3.0,0.113}, {0,0,-math.pi/2})
    addModel('ConcreteBox1', {-2.0,-3.0,0.5}, {0,0,math.pi/2})
    addModel('ConcreteBox2', {1.0,7.0,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-3.0,-4.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {5.5,0.0,0.5}, {0,0,0})
    addModel('ConcreteBox2', {5.5,-4.0,0.5}, {0,0,0})
    addModel('ConcreteBox2', {7.0,-2.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {7.25,-4.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {9.0,-4.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {7.5,-6.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-7.0,-6.0,0.5}, {0,0,0})

    -- Laptop
    addModel('laptop', {7.0,4.5,0.5}, {-1,0,-math.pi/2})

    -- Dock stations (1 for each robot,otherwise there are errors!)
    addModel('dockstation', {-6.5,-2.0,0.063}, {0,0,0})

    -- Robots
    addModel('turtlebot2i', {0.0,0.0,0.063}, {0,0,0})

    -- Humans (after robots, otherwise the robots are spawn with name not starting from 0)
--     addModel('Walking_Bill', {3.0,-2.5,0}, {0,0,-math.pi/2})
--     addModel('Walking_Bill', {-6.5,6.5,0}, {0,0,-math.pi/2})
--     addModel('Walking_Bill', {0.0,3.0,0}, {0,0,math.pi/2})
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
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end
