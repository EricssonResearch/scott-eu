--**************************
-- Scene builder
-- @author Klaus Raizer
-- @date 2019-01-30
-- PROGRAMATICALLY ADD COMPONENTS TO YOUR SCENE
--**************************

function sysCall_init()
    -- Fast simulation (V-REP does not render but you can still use rviz)
    -- Note: if you click on the V-REP screen, V-REP starts to render
    sim.setBoolParameter(sim.boolparam_display_enabled,false)

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
    addModel('Floor50x50m', {-5,-5,0}, {0,0,0})

    -- Boundaries
    addModel('240cmHighWall4000cm', {0,-20,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall4000cm', {0,20,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall4000cm', {20,0,1.2}, {0,0,0})
    addModel('240cmHighWall4000cm', {-20,0,1.2}, {0,0,0})

    -- Rooms {row=1} (office is in {row=1, col=4})
    addModel('240cmHighWall750cm', {-10,13.75,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {0,13.75,1.2}, {0,0,0})
    addModel('240cmHighWall910cm', {10,14.55,1.2}, {0,0,0})
    addModel('ClosedDoor', {10,19.55,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {-13.75,10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {-6.25,10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {3.75,10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall1000cm', {15,10,1.2}, {0,0,math.pi/2})

    -- Rooms {row=2} (dockstations are in {row=1, col=4})
    addModel('240cmHighWall1000cm', {-10,5,1.2}, {0,0,0})
    addModel('240cmHighWall1000cm', {0,5,1.2}, {0,0,0})
    addModel('240cmHighWall1000cm', {10,5,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {-16.25,0,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall1000cm', {-5,0,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall1000cm', {5,0,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {13.75,0,1.2}, {0,0,math.pi/2})

    -- Rooms {row=3}
    addModel('240cmHighWall1000cm', {-10,-5,1.2}, {0,0,0})
    addModel('240cmHighWall1000cm', {0,-5,1.2}, {0,0,0})
    addModel('240cmHighWall1000cm', {10,-5,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {-13.75,-10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {-6.25,-10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {3.75,-10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {13.75,-10,1.2}, {0,0,math.pi/2})

    -- Rooms {row=4}
    addModel('240cmHighWall750cm', {-10,-16.25,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {0,-16.25,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {10,-16.25,1.2}, {0,0,0})

    -- Laptop (edge server)
    addModel('Laptop', {15,15,0.8274}, {-1,0,-math.pi/2})
    addModel('Table', {15,15,0.6476}, {0,0,0})

    -- Conveyor belts
    addModel('ConveyorBelt_simple', {-15,-19,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {-5,-19,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {5,-19,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {15,-19,0.113}, {0,0,-math.pi/2})

    -- Shelves
    addModel('Shelf', {-12,8,0.066}, {0,0,math.pi})
    addModel('Shelf', {-8,12,0.066}, {0,0,0})
    addModel('Shelf', {-12,12,0.066}, {0,0,0})
    addModel('Shelf', {-18,-2,0.066}, {0,0,math.pi})
    addModel('Shelf', {2,2,0.066}, {0,0,0})
    addModel('Shelf', {-8,2,0.066}, {0,0,0})
    addModel('Shelf', {2,-2,0.066}, {0,0,math.pi})
    addModel('Shelf', {12,-2,0.066}, {0,0,math.pi})
    addModel('Shelf', {-8,-2,0.066}, {0,0,math.pi})

    -- Other static obstacles
    addModel('ConcreteBox', {16,-3.5,0.5}, {0,0,0})
    addModel('ConcreteBox', {13.5,-13,0.5}, {0,0,0})
    addModel('ConcreteBox', {3.8,-6.45,0.5}, {0,0,0})
    addModel('ConcreteBox', {2.75,-12.6,0.5}, {0,0,0})
    addModel('ConcreteBox', {4.7,2.45,0.5}, {0,0,0})
    addModel('ConcreteBox', {5.5,16.9,0.5}, {0,0,0})
    addModel('ConcreteBox', {-3.7,5.2,0.5}, {0,0,0})
    addModel('ConcreteBox', {-17.2,-12.6,0.5}, {0,0,0})
    addModel('ConcreteBox', {-13.7,-4.8,0.5}, {0,0,0})
    addModel('ConcreteBox', {-16.1,6.1,0.5}, {0,0,0})
    addModel('ConcreteBox2', {17.9,-7.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {8.3,-3.6,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-2.3,-11.5,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-3.4,12,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-13.2,-13.9,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-17.5,12.8,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-13.25,16.8,0.5}, {0,0,0})

    -- Dock stations (at least 1 for each robot, otherwise there are errors!)
    addModel('dockstation', {12,8,0.052}, {0,0,0})
    addModel('dockstation', {14,8,0.052}, {0,0,0})
    addModel('dockstation', {16,8,0.052}, {0,0,0})
    addModel('dockstation', {18,8,0.052}, {0,0,0})

    -- Robots
    addModel('turtlebot2i_edge_training', {12,7,0.063}, {0,0,-math.pi/2})
--     addModel('turtlebot2i_edge_training', {14,7,0.063}, {0,0,-math.pi/2})
--     addModel('turtlebot2i_edge_training', {16,7,0.063}, {0,0,-math.pi/2})
--     addModel('turtlebot2i_edge_training', {18,7,0.063}, {0,0,-math.pi/2})

    -- Humans (after robots, otherwise the robots are spawn with name not starting from 0)
    addModel('Walking_Bill', {15,-15,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-5,-15,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-15,-5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {5,-5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-5,5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-15,15,0}, {0,0,math.pi})
    addModel('Walking_Bill', {5,15,0}, {0,0,math.pi})

    -- Adjust camera
    cameraHandle = sim.getObjectHandle('DefaultCamera')
    sim.setObjectPosition(cameraHandle, -1, {5,0,68})
    sim.setObjectOrientation(cameraHandle, -1, {math.pi,0,math.pi})
end

function addModel(name,position,orientation)
    scenePath = sim.getStringParameter(sim.stringparam_scene_path)  -- retrieve scene path
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
