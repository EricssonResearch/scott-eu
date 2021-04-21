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

    -- Floor
    addModel('Floor50x50m', {-10,-5,0}, {0,0,0})

    -- Boundaries
    addModel('240cmHighWall3000cm', {0,-20,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall3000cm', {0,20,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall4000cm', {15,0,1.2}, {0,0,0})
    addModel('240cmHighWall4000cm', {-15,0,1.2}, {0,0,0})

    -- Rooms in row 4 (office is in {3,4})
    addModel('240cmHighWall750cm', {-5,13.75,1.2}, {0,0,0})
    addModel('240cmHighWall910cm', {5,14.55,1.2}, {0,0,0})
    addModel('ClosedDoor', {5,19.55,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {-8.75,10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {-1.25,10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall1000cm', {10,10,1.2}, {0,0,math.pi/2})

    -- Rooms in row 3 (dockstations are in {3,3})
    addModel('240cmHighWall1000cm', {-5,5,1.2}, {0,0,0})
    addModel('240cmHighWall1000cm', {5,5,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {-11.25,0,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall1000cm', {0,0,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {11.25,0,1.2}, {0,0,math.pi/2})

    -- Rooms in row 2
    addModel('240cmHighWall1000cm', {-5,-5,1.2}, {0,0,0})
    addModel('240cmHighWall1000cm', {5,-5,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {-8.75,-10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {1.25,-10,1.2}, {0,0,math.pi/2})
    addModel('240cmHighWall750cm', {8.75,-10,1.2}, {0,0,math.pi/2})

    -- Rooms in row 1
    addModel('240cmHighWall750cm', {-5,-16.25,1.2}, {0,0,0})
    addModel('240cmHighWall750cm', {5,-16.25,1.2}, {0,0,0})

    -- Laptop (MEC server)
    addModel('Laptop', {10,15,0.8274}, {-1,0,-math.pi/2})
    addModel('Table', {10,15,0.6476}, {0,0,0})

    -- Conveyor belts
    addModel('ConveyorBelt_simple', {-10,-19,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {0,-19,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {10,-19,0.113}, {0,0,-math.pi/2})

    -- Shelves
    addModel('Shelf', {-7,12,0.066}, {0,0,0})
    addModel('Shelf', {-3,12,0.066}, {0,0,0})
    addModel('Shelf', {-3,2,0.066}, {0,0,0})
    addModel('Shelf', {-13,2,0.066}, {0,0,0})
    addModel('Shelf', {13,2,0.066}, {0,0,0})
    addModel('Shelf', {3,-2,0.066}, {0,0,math.pi})
    addModel('Shelf', {13,-2,0.066}, {0,0,math.pi})
    addModel('Shelf', {-13,-2,0.066}, {0,0,math.pi})

    -- Other static obstacles
    addModel('240cmHighPillar100cm', {-10,15,1.2}, {0,0,0})
    addModel('240cmHighPillar100cm', {0,5,1.2}, {0,0,0})
    addModel('240cmHighPillar100cm', {10,-5,1.2}, {0,0,0})
    addModel('240cmHighPillar100cm', {-10,-5,1.2}, {0,0,0})
    addModel('240cmHighPillar100cm', {0,-15,1.2}, {0,0,0})
    addModel('ConcreteBox2', {8.2,-4.425,0.5}, {0,0,0})
    addModel('ConcreteBox2', {12.35,-5.2,0.5}, {0,0,0})
    addModel('ConcreteBox3', {7.275,-6.975,0.5}, {0,0,0})
    addModel('ConcreteBox3', {13.875,-11.475,0.5}, {0,0,0})
    addModel('ConcreteBox', {11.075,-10.6,0.5}, {0,0,0})
    addModel('ConcreteBox', {10.55,-13.15,0.5}, {0,0,0})
    addModel('ConcreteBox', {-1.225,-12.85,0.5}, {0,0,0})
    addModel('ConcreteBox', {-3.3,-10.725,0.5}, {0,0,0})
    addModel('ConcreteBox', {-0.275,-6,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-2.9,-4.275,0.5}, {0,0,0})
    addModel('ConcreteBox2', {1.075,-4.275,0.5}, {0,0,0})
    addModel('ConcreteBox', {-9.1,-10.8,0.5}, {0,0,0})
    addModel('ConcreteBox', {-11.075,-13.175,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-13.475,-13.7,0.5}, {0,0,0})
    addModel('ConcreteBox', {-11.9,-6.8,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-8.175,-3.9,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-6.225,-4.025,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-6.775,-2.325,0.5}, {0,0,0})
    addModel('ConcreteBox', {-9.9,1.65,0.5}, {0,0,0})
    addModel('ConcreteBox', {-7.35,3.7,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-9.9,3.8,0.5}, {0,0,0})
    addModel('ConcreteBox', {-14.15,7.95,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-11.925,8.9,0.5}, {0,0,0})
    addModel('ConcreteBox1', {-12.825,10.7,0.5}, {0,0,0})
    addModel('ConcreteBox', {-9.975,18.15,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-6.2,15.85,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-7.425,17.5,0.5}, {0,0,0})
    addModel('ConcreteBox', {-2.7,19,0.5}, {0,0,0})
    addModel('ConcreteBox', {-2.45,16.45,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-0.45,18.1,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-4,14.25,0.5}, {0,0,0})
    addModel('ConcreteBox', {4.15,8.575,0.5}, {0,0,0})
    addModel('ConcreteBox3', {2.45,6.85,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-1.575,7.025,0.5}, {0,0,0})

    -- Dock stations (at least 1 for each robot, otherwise there are errors!)
    addModel('dockstation', {7,8,0.052}, {0,0,0})
    addModel('dockstation', {9,8,0.052}, {0,0,0})
    addModel('dockstation', {11,8,0.052}, {0,0,0})
    addModel('dockstation', {13,8,0.052}, {0,0,0})

    -- Robots
    addModel('turtlebot2i_edge', {7,7,0.063}, {0,0,-math.pi/2})
--     addModel('turtlebot2i_edge', {9,7,0.063}, {0,0,-math.pi/2})
--     addModel('turtlebot2i_edge', {11,7,0.063}, {0,0,-math.pi/2})
--     addModel('turtlebot2i_edge', {13,7,0.063}, {0,0,-math.pi/2})

    -- Humans (after robots, otherwise the robots are spawn with name not starting from 0)
    addModel('Walking_Bill', {-10,15,0}, {0,0,math.pi})
    addModel('Walking_Bill', {0,15,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-10,5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {10,5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {0,-5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-10,-15,0}, {0,0,math.pi})
    addModel('Walking_Bill', {10,-15,0}, {0,0,math.pi})
    addModel('Standing_Bill', {7.575,-11.075,0}, {0,0,0})
    addModel('Standing_Bill', {-3.65,-8.55,0}, {0,0,0})
    addModel('Standing_Bill', {-13.075,-10.95,0}, {0,0,0})
    addModel('Standing_Bill', {-14.125,12.325,0}, {0,0,0})
    addModel('Standing_Bill', {0.825,8.075,0}, {0,0,0})

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
