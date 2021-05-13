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
    addModel('Floor50x50m', {-10,-10,0}, {0,0,0})

    -- Boundaries
    addModel('80cmHighWall3000cm', {15,0,0.4}, {0,0,0})
    addModel('80cmHighWall3000cm', {-15,0,0.4}, {0,0,0})
    addModel('80cmHighWall3000cm', {0,-15,0.4}, {0,0,math.pi/2})
    addModel('80cmHighWall3000cm', {0,15,0.4}, {0,0,math.pi/2})

    -- Rooms in row 3 (office is in {3,3})
    addModel('80cmHighWall750cm', {-5,8.75,0.4}, {0,0,0})
    addModel('80cmHighWall750cm', {5,8.75,0.4}, {0,0,0})
    addModel('80cmHighWall750cm', {-8.75,5,0.4}, {0,0,math.pi/2})
    addModel('80cmHighWall1000cm', {0,5,0.4}, {0,0,math.pi/2})
    addModel('80cmHighWall750cm', {8.75,5,0.4}, {0,0,math.pi/2})

    -- Rooms in row 2 (dockstations are in {2,3})
    addModel('80cmHighWall1000cm', {-5,0,0.4}, {0,0,0})
    addModel('80cmHighWall1000cm', {5,0,0.4}, {0,0,0})
    addModel('80cmHighWall750cm', {-11.25,-5,0.4}, {0,0,math.pi/2})
    addModel('80cmHighWall750cm', {1.25,-5,0.4}, {0,0,math.pi/2})
    addModel('80cmHighWall750cm', {11.25,-5,0.4}, {0,0,math.pi/2})

    -- Rooms in row 1
    addModel('80cmHighWall750cm', {-5,-11.25,0.4}, {0,0,0})
    addModel('80cmHighWall750cm', {5,-11.25,0.4}, {0,0,0})

    -- Laptop (base station)
    addModel('Laptop', {12.5,12.5,0.8274}, {-1,0,-math.pi/2})
    addModel('Table', {12.5,12.5,0.6476}, {0,0,0})

    -- Conveyor belts
    addModel('ConveyorBelt_simple', {-10,-14,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {0,-14,0.113}, {0,0,-math.pi/2})
    addModel('ConveyorBelt_simple', {10,-14,0.113}, {0,0,-math.pi/2})

    -- Shelves
    addModel('Shelf2', {-7,7,0.066}, {0,0,0})
    addModel('Shelf2', {3,7,0.066}, {0,0,0})
    addModel('Shelf2', {-7,3,0.066}, {0,0,math.pi})
    addModel('Shelf2', {3,3,0.066}, {0,0,math.pi})
    addModel('Shelf2', {7,3,0.066}, {0,0,math.pi})

    -- Other static obstacles
    addModel('80cmHighPillar100cm', {-10,10,0.4}, {0,0,0})
    addModel('80cmHighPillar100cm', {0,0,0.4}, {0,0,0})
    addModel('80cmHighPillar100cm', {10,-10,0.4}, {0,0,0})
    addModel('80cmHighPillar100cm', {-10,-10,0.4}, {0,0,0})
    addModel('80cmHighPillar100cm', {10,10,0.4}, {0,0,0})
    addModel('ConcreteBox2', {7.5,-9.425,0.5}, {0,0,0})
    addModel('ConcreteBox2', {12.35,-10.2,0.5}, {0,0,0})
    addModel('ConcreteBox3', {7.275,-11.975,0.5}, {0,0,0})
    addModel('ConcreteBox3', {13.875,3.525,0.5}, {0,0,0})
    addModel('ConcreteBox', {10.55,4.35,0.5}, {0,0,0})
    addModel('ConcreteBox', {10.55,1.85,0.5}, {0,0,0})
    addModel('ConcreteBox', {-0.275,-7,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-2.9,-9.275,0.5}, {0,0,0})
    addModel('ConcreteBox2', {1.075,-9.275,0.5}, {0,0,0})
    addModel('ConcreteBox', {5.9,-0.8,0.5}, {0,0,0})
    addModel('ConcreteBox', {7.925,-3.175,0.5}, {0,0,0})
    addModel('ConcreteBox2', {5.525,-3.7,0.5}, {0,0,0})
    addModel('ConcreteBox', {-11.9,-11.8,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-8.175,-11.8,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-6.225,-9.025,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-7.775,-7.325,0.5}, {0,0,0})
    addModel('ConcreteBox', {-9.9,-3.35,0.5}, {0,0,0})
    addModel('ConcreteBox', {-7.35,-1.3,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-9.9,0,0.5}, {0,0,0})
    addModel('ConcreteBox', {-14.15,2.95,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-11.5,3.5,0.5}, {0,0,0})
    addModel('ConcreteBox1', {-12.825,5.7,0.5}, {0,0,0})
    addModel('ConcreteBox', {-9.975,13.15,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-6.2,10.85,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-7.425,12.5,0.5}, {0,0,0})
    addModel('ConcreteBox', {-2.7,14,0.5}, {0,0,0})
    addModel('ConcreteBox', {-2.45,11.45,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-0.45,13.1,0.5}, {0,0,0})
    addModel('ConcreteBox2', {-4,9.25,0.5}, {0,0,0})
    addModel('ConcreteBox', {-4,-3,0.5}, {0,0,0})
    addModel('ConcreteBox1', {1.85,-1.725,0.5}, {0,0,0})
    addModel('ConcreteBox3', {-1.575,2.025,0.5}, {0,0,0})

    -- Dock stations (at least 1 for each robot, otherwise there are errors!)
    addModel('dockstation', {7,5.5,0.052}, {0,0,math.pi})
    addModel('dockstation', {9,5.5,0.052}, {0,0,math.pi})

    -- Robots
    addModel('turtlebot2i_edge', {7,7,0.063}, {0,0,math.pi/2})
    addModel('turtlebot2i_edge', {9,7,0.063}, {0,0,math.pi/2})

    -- Humans (after robots, otherwise the robots are spawn with name not starting from 0)
    addModel('Walking_Bill', {-10,12.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {0,12.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {10,12.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-10,2.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {0,2.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {10,2.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {-10,-7.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {0,-7.5,0}, {0,0,math.pi})
    addModel('Walking_Bill', {10,-7.5,0}, {0,0,math.pi})
    addModel('Standing_Bill', {-7,-3.35,0}, {0,0,0})
    addModel('Standing_Bill', {6.575,-6.875,0}, {0,0,0})

    -- Adjust camera
    cameraHandle = sim.getObjectHandle('DefaultCamera')
    sim.setObjectPosition(cameraHandle, -1, {5,0,45})
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
