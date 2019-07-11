function sysCall_init() 
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    legJointHandles={sim.getObjectHandle('Bill_leftLegJoint'),sim.getObjectHandle('Bill_rightLegJoint')}
    kneeJointHandles={sim.getObjectHandle('Bill_leftKneeJoint'),sim.getObjectHandle('Bill_rightKneeJoint')}
    ankleJointHandles={sim.getObjectHandle('Bill_leftAnkleJoint'),sim.getObjectHandle('Bill_rightAnkleJoint')}
    shoulderJointHandles={sim.getObjectHandle('Bill_leftShoulderJoint'),sim.getObjectHandle('Bill_rightShoulderJoint')}
    elbowJointHandles={sim.getObjectHandle('Bill_leftElbowJoint'),sim.getObjectHandle('Bill_rightElbowJoint')}
    neckJoint=sim.getObjectHandle('Bill_neck')
    
    legWaypoints={0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188}
    kneeWaypoints={0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092}
    ankleWaypoints={-0.133,0.041,0.244,0.382,0.304,0.232,0.266,0.061,-0.090,-0.145,-0.043,0.041,0.001,0.011,-0.099,-0.127,-0.121,-0.120,-0.107,-0.100,-0.090,-0.009}
    shoulderWaypoints={0.028,0.043,0.064,0.078,0.091,0.102,0.170,0.245,0.317,0.337,0.402,0.375,0.331,0.262,0.188,0.102,0.094,0.086,0.080,0.051,0.058,0.048}
    elbowWaypoints={-1.148,-1.080,-1.047,-0.654,-0.517,-0.366,-0.242,-0.117,-0.078,-0.058,-0.031,-0.001,-0.009,0.008,-0.108,-0.131,-0.256,-0.547,-0.709,-0.813,-1.014,-1.102}
    relativeVel={2,2,1.2,2.3,1.4,1,1,1,1,1.6,1.9,2.4,2.0,1.9,1.5,1,1,1,1,1,2.3,1.5}
    
    leftSensorClose=sim.getObjectHandle('Bill_proxSensorLeftClose')
    rightSensorClose=sim.getObjectHandle('Bill_proxSensorRightClose')
    leftSensorFar=sim.getObjectHandle('Bill_proxSensorLeftFar')
    rightSensorFar=sim.getObjectHandle('Bill_proxSensorRightFar')
    leftFloorSensorClose=sim.getObjectHandle('Bill_proxSensorFloorLeftClose')
    rightFloorSensorClose=sim.getObjectHandle('Bill_proxSensorFloorRightClose')
    leftFloorSensorFar=sim.getObjectHandle('Bill_proxSensorFloorLeftFar')
    rightFloorSensorFar=sim.getObjectHandle('Bill_proxSensorFloorRightFar')
    
    nominalVelocity=sim.getScriptSimulationParameter(sim.handle_self,'walkingSpeed')
    randomColors=sim.getScriptSimulationParameter(sim.handle_self,'randomColors')
    randomWalkingSpeed=sim.getScriptSimulationParameter(sim.handle_self,'randomWalkingSpeed')
    
    speedModulator=1
    if (randomWalkingSpeed) then
        speedModulator=(0.7+0.6*math.random())
    end
    nominalVelocity=nominalVelocity*speedModulator
    
    HairColors={4,{0.30,0.22,0.14},{0.75,0.75,0.75},{0.075,0.075,0.075},{0.75,0.68,0.23}}
    skinColors={2,{0.61,0.54,0.45},{0.52,0.45,0.35}}
    shirtColors={5,{0.27,0.36,0.54},{0.54,0.27,0.27},{0.31,0.51,0.33},{0.46,0.46,0.46},{0.18,0.18,0.18}}
    trouserColors={2,{0.4,0.34,0.2},{0.12,0.12,0.12}}
    shoeColors={2,{0.12,0.12,0.12},{0.25,0.12,0.045}}
    
    -- Initialize to random colors if desired:
    if (randomColors) then
        -- First we just retrieve all objects in the model:
        previousSelection=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(sim.handle_tree,modelHandle)
        modelObjects=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(previousSelection)
        -- Now we set random colors:
        math.randomseed(sim.getFloatParameter(sim.floatparam_rand)*10000) -- each lua instance should start with a different and 'good' seed
        setColor(modelObjects,'HAIR',HairColors[1+math.random(HairColors[1])])
        setColor(modelObjects,'SKIN',skinColors[1+math.random(skinColors[1])])
        setColor(modelObjects,'SHIRT',shirtColors[1+math.random(shirtColors[1])])
        setColor(modelObjects,'TROUSERS',trouserColors[1+math.random(trouserColors[1])])
        setColor(modelObjects,'SHOE',shoeColors[1+math.random(shoeColors[1])])
    end
    
    vel=nominalVelocity*0.8/0.56
    scaling=0
    tl=#legWaypoints
    dl=1/tl
    vp=0
    floorNotDetected=0
    obstacleDetected=0
    movement=0
    floorMovement=0
    obstacleMovement=0
    
    allFloorSensorsTriggerCount=0
    allFloorSensorsTriggerCountRequired=5
    obstacleSensorsNoTriggerCount=0
    obstacleSensorsNoTriggerCountRequired=2
end
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and earlier: 
colorCorrectionFunction=function(_aShapeHandle_) 
  local version=sim.getInt32Parameter(sim.intparam_program_version) 
  local revision=sim.getInt32Parameter(sim.intparam_program_revision) 
  if (version<30104)and(revision<3) then 
      return _aShapeHandle_ 
  end 
  return '@backCompatibility1:'.._aShapeHandle_ 
end 
------------------------------------------------------------------------------ 
 
 
setColor=function(objectTable,colorName,color)
    for i=1,#objectTable,1 do
        if (sim.getObjectType(objectTable[i])==sim.object_shape_type) then
            sim.setShapeColor(colorCorrectionFunction(objectTable[i]),colorName,0,color)
        end
    end
end


function sysCall_cleanup() 
    -- Restore to initial colors:
    if (randomColors) then
        previousSelection=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(sim.handle_tree,modelHandle)
        modelObjects=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(previousSelection)
        setColor(modelObjects,'HAIR',HairColors[2])
        setColor(modelObjects,'SKIN',skinColors[2])
        setColor(modelObjects,'SHIRT',shirtColors[2])
        setColor(modelObjects,'TROUSERS',trouserColors[2])
        setColor(modelObjects,'SHOE',shoeColors[2])
    end
end 

function sysCall_actuation() 
    dt=sim.getSimulationTimeStep()
    simTime=sim.getSimulationTime()
    
    forward=true
    
    -- 1. First check what the floor sensors tell us:
    if (floorNotDetected==0) then
        -- According to the floor sensors, we should walk straight here
        -- We have to check the close sensors to know whether we wanna turn (according to the floor sensors)
        l=sim.readProximitySensor(leftFloorSensorClose)
        r=sim.readProximitySensor(rightFloorSensorClose)
        if (l<1)and(r<1) then
            if (math.random()>0.5) then
                l=1
            else
                r=1
            end
        end
        if (l<1) then
            floorMovement=1
            floorNotDetected=floorMovement
        end
        if (r<1) then
            floorMovement=-1
            floorNotDetected=floorMovement
        end
    else
        -- According to the floor sensors, we should rotate here
        -- We have to check the far sensors to know whether we wanna walk straight again (according to the floor sensors)
        l=sim.readProximitySensor(leftFloorSensorFar)
        r=sim.readProximitySensor(rightFloorSensorFar)
        if (l>0)and(r>0) then
            allFloorSensorsTriggerCount=allFloorSensorsTriggerCount+1
        end
        if (allFloorSensorsTriggerCount*dt/0.05>allFloorSensorsTriggerCountRequired) then
            floorNotDetected=0
            floorMovement=0
            allFloorSensorsTriggerCount=0
        end
    end
    
    -- 2. Now check what the obstacle sensors tell us:
    lc,lcd=sim.readProximitySensor(leftSensorClose)
    rc,rcd=sim.readProximitySensor(rightSensorClose)
    lf=sim.readProximitySensor(leftSensorFar)
    rf=sim.readProximitySensor(rightSensorFar)
    if (obstacleDetected==0) then
        -- According to the floor sensors, we should walk straight here
        -- We have to check for obstacles to know whether we wanna turn (according to the obstacle sensors)
        if (lc>0)and(rc>0) then
            if (lcd>rcd) then
                lc=0
            else
                rc=0
            end
        end
        if (lc>0) then
            obstacleMovement=1
            obstacleDetected=obstacleMovement
        end
        if (rc>0) then
            obstacleMovement=-1
            obstacleDetected=obstacleMovement
        end
    else
        -- According to the obstacle sensors, we should rotate here
        -- We have to check whether we can walk straight again (according to the obstacle sensors)
        if (lf<1)and(rf<1) then
            obstacleSensorsNoTriggerCount=obstacleSensorsNoTriggerCount+1
        end
        if (obstacleSensorsNoTriggerCount*dt/0.05>obstacleSensorsNoTriggerCountRequired) then
            obstacleDetected=0
            obstacleMovement=0
            obstacleSensorsNoTriggerCount=0
        end
    end
    
    
    --3. Now decide how to walk by evaluating what the floor tells us and what the obstacles tell us (the floor has more weight than obstacles)
    wantRotation=(floorMovement~=0)or(obstacleMovement~=0)
    if (wantRotation) then
        if (movement==0) then
            movement=floorMovement
            if (movement==0) then
                movement=obstacleMovement
            end
        end
    else
        movement=0
    end
    
    s=sim.getObjectSizeFactor(modelHandle)
    
    if (movement==0) then
            scaling=1
            vp=vp+sim.getSimulationTimeStep()*vel
            p=math.fmod(vp,1)
            indexLow=math.floor(p/dl)
            t=p/dl-indexLow
            oppIndexLow=math.floor(indexLow+tl/2)
            if (oppIndexLow>=tl) then oppIndexLow=oppIndexLow-tl end
            indexHigh=indexLow+1
            if (indexHigh>=tl) then indexHigh=indexHigh-tl end
            oppIndexHigh=oppIndexLow+1
            if (oppIndexHigh>=tl) then oppIndexHigh=oppIndexHigh-tl end
    
            leftLegJointValue=(legWaypoints[indexLow+1]*(1-t)+legWaypoints[indexHigh+1]*t)*scaling
            leftKneeJointValue=(kneeWaypoints[indexLow+1]*(1-t)+kneeWaypoints[indexHigh+1]*t)*scaling
            leftAnkleJointValue=(ankleWaypoints[indexLow+1]*(1-t)+ankleWaypoints[indexHigh+1]*t)*scaling
            leftShoulderJointValue=(shoulderWaypoints[indexLow+1]*(1-t)+shoulderWaypoints[indexHigh+1]*t)*scaling
            leftElbowJointValue=(elbowWaypoints[indexLow+1]*(1-t)+elbowWaypoints[indexHigh+1]*t)*scaling
    
            rightLegJointValue=(legWaypoints[oppIndexLow+1]*(1-t)+legWaypoints[oppIndexHigh+1]*t)*scaling
            rightKneeJointValue=(kneeWaypoints[oppIndexLow+1]*(1-t)+kneeWaypoints[oppIndexHigh+1]*t)*scaling
            rightAnkleJointValue=(ankleWaypoints[oppIndexLow+1]*(1-t)+ankleWaypoints[oppIndexHigh+1]*t)*scaling
            rightShoulderJointValue=(shoulderWaypoints[oppIndexLow+1]*(1-t)+shoulderWaypoints[oppIndexHigh+1]*t)*scaling
            rightElbowJointValue=(elbowWaypoints[oppIndexLow+1]*(1-t)+elbowWaypoints[oppIndexHigh+1]*t)*scaling
    
    
            vvv=s*nominalVelocity*scaling*(relativeVel[indexLow+1]*(1-t)+relativeVel[indexHigh+1]*t)
    
            m=sim.getObjectMatrix(modelHandle,-1)
            r=sim.getObjectPosition(modelHandle,-1)
            r[1]=r[1]+m[1]*dt*vvv
            r[2]=r[2]+m[5]*dt*vvv
            sim.setObjectPosition(modelHandle,-1,r)
    else
        leftLegJointValue=0
        leftKneeJointValue=0
        leftAnkleJointValue=0
        leftShoulderJointValue=0
        leftElbowJointValue=0
    
        rightLegJointValue=0
        rightKneeJointValue=0
        rightAnkleJointValue=0
        rightShoulderJointValue=0
        rightElbowJointValue=0
        if (movement==-1) then
            r=sim.getObjectOrientation(modelHandle,-1)
            r[3]=r[3]+dt*1.8*speedModulator
            sim.setObjectOrientation(modelHandle,-1,r)
        end
        if (movement==1) then
            r=sim.getObjectOrientation(modelHandle,-1)
            r[3]=r[3]-dt*1.8*speedModulator
            sim.setObjectOrientation(modelHandle,-1,r)
        end
    end
    
    
    sim.setJointPosition(legJointHandles[1],leftLegJointValue)
    sim.setJointPosition(kneeJointHandles[1],leftKneeJointValue)
    sim.setJointPosition(ankleJointHandles[1],leftAnkleJointValue)
    sim.setJointPosition(shoulderJointHandles[1],leftShoulderJointValue)
    sim.setJointPosition(elbowJointHandles[1],leftElbowJointValue)
    
    sim.setJointPosition(legJointHandles[2],rightLegJointValue)
    sim.setJointPosition(kneeJointHandles[2],rightKneeJointValue)
    sim.setJointPosition(ankleJointHandles[2],rightAnkleJointValue)
    sim.setJointPosition(shoulderJointHandles[2],rightShoulderJointValue)
    sim.setJointPosition(elbowJointHandles[2],rightElbowJointValue)
    
end 
