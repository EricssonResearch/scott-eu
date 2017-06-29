--**************************
-- Target following script for the mobile robot
-- @author Klaus Raizer
-- @date 23-01-2017
--
-- This script takes care of the YouBot mobile platform:
-- The platform will try to follow the position and orientation of the 'youBot_vehicleTargetPosition' object
--**********************************


if (sim_call_type==sim_childscriptcall_initialization) then 
    --vehicleReference=simGetObjectHandle('youBot_vehicleReference')
    vehicleReference=simGetObjectHandle('youBot_Center')
    vehicleTarget=simGetObjectHandle('youBot_vehicleTargetPosition')

  --  simSetObjectPosition(vehicleTarget,sim_handle_parent,{0,0,0})
  --  simSetObjectOrientation(vehicleTarget,sim_handle_parent,{0,0,0})

    simSetObjectPosition(vehicleTarget,vehicleReference,{0,0,0})
    simSetObjectOrientation(vehicleTarget,vehicleReference,{0,0,0})


    simSetObjectParent(vehicleTarget,-1,true)
    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=simGetObjectHandle('rollingJoint_fl')
    wheelJoints[2]=simGetObjectHandle('rollingJoint_rl')
    wheelJoints[3]=simGetObjectHandle('rollingJoint_rr')
    wheelJoints[4]=simGetObjectHandle('rollingJoint_fr')

    previousForwBackVel=0
    previousLeftRightVel=0
    previousRotVel=0


    pParam=-20 
    --maxV=10 --maxV=2 -- Maximum linear velocity for the robot's movement
    maxV=tonumber(simGetScriptSimulationParameter(sim_handle_self,'robotMaximumLinearVelocity'))
    
    pParamRot=10
    --maxVRot=10 --maxVRot=3 -- Maximum angular velocity for the robot's movement
    maxVRot=tonumber(simGetScriptSimulationParameter(sim_handle_self,'robotMaximumAngularVelocity'))
   
    accelF=0.035

    maximumAllowedLinearVelocity=20 -- Maximum acceptable linear velocity 
    maximumAllowedAngularVelocity=10 -- Maximum acceptable angular velocity 

    Kpx=100
    Kpy=100

    Kdx=0
    Kdy=0

    Kix=0
    Kiy=0

    Vx=0 --forwBackVel
    Vy=0 --leftRightVel
    previousVx=0
    previousVy=0

    derivVx=0
    derivVy=0

    previous_t=simGetSimulationTime()

end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_actuation) then 

    relP=simGetObjectPosition(vehicleTarget,vehicleReference)
    relE=simGetObjectOrientation(vehicleTarget,vehicleReference)
     
    -- Speed Control
    --TODO: Finish implementing PID
    Vx=(relP[1]*Kpx + derivVx*Kdx)
    Vy=-(relP[2]*Kpy + derivVy*Kdy)
    rotVel=-relE[3]*pParamRot

    dVx=Vx-previousVx
    dVy=Vy-previousVy
    dr=rotVel-previousRotVel


    --TODO: Make sure maximum accelarations and velocities are respected

    current_t=simGetSimulationTime()


    delta_t=current_t-previous_t

    if(delta_t>0)then
        derivVx=dVx/delta_t
    else
        --print('delta_t division by zero error ',dVx,' ',previous_t)
        derivVx=0
    end
    previous_t=current_t

    --print('delta_t: ',delta_t,'  derivVx: ',derivVx,'  dVx: ',dVx,'  Vx: ',Vx,'  previousVx: ',previousVx)
    
    -- Set joint velocities according to swedish wheels mechanism
    simSetJointTargetVelocity(wheelJoints[1],-Vx-Vy-rotVel)
    simSetJointTargetVelocity(wheelJoints[2],-Vx+Vy-rotVel)
    simSetJointTargetVelocity(wheelJoints[3],-Vx-Vy+rotVel)
    simSetJointTargetVelocity(wheelJoints[4],-Vx+Vy+rotVel)
    
    previousVx=Vx
    previousVy=Vy
    previousRotVel=rotVel
end 

setRobotMaximumLinearVelocity = function (newMaxV)
    newMaxV=tonumber(newMaxV)
    if(newMaxV>maximumAllowedLinearVelocity)then
        maxV=maximumAllowedLinearVelocity
    elseif(newMaxV<0)then
        maxV=0
    else
        maxV=newMaxV
    end
    return true
end

getRobotMaximumLinearVelocity = function()
    return maxV
end

-- Maximum angular velocity for the robot's movement
setRobotMaximumAngularVelocity = function (newMaxVRot)
    newMaxVRot=tonumber(newMaxVRot)
    if(newMaxVRot>maximumAllowedAngularVelocity)then
        maxVRot=maximumAllowedAngularVelocity
    elseif(newMaxVRot<0)then
        maxVRot=0
    else
        maxVRot=newMaxVRot
    end
    return true
end

getRobotMaximumAngularVelocity = function()
    return maxVRot
end  