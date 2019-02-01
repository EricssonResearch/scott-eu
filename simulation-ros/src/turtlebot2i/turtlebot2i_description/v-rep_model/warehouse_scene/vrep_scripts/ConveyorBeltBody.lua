if (sim_call_type==sim.syscb_init) then 
    pathHandle=sim.getObjectHandle("ConveyorBeltPath")
    forwarder=sim.getObjectHandle('ConveyorBelt_forwarder')
    sim.setPathTargetNominalVelocity(pathHandle,0) -- for backward compatibility
end 

if (sim_call_type==sim.syscb_actuation) then 
    local beltVelocity=sim.getScriptSimulationParameter(sim.handle_self,"conveyorBeltVelocity")
    local dt=sim.getSimulationTimeStep()
    local pos=sim.getPathPosition(pathHandle)
    pos=pos+beltVelocity*dt
    sim.setPathPosition(pathHandle,pos) -- update the path's intrinsic position
    
    
    -- Here we "fake" the transportation pads with a single static rectangle that we dynamically reset
    -- at each simulation pass (while not forgetting to set its initial velocity vector) :
    
    local relativeLinearVelocity={beltVelocity,0,0}
    -- Reset the dynamic rectangle from the simulation (it will be removed and added again)
    sim.resetDynamicObject(forwarder)
    -- Compute the absolute velocity vector:
    local m=sim.getObjectMatrix(forwarder,-1)
    m[4]=0 -- Make sure the translation component is discarded
    m[8]=0 -- Make sure the translation component is discarded
    m[12]=0 -- Make sure the translation component is discarded
    local absoluteLinearVelocity=sim.multiplyVector(m,relativeLinearVelocity)
    -- Now set the initial velocity of the dynamic rectangle:
    sim.setObjectFloatParameter(forwarder,sim.shapefloatparam_init_velocity_x,absoluteLinearVelocity[1])
    sim.setObjectFloatParameter(forwarder,sim.shapefloatparam_init_velocity_y,absoluteLinearVelocity[2])
    sim.setObjectFloatParameter(forwarder,sim.shapefloatparam_init_velocity_z,absoluteLinearVelocity[3])
end 
