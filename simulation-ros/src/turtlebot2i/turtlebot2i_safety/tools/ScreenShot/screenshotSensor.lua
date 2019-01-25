function screenshotRemoteAPI()
    --====================================Render it===================================
    sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_resolution_x,resX)
    sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_resolution_y,resY)
    sim.setObjectFloatParameter(visionSensorHandle,sim.visionfloatparam_perspective_angle,camAngle*math.pi/180)
    if sim.boolAnd32(simGetUIButtonProperty(uiHandle,51),sim.buttonproperty_isdown)~=0 then
        sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_pov_focal_blur,1)
    else
        sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_pov_focal_blur,0)
    end
    sim.setObjectFloatParameter(visionSensorHandle,sim.visionfloatparam_pov_blur_distance,focalDistance)
    sim.setObjectFloatParameter(visionSensorHandle,sim.visionfloatparam_pov_aperture,aperture)
    sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_pov_blur_sampled,blurSamples)
    if sim.boolAnd32(simGetUIButtonProperty(uiHandle,50),sim.buttonproperty_isdown)~=0 then
        sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_render_mode,3)
    else
        sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_render_mode,0)
    end

    local parentCam=sim.getObjectParent(visionSensorHandle)
    local savedVisibilityMask=0
    if sim.getObjectType(parentCam)~=sim.object_camera_type then
        parentCam=-1
    else
        local r,savedVisibilityMask=sim.getObjectInt32Parameter(parentCam,sim.objintparam_visibility_layer)
        sim.setObjectInt32Parameter(parentCam,sim.objintparam_visibility_layer,0)
    end
    local newAttr=sim.displayattribute_renderpass
    newAttr=newAttr+sim.displayattribute_forvisionsensor
    newAttr=newAttr+sim.displayattribute_ignorerenderableflag
    sim.setObjectInt32Parameter(visionSensorHandle,sim.visionintparam_rendering_attributes,newAttr)

    sim.handleVisionSensor(visionSensorHandle)
    if parentCam~=-1 then
        sim.setObjectInt32Parameter(parentCam,sim.objintparam_visibility_layer,savedVisibilityMask)
    end
    sim.addStatusbarMessage("The screenshot was rendered.")
    --====================================Save it===================================
    local openDialog=sim.boolAnd32(simGetUIButtonProperty(uiHandle,9),sim.buttonproperty_isdown)~=0
    local saveRgba=sim.boolAnd32(simGetUIButtonProperty(uiHandle,5),sim.buttonproperty_isdown)~=0
    local options=0
    local cutOff=0
    if saveRgba then
        options=1
        cutOff=0.99
    end
    local image,resX,resY=sim.getVisionSensorCharImage(visionSensorHandle,0,0,0,0,cutOff)
    local filenameAndPath
    if openDialog then
        filenameAndPath=sim.fileDialog(sim.filedlg_type_save,'title','','screenshot.png','image file','*')
    else
        local theOs=sim.getInt32Parameter(sim.intparam_platform)
        if theOs==1 then
            -- MacOS, special: executable is inside of a bundle:
            filenameAndPath='../../../vrep_screenshot_'..os.date("%Y_%m_%d-%H_%M_%S",os.time())..'.png'
        else
            --print("Os!=1. Linux use this!")
            filenameAndPath='vrep_screenshot_'..os.date("%Y_%m_%d-%H_%M_%S",os.time())..'.png'
        end
    end
    if filenameAndPath then
        --if sim.saveImage(image,{resX,resY},options,filenameAndPath,-1)~=-1 then 
        --quality: the quality of the written image: 0 for best compression, 100 for largest file. Use -1 for default behaviour.
        if sim.saveImage(image,{resX,resY},options,filenameAndPath,0)~=-1 then
            sim.addStatusbarMessage("Screenshot was saved to "..filenameAndPath)
            --sim.displayDialog('Screenshot',"Screenshot was saved to "..filenameAndPath,sim.dlgstyle_ok,false,'')
            return {},{},{"Success"},''
        else
            sim.addStatusbarMessage("Failed saving the screenshot. Did you specify a supported image extension?")
            --sim.displayDialog('Screenshot',"Failed saving the screenshot. Did you specify a supported image extension?",sim.dlgstyle_ok,false,'')
            return {},{},{"Failed1"},''
        end
    else
        sim.addStatusbarMessage("Failed saving the screenshot. Bad filename or action canceled.")
        --sim.displayDialog('Screenshot',"Failed saving the screenshot. Bad filename or action canceled.",sim.dlgstyle_ok,false,'')
        return {},{},{"Failed2"},''
    end    
end
--=========================Original Code Below=================================
