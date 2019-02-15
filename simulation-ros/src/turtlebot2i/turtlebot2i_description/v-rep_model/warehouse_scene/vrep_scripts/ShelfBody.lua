
-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

-- Shelf functions
-- http://www.coppeliarobotics.com/helpFiles/en/accessingGeneralObjects.htm

-- List all products on this shelf

getObjectName = function(intArg, fltArg, strArg, byteArg)
    objectHandle = intArg[1]
    objectName=sim.getObjectName(objectHandle)
    return {},{},{objectName},''
end

getListOfProducts = function ()

    shelfHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    shelfName=sim.getObjectName(shelfHandle)

    objects = sim.getObjectsInTree(sim.handle_scene,0,0)
    numberOfObjectsOfTypeShape=table.getn(objects)

    cp=1
    cs=1
    
    products={}
    shelves={}
    for i=1,numberOfObjectsOfTypeShape,1 do
        objectName=sim.getObjectName(objects[i])
       -- p=sim.getObjectPosition(objects[i],shelfHandle) -- relative position
        --distance=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
        if string.match(objectName, "product") then
            products[cp]=objects[i]
            cp=cp+1
        elseif string.match(objectName, "ShelfBody") then
            shelves[cs]=objects[i] 
            cs=cs+1
        end
    end

    shelfContent={}
    counter=1
    for p=1,cp-1,1 do
        product=products[p]
        closestShelf=shelves[1]
        p=sim.getObjectPosition(product,closestShelf) -- relative position
        distance=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
        for s=1,cs-1,1 do
            p=sim.getObjectPosition(product,shelves[s]) -- relative position
            distance_new=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
            if(distance_new<distance)then
                closestShelf=shelves[s]
                distance=distance_new
            end
        end

        if(closestShelf==shelfHandle and distance <= shelf_bound_box_radius)then    
            shelfContent[counter]=sim.getObjectName(product)
            --print(shelfName,' ',sim.getObjectName(shelfContent[counter]))
            counter=counter+1
        end
    end
    return shelfContent
end

---------------------------
-- Adds Product to Shelf
-- Returns {1},{},{},"" if successful and {0},{},{},"" if it fails
addProduct=function(inInts, inFloats, inStrings, inBuff)

    productType = inStrings[1]
--[[
local initial_t = sim.getSimulationTime()
local dt=0

while(dt<1000)do
t=sim.getSimulationTime()
if(t~=nil)then
    dt=t-initial_t
end
--print("dt: ",dt)
end

]]-- --locks the simulation

--sim.wait(1000, true) -- only works with threaded scripts


    if((productType=='productGreen') or (productType=='productRed') or (productType=='productYellow'))then
        greenPosition={-.35,.1,.3}
        redPosition={-.35,0,.3}
        yellowPosition={-.35,-.1,.3}

        if(productType=='productGreen')then
            h=proximitySensorInputGreenHandle
            position=greenPosition            
        elseif(productType=='productRed')then
            h=proximitySensorInputRedHandle
            position=redPosition
        elseif(productType=='productYellow')then
            h=proximitySensorInputYellowHandle
            position=yellowPosition
        end
        
        --Check if there is enough room for it
        
        counter_of_product_type=0;
        shelfContent=getListOfProducts()
        
       -- print('======')
        for i = 1,table.getn(shelfContent),1 do 
           -- print("-> ",shelfContent[i])           
            if(string.match(shelfContent[i], productType))then
               
                counter_of_product_type=counter_of_product_type+1;
            end
        end
       -- print('======')
        --print("counter_of_product_type(",productType,"): ",counter_of_product_type)
        if(counter_of_product_type<maximum_number_of_products_per_collumn)then
            -- Check if position is free before creating a new one        
            local res,dist,pt,obj=sim.handleProximitySensor(h)
            if obj then
                local fullnm=sim.getObjectName(obj)
                local suffix,nm=sim.getNameSuffix(fullnm)
                print("Couldn't create a new ",nm,". There is already ",fullnm," in this position. ")
                return {0},{},{},""            
            else
                string_to_avoid_dynamic_naming=productType .. '#'
                objectHandle=sim.getObjectHandle(string_to_avoid_dynamic_naming)
                copiedObjectHandles=sim.copyPasteObjects({objectHandle},1)
                shelfHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
                copiedObjectHandle=copiedObjectHandles[1]
                sim.setObjectOrientation(copiedObjectHandle,shelfHandle,{0,0,0})
                sim.setObjectPosition(copiedObjectHandle,shelfHandle,position)
                sim.setObjectInt32Parameter(copiedObjectHandle,sim.shapeintparam_static,0) -- ,0) for dynamic and ,1) for static
		sim.setObjectParent(copiedObjectHandle, shelfHandle, true)                
            end
        else   
            print("Product creation error. There are too many (",counter_of_product_type,") ",productType," units in this shelf.")
            return {0},{},{"full shelf"},"" 
        end

    else
        print('Product creation error. Product type ',productType,' does not exist.')
        return {0},{},{"product not found"},"" 
    end

    return {1},{},{"True"},"";
end



-- Return a list with all currrent pickable products
    getListOfPickableProducts = function(inInts, inFloats, inStrings, inBuff)

    psg=sim.getObjectPosition(positionSensorGreenHandle,-1) -- position in space of the green sensor
    psr=sim.getObjectPosition(positionSensorRedHandle,-1) -- position in space of the red sensor
    psy=sim.getObjectPosition(positionSensorYellowHandle,-1) -- position in space of the yellow sensor
   
    shelfContent=getListOfProducts()

    pickable={"None", "None", "None"}
    for i = 1,table.getn(shelfContent),1 do
        --print(shelfContent[i])
        p=sim.getObjectPosition(sim.getObjectHandle(shelfContent[i]),-1)
        -- for this product to be pickable, it should be bellow (in z) its sensor
        if(string.match(shelfContent[i], "Yellow"))then
            if(p[3]<psr[3])then 
                pickable[1]=shelfContent[i]
            end
        elseif(string.match(shelfContent[i], "Red"))then
            if(p[3]<psg[3])then 
                pickable[2]=shelfContent[i]
            end
        elseif(string.match(shelfContent[i], "Green"))then
            if(p[3]<psy[3])then 
                pickable[3]=shelfContent[i]
            end
        else print('Product ID error in child script.')
        end

    end
    
    return {1}, {}, pickable, ""

end

if (sim_call_type==sim.syscb_init) then
 
end




if (sim_call_type==sim.syscb_actuation) then

if (firstRun==nil) then
    sim.setThreadSwitchTiming(100) -- Default timing for automatic thread switching  
    maximum_number_of_products_per_collumn=10 --Arbitrary parameter
    shelf_bound_box_radius = 0.6 --NOTE: must be carefull not to put other products too close to this shelf. 

    positionSensorGreenHandle=sim.getObjectHandle('PositionSensorGreen')
    positionSensorRedHandle=sim.getObjectHandle('PositionSensorRed')
    positionSensorYellowHandle=sim.getObjectHandle('PositionSensorYellow')

    proximitySensorInputGreenHandle=sim.getObjectHandle('proximitySensorInputGreen')
    proximitySensorInputRedHandle=sim.getObjectHandle('proximitySensorInputRed')
    proximitySensorInputYellowHandle=sim.getObjectHandle('proximitySensorInputYellow')

    num_red_products = sim.getScriptSimulationParameter(sim.handle_self,'num_red_products')
    num_green_products = sim.getScriptSimulationParameter(sim.handle_self,'num_green_products')
    num_yellow_products = sim.getScriptSimulationParameter(sim.handle_self,'num_yellow_products')
    fill_shelf_on_start = sim.getScriptSimulationParameter(sim.handle_self,'fill_shelf_on_start')

    count_red_products = 0
    count_green_products = 0
    count_yellow_products = 0
    t0=os.clock()
    dt=2
    firstRun=false
end


if(os.clock() - t0 >= dt)then
    t0 = os.clock()

    if fill_shelf_on_start then

        if count_red_products < num_red_products then
            ret = addProduct({}, {}, {'productRed'}, '')
            if ret[1] == 1 then
                count_red_products = count_red_products + 1
            end
        end

        if count_green_products < num_green_products then
            ret = addProduct({}, {}, {'productGreen'}, '')
            if ret[1] == 1 then
                count_green_products = count_green_products + 1
            end
        end

        if count_yellow_products < num_yellow_products then
            ret = addProduct({}, {}, {'productYellow'}, '')
            if ret[1] == 1 then
                count_yellow_products = count_yellow_products + 1
            end
        end
    end
end
end


if (sim_call_type==sim.syscb_sensing) then
end


if (sim_call_type==sim.syscb_cleanup) then
end
