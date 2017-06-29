--**************************
-- Actions for the YouBot Kuka Robot
-- @author Klaus Raizer
-- @date 08-02-2017
--
-- Description: Controls information comming from
-- This script calls a lua script located at the same folder.
--**************************

if (sim_call_type==sim_childscriptcall_initialization) then
    simSetThreadSwitchTiming(100) -- Default timing for automatic thread switching
    maximum_number_of_products_per_collumn=10 --Arbitrary parameter
    shelf_bound_box_radius = 0.6 --NOTE: must be carefull not to put other products too close to this shelf.

    positionSensorGreenHandle=simGetObjectHandle('PositionSensorGreen')
    positionSensorRedHandle=simGetObjectHandle('PositionSensorRed')
    positionSensorYellowHandle=simGetObjectHandle('PositionSensorYellow')

    proximitySensorInputGreenHandle=simGetObjectHandle('proximitySensorInputGreen')
    proximitySensorInputRedHandle=simGetObjectHandle('proximitySensorInputRed')
    proximitySensorInputYellowHandle=simGetObjectHandle('proximitySensorInputYellow')
end


if (sim_call_type==sim_childscriptcall_actuation) then
  --[[  local listOfProducts=getListOfProducts()
    print("listOfProducts: ")
    local n=table.getn(listOfProducts)
    print("n: ",n)
    for i=1,n,1 do
        print(listOfProducts[i])
    end]]--
end


if (sim_call_type==sim_childscriptcall_sensing) then
end


if (sim_call_type==sim_childscriptcall_cleanup) then
end

-- Shelf functions
-- http://www.coppeliarobotics.com/helpFiles/en/accessingGeneralObjects.htm

-- List all products on this shelf
getListOfProducts = function ()

    shelfHandle=simGetObjectAssociatedWithScript(sim_handle_self)
    shelfName=simGetObjectName(shelfHandle)

    objects = simGetObjectsInTree(sim_handle_scene,0,0)
    numberOfObjectsOfTypeShape=table.getn(objects)

    cp=1
    cs=1

    products={}
    shelves={}
    for i=1,numberOfObjectsOfTypeShape,1 do
        objectName=simGetObjectName(objects[i])
       -- p=simGetObjectPosition(objects[i],shelfHandle) -- relative position
        --distance=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
        if string.match(objectName, "product") then
            products[cp]=objects[i]
            cp=cp+1
        --elseif (objectName=="ShelfBody") then--string.match(objectName, "ShelfBody") then
        elseif (string.match(objectName, "ShelfBody")) and not (string.match(objectName, "ForPathPlanning")) then
            shelves[cs]=objects[i]
            cs=cs+1
        end
    end

    shelfContent={}
    counter=1
    for p=1,cp-1,1 do
        product=products[p]
        closestShelf=shelves[1]
        p=simGetObjectPosition(product,closestShelf) -- relative position
        distance=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
        for s=1,cs-1,1 do
            p=simGetObjectPosition(product,shelves[s]) -- relative position
            distance_new=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
            if(distance_new<distance)then
                closestShelf=shelves[s]
                distance=distance_new
            end
        end
        --print('closestShelf: ',simGetObjectName(closestShelf))
        if(closestShelf==shelfHandle and distance <= shelf_bound_box_radius)then
            shelfContent[counter]=simGetObjectName(product)
            --print(shelfName,' ',simGetObjectName(shelfContent[counter]))
            counter=counter+1
        end
    end
    return shelfContent
end


-- Return a list with all current pickable products
getListOfPickableProducts = function(inInts, inFloats, inStrings, inBuff)

    psg=simGetObjectPosition(positionSensorGreenHandle,-1) -- position in space of the green sensor
    psr=simGetObjectPosition(positionSensorRedHandle,-1) -- position in space of the red sensor
    psy=simGetObjectPosition(positionSensorYellowHandle,-1) -- position in space of the yellow sensor

    shelfContent=getListOfProducts()

    pickable={"None", "None", "None"}
    for i = 1,table.getn(shelfContent),1 do
        --print(shelfContent[i])
        p=simGetObjectPosition(simGetObjectHandle(shelfContent[i]),-1)
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

---------------------------
-- Adds Product to Shelf
-- Returns {1},{},{},"" if successful and {0},{},{},"" if it fails
addProduct=function(inInts, inFloats, inStrings, inBuff)
    productType = inStrings[1]
--[[
local initial_t = simGetSimulationTime()
local dt=0

while(dt<1000)do
t=simGetSimulationTime()
if(t~=nil)then
    dt=t-initial_t
end
--print("dt: ",dt)
end

]]-- --locks the simulation

--simWait(1000, true) -- only works with threaded scripts


    if((productType=='productGreen') or (productType=='productRed') or (productType=='productYellow'))then
        greenPosition={.30,.1,.50}
        redPosition={.30,0,.50}
        yellowPosition={.30,-.1,.50}
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
            local res,dist,pt,obj=simHandleProximitySensor(h)
            if obj then
                local fullnm=simGetObjectName(obj)
                local suffix,nm=simGetNameSuffix(fullnm)
                print("Couldn't create a new ",nm,". There is already ",fullnm," in this position. ")
                return {0},{},{},""
            else
                string_to_avoid_dynamic_naming=productType .. '#'
                objectHandle=simGetObjectHandle(string_to_avoid_dynamic_naming)
                copiedObjectHandles=simCopyPasteObjects({objectHandle},1)
                shelfHandle=simGetObjectAssociatedWithScript(sim_handle_self)
                copiedObjectHandle=copiedObjectHandles[1]
                simSetObjectOrientation(copiedObjectHandle,shelfHandle,{0,0,0})
                simSetObjectPosition(copiedObjectHandle,shelfHandle,position)
                simSetObjectInt32Parameter(copiedObjectHandle,sim_shapeintparam_static,0)
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
