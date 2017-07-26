/*******************************************************************************
 * Copyright (c) 2011, 2012 IBM Corporation and others.
 *
 *  All rights reserved. This program and the accompanying materials
 *  are made available under the terms of the Eclipse Public License v1.0
 *  and Eclipse Distribution License v. 1.0 which accompanies this distribution.
 *  
 *  The Eclipse Public License is available at http://www.eclipse.org/legal/epl-v10.html
 *  and the Eclipse Distribution License is available at
 *  http://www.eclipse.org/org/documents/edl-v10.php.
 *  
 *  Contributors:
 *  
 *	   Sam Padgett	       - initial API and implementation
 *     Michael Fiedler     - adapted for OSLC4J
 *     Jad El-khoury        - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 *     Matthieu Helleboid   - Support for multiple Service Providers.
 *     Anass Radouani       - Support for multiple Service Providers.
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/

package se.ericsson.cf.scott.sandbox;

import java.io.IOException;
import javax.servlet.ServletContext;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;
import java.util.List;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import org.eclipse.lyo.tools.store.Store;
import org.eclipse.lyo.tools.store.StoreFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.ServiceProviderInfo;
import scott.lyo.domain.planning.Action;
import scott.lyo.domain.planning.ActionType;
import scott.lyo.domain.planning.Mission;
import scott.lyo.domain.warehouse.Place;
import scott.lyo.domain.planning.Plan;
import scott.lyo.domain.planning.Predicate;
import scott.lyo.domain.planning.ProblemState;
import scott.lyo.domain.warehouse.Robot;
import scott.lyo.domain.planning.Variable;
import scott.lyo.domain.planning.VariableInstance;
import scott.lyo.domain.warehouse.Waypoint;
import scott.lyo.domain.warehouse.WhObject;

// Start of user code imports
import se.ericsson.cf.scott.sandbox.clients.WarehouseAdaptorClient;
import org.eclipse.lyo.oslc4j.core.model.Link;
import se.ericsson.cf.scott.sandbox.clients.OslcClientException;
import java.net.URISyntaxException;
// End of user code

// Start of user code pre_class_code
// End of user code

public class TwinManager {

    // Start of user code class_attributes
    private final static Logger log = LoggerFactory.getLogger(TwinManager.class);
    private static WarehouseAdaptorClient warehouseAdaptorClient;
    private static String spId = "dummy";
    private static Store store;
    // End of user code
    
    
    // Start of user code class_methods
    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {
        
        // Start of user code contextInitializeServletListener
        final ServletContext context = servletContextEvent.getServletContext();
        final String queryUri = context.getInitParameter(
                "se.ericsson.cf.scott.sandbox.store.query");
        final String updateUri = context.getInitParameter(
                "se.ericsson.cf.scott.sandbox.store.query");
        warehouseAdaptorClient = new WarehouseAdaptorClient("http://sandbox-warehouse:8080/sandbox-warehouse/services");
//        try {
//            store = StoreFactory.sparql(queryUri, updateUri);
//            // TODO Andrew@2017-07-18: Remember to deactivate when switch to more persistent arch
//            store.removeAll();
//            throw new IOException("test");
//        } catch (IOException e) {
//            log.error("SPARQL Store failed to initialise with the URIs query={};update={}",
//                    new Object[]{queryUri, updateUri, e});
//        }
        // End of user code
    }

    public static void contextDestroyServletListener(ServletContextEvent servletContextEvent) 
    {
        
        // Start of user code contextDestroyed
        // TODO Implement code to shutdown connections to data backbone etc...
        // End of user code
    }

    public static ServiceProviderInfo[] getServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        ServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "ServiceProviderInfo[] getServiceProviderInfos(...)"
        final ServiceProviderInfo serviceProviderInfo = new ServiceProviderInfo();
        serviceProviderInfo.serviceProviderId = spId;
        serviceProviderInfo.name = "Handcrafted robot";
        serviceProviderInfos = new ServiceProviderInfo[]{serviceProviderInfo};
        // End of user code
        return serviceProviderInfos;
    }



    public static Robot getRobot(HttpServletRequest httpServletRequest, final String serviceProviderId, final String robotId)
    {
        Robot aResource = null;
        
        // Start of user code getRobot
        if ("1".equals(robotId)) {
            try {
                final AbstractResource wp2 = warehouseAdaptorClient.fetchWaypoint("dummy", "wp2");

                aResource = new Robot(spId, "1");
                aResource.setIsAt(new Link(wp2.getAbout()));
                aResource.setChargeLevel(7);
                aResource.setCapacity(1);
                aResource.setMaxCharge(10);
                aResource.setHighCharge(6);
                aResource.setLowCharge(3);
                aResource.setIsCharging(false);
            } catch (OslcClientException | URISyntaxException e) {
                e.printStackTrace();
            }
        }
        // End of user code
        return aResource;
    }




    public static String getETagFromRobot(final Robot aResource)
    {
        String eTag = null;
        // Start of user code getETagFromRobot
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}
