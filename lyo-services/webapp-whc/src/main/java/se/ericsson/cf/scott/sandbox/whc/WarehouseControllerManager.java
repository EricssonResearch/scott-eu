// Start of user code Copyright
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
// End of user code

package se.ericsson.cf.scott.sandbox.whc;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;
import java.util.List;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import se.ericsson.cf.scott.sandbox.whc.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.whc.ServiceProviderInfo;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.mission.AgentRequest;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.mission.RegistrationRequest;
import eu.scott.warehouse.domains.pddl.Step;


// Start of user code imports
import java.net.URI;
import javax.servlet.ServletContext;
//import org.eclipse.lyo.store.Store;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.UUID;
import java.util.concurrent.Executors;


//import org.eclipse.lyo.store.StoreFactory;
import se.ericsson.cf.scott.sandbox.whc.managers.MqttManager;
import se.ericsson.cf.scott.sandbox.whc.managers.TRSManager;
import se.ericsson.cf.scott.sandbox.whc.planning.PlanDTO;
import se.ericsson.cf.scott.sandbox.whc.trs.WhcChangeHistories;

import org.eclipse.paho.client.mqttv3.MqttClient;
// End of user code

// Start of user code pre_class_code
// End of user code

public class WarehouseControllerManager {

    // Start of user code class_attributes
    private final static Logger log = LoggerFactory.getLogger(WarehouseControllerManager.class);
    private final static UUID whcUUID = UUID.randomUUID();
    private static ServletContext context;
    private static WhcChangeHistories changeHistoriesInstance;
    private static ScheduledExecutorService execService = Executors.newSingleThreadScheduledExecutor();
    // End of user code
    
    
    // Start of user code class_methods
    private static UUID getUUID() {
        return whcUUID;
    }

    public static String p(final String s) {
        return context.getInitParameter(AdaptorHelper.parameterFQDN(s));
    }

    public static WhcChangeHistories getChangeHistories() {
        return changeHistoriesInstance;
    }

    public static void setChangeHistories(final WhcChangeHistories changeHistoriesInstance) {
        WarehouseControllerManager.changeHistoriesInstance = changeHistoriesInstance;
    }

    @NotNull
    public static String getWhcId() {
        return "whc-" + getUUID();
    }
    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {
        
        // Start of user code contextInitializeServletListener

        AdaptorHelper.initLogger();
        log.info("WHC instance '{}' is initialising", whcUUID.toString());

        context = servletContextEvent.getServletContext();
        // TODO Andrew@2018-07-30: https://github.com/EricssonResearch/scott-eu/issues/101
//        StoreManager.initLyoStore();

        final MqttClient mqttClient = MqttManager.initMqttClient();

        execService.schedule(() -> {
            log.debug("Can now begin the execution of the planner");
            TRSManager.fetchPlanForProblem("sample-problem-request.ttl");
//            final Object[] resources = new PlanRequestBuilder().build();
//            MqttManager.triggerPlan(new PlanDTO(resources));
        }, 30, TimeUnit.SECONDS);

//        TRSManager.initTRSServer(mqttClient);
        TRSManager.initTRSClient(mqttClient);

        // End of user code
    }

    public static void contextDestroyServletListener(ServletContextEvent servletContextEvent) 
    {
        
        // Start of user code contextDestroyed
        // TODO Implement code to shutdown connections to data backbone etc...
        log.info("Shutting down the adaptor");
        // FIXME Andrew@2018-07-30: disconnect from the MQTT broker
        // End of user code
    }

    public static ServiceProviderInfo[] getServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        ServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "ServiceProviderInfo[] getServiceProviderInfos(...)"
        serviceProviderInfos = AdaptorHelper.defaultSPInfo();
        // End of user code
        return serviceProviderInfos;
    }



    public static Object[] getPlan(HttpServletRequest httpServletRequest, final String serviceProviderId, final String planId)
    {
        Object[] aResource = null;
        
        // Start of user code getPlan
        log.trace("getPlan({}, {}) called", serviceProviderId, planId);
        // minimal impl to get the TRS provider going
        if (serviceProviderId.equals(AdaptorHelper.DEFAULT_SP_ID)) {
            final URI planURI = WarehouseControllerResourcesFactory.constructURIForPlan(
                    serviceProviderId, planId);
            if (TRSManager.getPlans().containsKey(planURI)) {
                aResource = TRSManager.getPlans().get(planURI);
                log.info("found a plan", aResource);
            } else {
                log.warn("a plan {} was not found", planId);
            }
        }
        // End of user code
        return aResource;
    }



    public static RegistrationRequest createRegistrationRequest(HttpServletRequest httpServletRequest, final RegistrationRequest aResource)
    {
        RegistrationRequest newResource = null;
        
        // Start of user code createRegistrationRequest
        // TODO Implement code to create a resource
        // End of user code
        return newResource;
    }




    public static String getETagFromPlan(final Plan aResource)
    {
        String eTag = null;
        // Start of user code getETagFromPlan
        eTag = AdaptorHelper.hexHashCodeFor(aResource);
        // End of user code
        return eTag;
    }
    public static String getETagFromRegistrationRequest(final RegistrationRequest aResource)
    {
        String eTag = null;
        // Start of user code getETagFromRegistrationRequest
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}