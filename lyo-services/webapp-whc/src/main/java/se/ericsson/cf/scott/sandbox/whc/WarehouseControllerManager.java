// Start of user code Copyright
/*!*****************************************************************************
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
import org.eclipse.paho.client.mqttv3.MqttException;
import se.ericsson.cf.scott.sandbox.whc.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.whc.ServiceProviderInfo;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.twins.RegistrationMessage;
import eu.scott.warehouse.domains.pddl.Step;


// Start of user code imports
import java.net.URI;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Executors;

import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig;
import se.ericsson.cf.scott.sandbox.whc.xtra.clients.TwinClient;
import se.ericsson.cf.scott.sandbox.whc.xtra.managers.MqttManager;
import se.ericsson.cf.scott.sandbox.whc.xtra.managers.PlanningManager;
import se.ericsson.cf.scott.sandbox.whc.xtra.managers.TRSManager;
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelper;
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.PlanRepository;
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.PlanRepositoryMem;
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinRepository;

import org.eclipse.paho.client.mqttv3.MqttClient;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;

import eu.scott.warehouse.lib.InstanceWithResources;
import eu.scott.warehouse.lib.OslcHelper;
import eu.scott.warehouse.lib.OslcHelpers;
import java.util.Optional;
// End of user code

// Start of user code pre_class_code
// End of user code

public class WarehouseControllerManager {

    // Start of user code class_attributes
    private static final Logger log = LoggerFactory.getLogger(WarehouseControllerManager.class);
    private static final ScheduledExecutorService execService = Executors.newScheduledThreadPool(1);
    private static TwinRepository twinRepository;
    private static PlanningManager planningManager;
    private static PlanRepository planRepository;
    // End of user code


    // Start of user code class_methods
    public static ScheduledExecutorService getExecService() {
        return execService;
    }

    public static TwinRepository getTwinRepository() {
        return twinRepository;
    }

    @NotNull
    public static PlanRepository getPlanRepository() {
        return planRepository;
    }
    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {

        // Start of user code contextInitializeServletListener
        AdaptorHelper.initLogger();
        log.debug("WHC contextInitializeServletListener START");
        AdaptorHelper.setContext(servletContextEvent.getServletContext());

        twinRepository = new TwinRepository();
        planRepository = new PlanRepositoryMem();
        planningManager = new PlanningManager(
            new PlanRequestHelper(new OslcHelper(WhcConfig.getBaseUri())), new TwinClient(),
            planRepository);

        getExecService().schedule(() -> {
            log.debug("Initialising an MQTT client");
            final MqttClient mqttClient = MqttManager.initMqttClient();

//            log.debug("Initialising a TRS Client");
//            try {
//                TRSManager.initTRSClient(mqttClient);
//            } catch (MqttException e) {
//                log.error("Failed to attach an MQTT TRS event listenter");
//            }

//            log.debug("Initialising a TRS Server");
//            TRSManager.initTRSServer(mqttClient);
            log.debug("WHC contextInitializeServletListener BACKGROUND TASK COMPLETE");
        }, 5, TimeUnit.SECONDS);
        log.debug("WHC contextInitializeServletListener COMPLETE");
        // End of user code
    }

    public static void contextDestroyServletListener(ServletContextEvent servletContextEvent)
    {

        // Start of user code contextDestroyed
        log.info("Shutting down the adaptor");
        MqttManager.disconnect();
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
        if (serviceProviderId.equals(WhcConfig.DEFAULT_SP_ID)) {
            final URI planURI = WarehouseControllerResourcesFactory.constructURIForPlan(
                serviceProviderId, planId);
            final Optional<InstanceWithResources<Plan>> plan = planRepository.getPlanByURI(planURI);
            if (plan.isPresent()) {
                aResource = plan.get().asResourceArray();
            } else {
                log.warn("Plan '{}' was not found", planId);
            }
        } else {
            log.warn("Wrong service provider: '{}'", serviceProviderId);
        }
        // End of user code
        return aResource;
    }



    public static RegistrationMessage createRegistrationMessage(HttpServletRequest httpServletRequest, final RegistrationMessage aResource)
    {
        RegistrationMessage newResource = null;

        // Start of user code createRegistrationMessage
        if(aResource != null) {
            log.info("Registering Twin {} ({})", aResource.getLabel(), aResource.getServiceProvider().getValue());
            getTwinRepository().registerTwin(aResource);
            newResource = aResource;
        } else {
            throw new IllegalArgumentException("The input must be of type twins:RegistrationMessage");
        }
        // End of user code
        return newResource;
    }




    public static String getETagFromPlan(final Plan aResource)
    {
        String eTag = null;
        // Start of user code getETagFromPlan
        eTag = OslcHelpers.hexHashCodeFor(aResource);
        // End of user code
        return eTag;
    }
    public static String getETagFromRegistrationMessage(final RegistrationMessage aResource)
    {
        String eTag = null;
        // Start of user code getETagFromRegistrationMessage
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}
