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

package se.ericsson.cf.scott.sandbox.twin;

import java.util.Random;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;
import java.util.List;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import se.ericsson.cf.scott.sandbox.twin.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.twin.TwinsServiceProviderInfo;
import se.ericsson.cf.scott.sandbox.twin.IndependentServiceProviderInfo;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.twins.DeviceRegistrationMessage;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.twins.PlanExecutionRequest;
import eu.scott.warehouse.domains.pddl.Step;

// Start of user code imports
import org.apache.commons.lang.WordUtils;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import se.ericsson.cf.scott.sandbox.twin.clients.TwinRegistrationClient;
import se.ericsson.cf.scott.sandbox.twin.servlet.TwinsServiceProvidersFactory;
import se.ericsson.cf.scott.sandbox.twin.trs.TrsMqttClientManager;
import se.ericsson.cf.scott.sandbox.twin.trs.TwinAckRegistrationAgent;

//import se.ericsson.cf.scott.sandbox.twin.ros.RobotClientNode;

import com.google.common.base.Strings;
import com.google.common.collect.ImmutableList;
import eu.scott.warehouse.lib.MqttClientBuilder;
import eu.scott.warehouse.lib.MqttTopics;
import eu.scott.warehouse.lib.TrsMqttGateway;
import java.util.UUID;
import javax.servlet.ServletContext;

import org.eclipse.lyo.client.oslc.OslcClient;
import org.eclipse.lyo.store.Store;
import com.hazelcast.core.HazelcastInstance;
import com.hazelcast.core.IMap;
import eu.scott.warehouse.lib.hazelcast.HazelcastFactory;
import com.hazelcast.core.EntryEvent;
import com.hazelcast.map.listener.EntryAddedListener;
import com.hazelcast.map.listener.MapListener;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
// End of user code

// Start of user code pre_class_code
// End of user code

public class TwinManager {

    // Start of user code class_attributes
    public final static String PACKAGE_ROOT = TwinManager.class.getPackage().getName();
    private final static Logger log = LoggerFactory.getLogger(TwinManager.class);
    private final static UUID uuid = UUID.randomUUID();
    private static String trsTopic;
    private static TrsMqttClientManager trsClientManager;
    private static Store store;
    private static ServletContext servletContext;
    private static TrsMqttGateway mqttGateway;
    private static HazelcastInstance hc;
    private static IMap<String, TwinsServiceProviderInfo> twinProviderInfo;
    private static Random r;
    // End of user code
    
    
    // Start of user code class_methods
    @NotNull
    public static String getTwinUUID() {
        return "twn-" + uuid.toString();
    }

    public static String getTrsTopic() {
        if(Strings.isNullOrEmpty(trsTopic)) {
            log.warn("The TRS topic was requested before it was set");
        }
        return trsTopic;
    }

    public static void setTrsTopic(String trsTopic) {
        TwinManager.trsTopic = trsTopic;
    }

    public static TrsMqttClientManager getTrsClientManager() {
        return trsClientManager;
    }

    public static void setTrsClientManager(TrsMqttClientManager trsClientManager) {
        TwinManager.trsClientManager = trsClientManager;
    }

    public static ServletContext getServletContext() {
        return servletContext;
    }

    private static void registerTwins() {
        final OslcClient client = new OslcClient();
        final TwinRegistrationClient registrationClient = new TwinRegistrationClient(
            client, "http://sandbox-whc:8080/services/service2/registrationRequests/register");

        for(String id: ImmutableList.of("r1", "r2", "r3")) {
            registrationClient.registerTwin("robot", id);
        }

    }

    private static void setStore(final Store store) {
        TwinManager.store = store;
    }

    private static void registerProvider(final TwinsServiceProviderInfo info) {
        try {
            log.info("Registering provider: {}", info);
            final ServiceProvider robotSP = ServiceProviderCatalogSingleton.createTwinServiceProvider(
                info);
            ServiceProviderCatalogSingleton.registerTwinsServiceProvider(
                null, robotSP, info.twinKind, info.twinId);
        } catch (URISyntaxException | OslcCoreApplicationException e) {
            log.error("Cannot register the Robot SP", e);
        }
    }
    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {
        
        // Start of user code contextInitializeServletListener
        log.info("Twin {} is starting", getTwinUUID());
        servletContext = servletContextEvent.getServletContext();
//        final Store store = LyoStoreManager.initLyoStore();
//        setStore(store);

//        RosManager.runRosNode();
//        new Thread(RobotTwinManager::runRosNode).run();
//        RosManager.execMainNode();

        final String mqttBroker = AdaptorHelper.p("trs.mqtt.broker");
//        final TrsMqttClientManager trsClientManager = new TrsMqttClientManager(mqttBroker);
//        setTrsClientManager(trsClientManager);
//        new Thread(trsClientManager::connectAndSubscribeToPlans).run();
        // FIXME Andrew@2018-07-31: remove non-gateway based code
//        try {
//            mqttGateway = new MqttClientBuilder().withBroker(mqttBroker)
//                                                 .withId(getTwinUUID())
//                                                 .withRegistration(new TwinAckRegistrationAgent(
//                                                     MqttTopics.WHC_PLANS))
//                                                 .build();
//        } catch (MqttException e) {
//            log.error("Failed to initialise the MQTT gateway", e);
//        }

        hc = HazelcastFactory.INSTANCE.instanceFromDefaultXmlConfig();

        twinProviderInfo = hc.getMap("twin-providers");

        twinProviderInfo.addEntryListener((EntryAddedListener) event -> {
            try {
                log.info("New Robot SP info map entry received '{}:{}'", event.getKey(),
                         event.getValue()
                );
                final TwinsServiceProviderInfo info = (TwinsServiceProviderInfo) event.getValue();
                if (!ServiceProviderCatalogSingleton.containsTwinServiceProvider(
                    info.twinKind, info.twinId)) {
                    registerProvider(info);
                } else {
                    log.debug(
                        "SP {}/{} is already registered, skipping", info.twinKind, info.twinId);
                }
            } catch (Exception e) {
                log.warn("Unhandled exception in EntryAddedListener", e);
            }

        }, true);

//        registerTwins();

        r = new Random();
        // End of user code
    }

    public static void contextDestroyServletListener(ServletContextEvent servletContextEvent) 
    {
        
        // Start of user code contextDestroyed

        log.info("Destroying the servlet");
        try {
            mqttGateway.disconnect();
        } catch (MqttException e) {
            log.error("Failed to disconnect from the MQTT broker");
        }
//        getTrsClientManager().unregisterTwinAndDisconnect();

        // End of user code
    }

    public static TwinsServiceProviderInfo[] getTwinsServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        TwinsServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "TwinsServiceProviderInfo[] getTwinsServiceProviderInfos(...)"
        // needed if the new node joins
        serviceProviderInfos = twinProviderInfo.values().toArray(new TwinsServiceProviderInfo[0]);
        // End of user code
        return serviceProviderInfos;
    }
    public static IndependentServiceProviderInfo[] getIndependentServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        IndependentServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "IndependentServiceProviderInfo[] getIndependentServiceProviderInfos(...)"
        // TODO Implement code to return the set of ServiceProviders
        // End of user code
        return serviceProviderInfos;
    }

    public static PlanExecutionRequest createPlanExecutionRequest(HttpServletRequest httpServletRequest, final PlanExecutionRequest aResource, final String twinKind, final String twinId)
    {
        PlanExecutionRequest newResource = null;
        
        // Start of user code createPlanExecutionRequest
        // TODO Implement code to create a resource
        // End of user code
        return newResource;
    }



    public static DeviceRegistrationMessage createDeviceRegistrationMessage(HttpServletRequest httpServletRequest, final DeviceRegistrationMessage aResource)
    {
        DeviceRegistrationMessage newResource = null;
        
        // Start of user code createDeviceRegistrationMessage
        log.info("Registering a twin: {}", aResource.toString());
        final TwinsServiceProviderInfo spInfo = new TwinsServiceProviderInfo();
        spInfo.twinKind = aResource.getTwinType();
        spInfo.twinId = aResource.getTwinId();
        if (spInfo.twinId == null) {
            spInfo.twinId = String.valueOf(r.nextInt(10000));
        }
        spInfo.name = String.format(
            "%s Twin '%s'", WordUtils.capitalize(spInfo.twinKind), spInfo.twinId);
        registerProvider(spInfo);
        twinProviderInfo.put(spInfo.twinKind + '/' + spInfo.twinId, spInfo);

        newResource = aResource;
        newResource.setTwinId(spInfo.twinId);

        // End of user code
        return newResource;
    }




    public static String getETagFromDeviceRegistrationMessage(final DeviceRegistrationMessage aResource)
    {
        String eTag = null;
        // Start of user code getETagFromDeviceRegistrationMessage
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }
    public static String getETagFromPlanExecutionRequest(final PlanExecutionRequest aResource)
    {
        String eTag = null;
        // Start of user code getETagFromPlanExecutionRequest
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}
