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

package se.ericsson.cf.scott.sandbox.executor;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;
import java.util.List;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import se.ericsson.cf.scott.sandbox.executor.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.executor.ServiceProviderInfo;


// Start of user code imports
import java.util.UUID;
import javax.servlet.ServletContext;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;

import eu.scott.warehouse.lib.TrsMqttGateway;
import se.ericsson.cf.scott.sandbox.executor.xtra.ExecutorServiceHelper;
import se.ericsson.cf.scott.sandbox.executor.xtra.mqtt.MqttManager;
// End of user code

// Start of user code pre_class_code
// End of user code

public class PlanExecutorManager {

    // Start of user code class_attributes
    private final static Logger log = LoggerFactory.getLogger(PlanExecutorManager.class);
    private static String adaptorId = "exec-" + UUID.randomUUID();
    // FIXME Andrew@2019-01-22: init
    private static String mqttBroker;
    private static TrsMqttGateway mqttGateway;
    private static ServletContext servletContext;
    private static MqttClient mqttClient;
    // End of user code
    
    
    // Start of user code class_methods
    public static ServletContext getServletContext() {
        return servletContext;
    }
    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {
        
        // Start of user code contextInitializeServletListener
        log.info("Plan Executor {} is starting", adaptorId);
        servletContext = servletContextEvent.getServletContext();
        if (log.isTraceEnabled()) {
            ExecutorServiceHelper.dumpInitParams(servletContext);
            ExecutorServiceHelper.dumpEnvVars();
        }

        mqttBroker = ExecutorServiceHelper.p("trs.mqtt.broker");
        mqttClient = MqttManager.initMqttClient(mqttBroker, adaptorId);
        // End of user code
    }

    public static void contextDestroyServletListener(ServletContextEvent servletContextEvent) 
    {
        
        // Start of user code contextDestroyed
        log.info("Plan Executor is shutting down");
        try {
            if (mqttGateway != null) {
                mqttGateway.disconnect();
            }
        } catch (MqttException e) {
            log.error("The MQTT gateway could not shut down cleanly");
            log.debug("The MQTT gateway could not shut down cleanly", e);
        }
        // End of user code
    }

    public static ServiceProviderInfo[] getServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        ServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "ServiceProviderInfo[] getServiceProviderInfos(...)"
        log.warn("The REST I/F is not implemented yet");
        // End of user code
        return serviceProviderInfos;
    }








}
