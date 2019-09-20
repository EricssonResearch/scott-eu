// Start of user code Copyright
/*!******************************************************************************
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

import java.util.Set;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import eu.scott.warehouse.domains.scott.ActionExecutionReport;
import eu.scott.warehouse.domains.twins.DeviceRegistrationMessage;
import eu.scott.warehouse.domains.twins.PlanExecutionRequest;

// Start of user code imports
import com.google.common.base.Strings;
import java.net.URISyntaxException;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import org.apache.commons.lang.WordUtils;
import org.eclipse.lyo.oslc4j.client.OslcClient;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twin.servlet.TwinsServiceProvidersFactory;
import se.ericsson.cf.scott.sandbox.twin.xtra.PlanExecutionService;
import se.ericsson.cf.scott.sandbox.twin.xtra.TwinAdaptorHelper;
import se.ericsson.cf.scott.sandbox.twin.xtra.factory.NaiveTrsFactories;
import se.ericsson.cf.scott.sandbox.twin.xtra.plans.TrsActionStatusHandler;
import eu.scott.warehouse.lib.trs.ConcurrentMqttAppender;
import eu.scott.warehouse.lib.trs.ConcurrentRedisOrderGenerator;
import redis.clients.jedis.Jedis;
import eu.scott.warehouse.lib.trs.ITrsLogAppender;
// End of user code

// Start of user code pre_class_code
// End of user code

public class TwinManager {

    // Start of user code class_attributes
    public final static String PACKAGE_ROOT = TwinManager.class.getPackage().getName();
    private final static Logger log = LoggerFactory.getLogger(TwinManager.class);
    private static Random r;
    private static PlanExecutionService planExecutionService;
//    private static final TrsEventKafkaPublisher kafkaPublisher = new TrsEventKafkaPublisher(
//        "kafka:9092", "test");
    // End of user code

    // Start of user code class_methods
//    public static TrsEventKafkaPublisher getKafkaPublisher() {
//        return kafkaPublisher;
//    }

    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {

        // Start of user code contextInitializeServletListener
        NaiveTrsFactories.activate();
        log.info("Twin {} is starting", TwinAdaptorHelper.getTwinUUID());
        TwinAdaptorHelper.setServletContext(servletContextEvent.getServletContext());
        r = new Random();


        log.debug("Initialising the KB");

        final Jedis jedis = new Jedis("redis.svc");

        final String slot = System.getenv("CONTAINER_SLOT");
        if(Strings.isNullOrEmpty(slot) || Integer.parseUnsignedInt(slot) != 1) {
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            log.info("Non-master node initialising...");
            TwinAdaptorHelper.initStore(false);
        } else {
            log.warn("Clearing the state of the triplestore");
            TwinAdaptorHelper.initStore(true);
            log.warn("Clearing Redis keys");
            Set<String> keys = jedis.keys("order:*");
            for (String key : keys) {
                jedis.del(key);
            }
        }

        final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
        // TODO Andrew@2019-07-15: replace with something concurrent
        final ConcurrentRedisOrderGenerator orderGenerator = new ConcurrentRedisOrderGenerator(
            jedis);
        final ITrsLogAppender logAppender = new ConcurrentMqttAppender(
            TwinAdaptorHelper.getMqttClient(), orderGenerator);
        final TrsActionStatusHandler statusHandler = new TrsActionStatusHandler(logAppender,
            TwinAdaptorHelper.getExecutionReportRepository());
        final OslcClient client = new OslcClient();
        planExecutionService = new PlanExecutionService(executorService, client, statusHandler);
       // End of user code
    }

    public static void contextDestroyServletListener(ServletContextEvent servletContextEvent)
    {

        // Start of user code contextDestroyed

        log.info("Destroying the servlet");
        try {
            TwinAdaptorHelper.getTrsClientManager().unregisterTwinAndDisconnect();
            TwinAdaptorHelper.getMqttGateway().disconnect();
        } catch (MqttException e) {
            log.error("Failed to disconnect from the MQTT broker");
        }

        // End of user code
    }

    public static TwinsServiceProviderInfo[] getTwinsServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        TwinsServiceProviderInfo[] serviceProviderInfos = {};

        // Start of user code "TwinsServiceProviderInfo[] getTwinsServiceProviderInfos(...)"
        // End of user code
        return serviceProviderInfos;
    }
    public static IndependentServiceProviderInfo[] getIndependentServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        IndependentServiceProviderInfo[] serviceProviderInfos = {};

        // Start of user code "IndependentServiceProviderInfo[] getIndependentServiceProviderInfos(...)"
        // End of user code
        return serviceProviderInfos;
    }

    public static PlanExecutionRequest createPlanExecutionRequest(HttpServletRequest httpServletRequest, final PlanExecutionRequest aResource, final String twinKind, final String twinId)
    {
        PlanExecutionRequest newResource = null;

        // Start of user code createPlanExecutionRequest
        log.info("Incoming plan: {}", aResource.getPlan().getValue());
        newResource = aResource;
        planExecutionService.fulfillRequest(aResource, twinKind, twinId);
        // End of user code
        return newResource;
    }





    public static ActionExecutionReport getActionExecutionReport(HttpServletRequest httpServletRequest, final String twinKind, final String twinId, final String actionExecutionReportId)
    {
        ActionExecutionReport aResource = null;

        // Start of user code getActionExecutionReport
        // TODO Implement code to return a resource
        // End of user code
        return aResource;
    }



    public static DeviceRegistrationMessage createDeviceRegistrationMessage(HttpServletRequest httpServletRequest, final DeviceRegistrationMessage aResource)
    {
        DeviceRegistrationMessage newResource = null;

        // Start of user code createDeviceRegistrationMessage
        log.info("Registering a twin: {}", aResource.toString());
        final String twinKind = aResource.getTwinType();
        String twinId = aResource.getTwinId();
        if (twinId == null) {
            twinId = String.valueOf(r.nextInt(10000));
        }
        final String name = String.format("%s Twin '%s'", WordUtils.capitalize(twinKind), twinId);
        final TwinsServiceProviderInfo spInfo = new TwinsServiceProviderInfo(name, twinKind, twinId);
        try {
            final ServiceProvider serviceProvider = TwinsServiceProvidersFactory.createServiceProvider(spInfo);
            TwinAdaptorHelper.getTwins().addServiceProvider(serviceProvider);
        } catch (OslcCoreApplicationException | URISyntaxException e) {
            log.error("Failed to create an SP", e);
        }

        newResource = aResource;
        newResource.setTwinId(spInfo.twinId);

        // End of user code
        return newResource;
    }




    public static String getETagFromActionExecutionReport(final ActionExecutionReport aResource)
    {
        String eTag = null;
        // Start of user code getETagFromActionExecutionReport
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
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
