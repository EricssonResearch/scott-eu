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

package se.ericsson.cf.scott.sandbox.twins.shelf;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;
import java.util.List;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import se.ericsson.cf.scott.sandbox.twins.shelf.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.twins.shelf.ServiceProviderInfo;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.PlanExecutionResult;
import eu.scott.warehouse.domains.pddl.Step;


// Start of user code imports
import com.google.common.collect.Lists;
import com.google.common.collect.Queues;
import java.io.IOException;
import java.util.Collection;
import java.util.Random;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import javax.servlet.ServletContext;
import org.apache.jena.sparql.ARQException;
import org.eclipse.leshan.core.node.LwM2mNode;
import org.eclipse.leshan.core.request.AbstractDownlinkRequest;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreFactory;
import org.eclipse.lyo.trs.consumer.config.TrsConsumerConfiguration;
import org.eclipse.lyo.trs.consumer.config.TrsProviderConfiguration;
import org.eclipse.lyo.trs.consumer.handlers.TrsProviderHandler;
import org.eclipse.lyo.trs.consumer.util.TrsBasicAuthOslcClient;
import org.eclipse.lyo.trs.consumer.util.TrsConsumerUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twins.shelf.trs.PlanChangeEventListener;
// End of user code

// Start of user code pre_class_code
// End of user code

public class ShelfTwinManager {

    // Start of user code class_attributes
    private final static String PACKAGE_ROOT = ShelfTwinManager.class.getPackage().getName();
    private final static Logger log          = LoggerFactory.getLogger(ShelfTwinManager.class);
    private static Store          store;
    private static ServletContext context;
    public final static BlockingQueue<LwM2mNode>               newlyObservedValues  = Queues
            .newLinkedBlockingDeque();
    public final static BlockingQueue<AbstractDownlinkRequest> pendingWriteRequests = Queues
            .newLinkedBlockingDeque();
    public final static ExecutorService planExecutorSvc = Executors.newSingleThreadExecutor();
    // End of user code


    // Start of user code class_methods
    private static String parameterFQDN(final String s) {
        return PACKAGE_ROOT + "." + s;
    }

    private static String p(final String s) {
        return context.getInitParameter(parameterFQDN(s));
    }
    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {

        // Start of user code contextInitializeServletListener
        context = servletContextEvent.getServletContext();
        try {
            store = StoreFactory.sparql(p("store.query"), p("store.update"));
            // TODO Andrew@2017-07-18: Remember to deactivate when switch to more persistent arch
            store.removeAll();
        } catch (IOException | ARQException e) {
            log.error("SPARQL Store failed to initialise with the URIs query={};update={}",
                    p("store.query"),
                    p("store.update"),
                    e
            );
        }

        final TrsConsumerConfiguration consumerConfig = new TrsConsumerConfiguration(
                p("store.query"),
                // disable the SPARQL triplestore update
                null, null, null, new TrsBasicAuthOslcClient(), "trs-consumer-whc",
                // nothing fancy is really needed on the twins
                Executors.newSingleThreadScheduledExecutor()
        );
        // FIXME Andrew@2018-06-19: extract to a property
        final String warehouseTrsUri = "http://sandbox-whc:8080/services/trs";
        final String basicAuthUsername = null;
        final String basicAuthPassword = null;
        final String mqttBroker = p("trs.mqtt.broker");
        final String mqttTopic = p("trs.mqtt.topic");
        final Collection<TrsProviderConfiguration> providerConfigs = Lists.newArrayList(
                new TrsProviderConfiguration(warehouseTrsUri, basicAuthUsername, basicAuthPassword,
                                             mqttBroker, mqttTopic
                ));
        final List<TrsProviderHandler> handlers = TrsConsumerUtils.buildHandlersSequential(
                consumerConfig, providerConfigs);

        // attach our own listener to use "TRS everywhere"
        final PlanChangeEventListener listener = new PlanChangeEventListener();
        for (TrsProviderHandler handler : handlers) {
            handler.attachListener(listener);
        }

        final Random random = new Random(System.currentTimeMillis());
        final Integer updateInterval = Integer.valueOf(p("update.interval"));
        for (TrsProviderHandler handler : handlers) {
            consumerConfig.getScheduler()
                          .scheduleAtFixedRate(handler,
                                  random.nextInt(updateInterval),
                                  updateInterval,
                                  TimeUnit.SECONDS
                          );
        }

        /*+++++++++++++++++++++++++++++++++++++++++++++++++
        Leshan server
        ++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*        // build the lwm2m server
        final LeshanServer server = new LeshanServerBuilder().build();

        // listen for new client registrations
        server.getClientRegistry().addListener(new ScottClientRegistryListener(server));
        // listen for observe notifications
        server.getObservationRegistry().addListener(new ScottObservationRegistryListener());

        server.start();*/


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
        // FIXME Andrew@2018-02-28: add ctor to Lyo class
        final ServiceProviderInfo serviceProviderInfo = new ServiceProviderInfo();
        serviceProviderInfo.serviceProviderId = "default";
        serviceProviderInfo.name = "Default Service Provider";
        serviceProviderInfos = new ServiceProviderInfo[]{serviceProviderInfo};
        // End of user code
        return serviceProviderInfos;
    }



    public static PlanExecutionResult getPlanExecutionResult(HttpServletRequest httpServletRequest, final String serviceProviderId, final String planExecutionResultId)
    {
        PlanExecutionResult aResource = null;

        // Start of user code getPlanExecutionResult
        // TODO Implement code to return a resource
        // End of user code
        return aResource;
    }




    public static String getETagFromPlanExecutionResult(final PlanExecutionResult aResource)
    {
        String eTag = null;
        // Start of user code getETagFromPlanExecutionResult
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}
