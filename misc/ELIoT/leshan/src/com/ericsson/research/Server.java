package com.ericsson.research;

import org.eclipse.leshan.core.node.LwM2mNode;
import org.eclipse.leshan.core.request.ObserveRequest;
import org.eclipse.leshan.core.request.ReadRequest;
import org.eclipse.leshan.core.response.LwM2mResponse;
import org.eclipse.leshan.server.californium.LeshanServerBuilder;
import org.eclipse.leshan.server.californium.impl.LeshanServer;
import org.eclipse.leshan.server.client.Client;
import org.eclipse.leshan.server.client.ClientRegistryListener;
import org.eclipse.leshan.server.observation.Observation;
import org.eclipse.leshan.server.observation.ObservationRegistryListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;

public class Server {
    public static void main(String[] args) {

        // build the lwm2m server
        final LeshanServer server = new LeshanServerBuilder().build();
        Logger logger = LoggerFactory.getLogger(com.ericsson.research.Server.class);

        // listen for new client registrations
        server.getClientRegistry().addListener(new ClientRegistryListener() {

            @Override
            public void registered(Client client) {
                logger.info("New client: " + client);
                boolean ReadAttempt =false;
                // Request for SmartObject
                // write the current time resource
                // WriteRequest write = new WriteRequest("/3/0/13", new LwM2mResource(13, Value.newDateValue(new Date())),
                //        ContentFormat.TEXT, true);
                logger.info("Get Object Links: " +  Arrays.asList(client.getObjectLinks()));
                if (Arrays.asList(client.getObjectLinks()).stream().anyMatch(l -> "/6/0".equals(l.getUrl()))) {
                    logger.info("Object found!");
                    try {
                            ReadRequest read = new ReadRequest(6, 0, 1);
                            logger.debug("Send Read! " + read);
                            Thread.sleep(5000L);
                            LwM2mResponse response = server.send(client, read);
                            logger.debug("Request Sent!");

                            logger.info("Read response: " + response);
                            if (response != null) {
                                logger.debug("Good Read!");
                                ReadAttempt = true;
                            } else {
                                logger.info("Read Fails! Retrying...");
                            }
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }


                if (ReadAttempt) {
                	try {
                logger.debug("Observe Sent!");
                LwM2mResponse response = server.send(client, new ObserveRequest(6, 0, 0));
                Thread.sleep(500L);
                logger.info("Observe response: " + response);
                	} catch (InterruptedException e) {
                	e.printStackTrace();
                }
                }
            }

            @Override
            public void updated(Client client) {
                // TODO: node status update
            }

            @Override
            public void unregistered(Client client) {
                // TODO: node unregistered Flag
            }

        });


        // listen for observe notifications
        server.getObservationRegistry().addListener(new ObservationRegistryListener() {

            @Override
            public void newValue(Observation observation, LwM2mNode value) {

                logger.info("New notification from client " + observation.getClient().getEndpoint() + ": "
                        + value);
            }

            @Override
            public void newObservation(Observation observation) {

                logger.info("Observing resource " + observation.getPath() + " from client "
                        + observation.getClient().getEndpoint());
            }

            @Override
            public void cancelled(Observation observation) {
                // TODO: cancel ongoing observation
            }

        });

        server.start();
    }

}
