package se.ericsson.cf.scott.sandbox.twins.shelf.lwm2m;

import java.util.Arrays;
import org.eclipse.leshan.core.request.ObserveRequest;
import org.eclipse.leshan.core.request.ReadRequest;
import org.eclipse.leshan.core.request.WriteRequest;
import org.eclipse.leshan.core.response.LwM2mResponse;
import org.eclipse.leshan.server.californium.impl.LeshanServer;
import org.eclipse.leshan.server.client.Client;
import org.eclipse.leshan.server.client.ClientRegistryListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinManager;

/**
 * Created on 2018-03-02
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class ScottClientRegistryListener implements ClientRegistryListener {
    private final static Logger log = LoggerFactory.getLogger(ScottClientRegistryListener.class);
    private final LeshanServer server;

    public ScottClientRegistryListener(final LeshanServer server) {this.server = server;}

    @Override
    public void registered(Client client) {
        log.info("New client: " + client);
        boolean ReadAttempt = false;
        // Request for SmartObject
        // write the current time resource
//         WriteRequest write = new WriteRequest("/3/0/13", new LwM2mResource(13, Value
//                                                                             .newDateValue(new Date())),
//                                                                                    ContentFormat.TEXT, true);
//                                                                            log.info("Get Object Links: " + Arrays.asList(client.getObjectLinks()));
        if (Arrays.asList(client.getObjectLinks())
                  .stream()
                  .anyMatch(l -> "/6/0".equals(l.getUrl()))) {
            log.info("Object found!");
            try {
                ReadRequest read = new ReadRequest(6, 0, 1);
                log.debug("Send Read! " + read);
                Thread.sleep(5000L);
                LwM2mResponse response = server.send(client, read);
                log.debug("Request Sent!");

                log.info("Read response: " + response);
                if (response != null) {
                    log.debug("Good Read!");
                    ReadAttempt = true;
                } else {
                    log.info("Read Fails! Retrying...");
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (ReadAttempt) {
            try {
                log.debug("Observe Sent!");
                LwM2mResponse response = server.send(client, new ObserveRequest(6, 0, 0));
                Thread.sleep(500L);
                log.info("Observe response: " + response);
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

}
