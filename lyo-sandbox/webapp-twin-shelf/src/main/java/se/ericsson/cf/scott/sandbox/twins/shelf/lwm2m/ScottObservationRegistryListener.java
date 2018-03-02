package se.ericsson.cf.scott.sandbox.twins.shelf.lwm2m;

import org.eclipse.leshan.core.node.LwM2mNode;
import org.eclipse.leshan.server.observation.Observation;
import org.eclipse.leshan.server.observation.ObservationRegistryListener;
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
public class ScottObservationRegistryListener implements ObservationRegistryListener {

    private final static Logger log = LoggerFactory.getLogger(ScottObservationRegistryListener
            .class);

    @Override
    public void newValue(Observation observation, LwM2mNode value) {

        log.info(
                "New notification from client {}:{}",
                observation.getClient().getEndpoint(),
                value
        );
    }

    @Override
    public void newObservation(Observation observation) {

        log.info("Observing resource {} from client {}",
                observation.getPath(),
                observation.getClient().getEndpoint()
        );
    }

    @Override
    public void cancelled(Observation observation) {
        // TODO: cancel ongoing observation
    }

}
