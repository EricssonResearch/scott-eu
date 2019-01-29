package se.ericsson.cf.scott.sandbox.twin.trs;

import java.io.IOException;
import org.apache.jena.sparql.ARQException;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static se.ericsson.cf.scott.sandbox.twin.TwinAdaptorHelper.p;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class LyoStoreManager {
    private final static Logger log = LoggerFactory.getLogger(LyoStoreManager.class);

    // TODO Andrew@2018-07-29: merge with the Shelf twin LyoStoreManager
    // TODO Andrew@2019-01-23: move to the TwinAdaptorHelper? Or to the sandbox library?
    public static Store initLyoStore(final boolean wipeOnStartup) {
        final String query_endpoint = p("store.query");
        final String update_endpoint = p("store.update");
        log.info("Initialising Lyo Store");
        log.info("SPARQL endpoints: query={}; update={}", query_endpoint, update_endpoint);
        try {
            final Store store = StoreFactory.sparql(query_endpoint, update_endpoint);
            if (wipeOnStartup) {
                log.warn("Erasing the whole dataset");
                store.removeAll();
            }
            return store;
        } catch (IOException | ARQException e) {
            log.error("SPARQL Store failed to initialise with the URIs query={};update={}", query_endpoint,
                      update_endpoint, e);
            // TODO Andrew@2018-07-29: rethink exception management
            throw new IllegalStateException(e);
        }
    }
}
