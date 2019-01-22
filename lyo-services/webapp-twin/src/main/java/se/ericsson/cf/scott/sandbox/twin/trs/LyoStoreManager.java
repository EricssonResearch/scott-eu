package se.ericsson.cf.scott.sandbox.twin.trs;

import java.io.IOException;
import org.apache.jena.sparql.ARQException;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static se.ericsson.cf.scott.sandbox.twin.TwinAdaptorHelper.p;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class LyoStoreManager {
    private final static Logger log = LoggerFactory.getLogger(LyoStoreManager.class);

    // TODO Andrew@2018-07-29: merge with the Shelf twin LyoStoreManager
    public static Store initLyoStore() {
        try {
            final Store store = StoreFactory.sparql(p("store.query"), p("store.update"));
            // TODO Andrew@2017-07-18: Remember to deactivate when switch to more persistent arch
            store.removeAll();
            return store;
        } catch (IOException | ARQException e) {
            log.error("SPARQL Store failed to initialise with the URIs query={};update={}",
                                                                        p("store.query"), p("store.update"), e);
            // TODO Andrew@2018-07-29: rethink exception management
            throw new IllegalStateException(e);
        }
    }
}
