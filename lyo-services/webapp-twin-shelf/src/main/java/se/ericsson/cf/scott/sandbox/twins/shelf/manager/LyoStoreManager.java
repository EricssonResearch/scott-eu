package se.ericsson.cf.scott.sandbox.twins.shelf.manager;

import java.io.IOException;
import org.apache.jena.sparql.ARQException;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twins.shelf.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinManager;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class LyoStoreManager {
    private final static Logger log = LoggerFactory.getLogger(LyoStoreManager.class);

    public static Store initLyoStore() {
        try {
            Store store = StoreFactory.sparql(
                    AdaptorHelper.p("store.query"), AdaptorHelper.p("store.update"));
            // TODO Andrew@2017-07-18: Remember to deactivate when switch to more persistent arch
            store.removeAll();
            return store;
        } catch (IOException | ARQException e) {
            log.error("SPARQL Store failed to initialise with the URIs query={};update={}",
                      AdaptorHelper.p("store.query"), AdaptorHelper.p("store.update"), e
            );
            // TODO Andrew@2018-07-29: rethink exception handling
            throw new IllegalStateException(e);
        }
    }
}
