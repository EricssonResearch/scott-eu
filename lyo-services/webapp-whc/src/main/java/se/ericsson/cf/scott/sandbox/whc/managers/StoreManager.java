package se.ericsson.cf.scott.sandbox.whc.managers;

import java.io.IOException;
import org.apache.jena.atlas.web.HttpException;
import org.apache.jena.sparql.ARQException;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager.p;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class StoreManager {
    private final static Logger log = LoggerFactory.getLogger(StoreManager.class);

    public static Store getStore() {
        return store;
    }

    private static Store store;

    public static Store initLyoStore() {
        log.warn("https://github.com/EricssonResearch/scott-eu/issues/101 must be fixed first");
        try {
            store = StoreFactory.sparql(p("store.query"), p("store.update"));
            // TODO Andrew@2017-07-18: Remember to deactivate when switch to more persistent arch
            store.removeAll();
            return store;
        } catch (IOException | ARQException | HttpException e) {
            log.error(
                    "SPARQL Store failed to initialise with the URIs query={};update={}",
                    p("store.query"), p("store.update"), e
            );
            // TODO Andrew@2018-07-30: exception strategy
            throw new IllegalStateException(e);
        }
    }
}
