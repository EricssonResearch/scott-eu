package se.ericsson.cf.scott.sandbox.twin.xtra.factory;

import com.google.common.collect.ImmutableList;
import javax.inject.Inject;
import javax.ws.rs.core.UriBuilder;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.trs.server.InmemPagedTrs;
import org.eclipse.lyo.oslc4j.trs.server.PagedTrs;
import org.eclipse.lyo.oslc4j.trs.server.TrsEventHandler;
import org.eclipse.lyo.oslc4j.trs.server.service.TrackedResourceSetService;
import org.glassfish.hk2.api.Factory;
import org.glassfish.hk2.api.InstantiationService;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * The whole nested static class thing is done because I can't implement multiple interfaces with
 * the same erasure type.
 */
public class NaiveTrsFactories {
    
    private final static Logger log = LoggerFactory.getLogger(PagedTrsFactory.class);

    private static boolean active = false;
    private static InmemPagedTrs pagedTrs;

    public static void activate() {
        active = true;
    }

    public static class PagedTrsFactory implements Factory<PagedTrs> {
        @Override
        public PagedTrs provide() {
            return getInmemPagedTrs();
        }

        @Override
        public void dispose(final PagedTrs instance) {
            log.debug("{} is getting disposed", instance);
        }
    }

    public static class ResourceEventHandlerFactory implements Factory<TrsEventHandler> {

        @Inject InstantiationService instantiationService;

        @Override
        public TrsEventHandler provide() {
            // TODO Andrew@2019-05-02: try to decouple those things and rely on InstantiationService
            return getInmemPagedTrs();
        }

        @Override
        public void dispose(final TrsEventHandler instance) {

        }
    }

    @NotNull
    private static InmemPagedTrs getInmemPagedTrs() {
        if (!active) {
            log.error("PagedTrs was requested before the context has been initialised");
            throw new IllegalStateException(
                "Cannot provide instances before the context has been initialised");
        }
        if(pagedTrs == null) {
            log.debug("Initialising 'InmemPagedTrs' instance");
            // not thread-safe
            pagedTrs = new InmemPagedTrs(5, 5,
                UriBuilder.fromUri(OSLC4JUtils.getServletURI()).path(
                    TrackedResourceSetService.RESOURCE_PATH).build(),
                ImmutableList.of());
        }
        return pagedTrs;
    }
}
