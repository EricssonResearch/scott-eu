package se.ericsson.cf.scott.sandbox.twin;

import java.net.URI;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.store.ModelUnmarshallingException;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreAccessException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * TODO
 *
 * @version $version-stub$
 * @since TODO
 */
public class ServiceProviderRepositoryStoreImpl implements ServiceProviderRepository {
    private final static Logger log = LoggerFactory.getLogger(ServiceProviderRepositoryStoreImpl.class);
    private final Store store;
    private final URI twinsGraphURI;

    public ServiceProviderRepositoryStoreImpl(Store _store, URI _twinsGraphURI) {
        store = _store;
        twinsGraphURI = _twinsGraphURI;
    }

    @Override
    public Collection<ServiceProvider> getServiceProviders() {
        try {
            log.trace("Fetching ServiceProviders");
            final List<ServiceProvider> providers = store.getResources(twinsGraphURI, ServiceProvider.class);
            if(log.isTraceEnabled()) {
                providers.forEach(serviceProvider -> log.trace(serviceProvider.getTitle()));
            }
            log.trace("Returning ServiceProviders");
            return providers;
        } catch (StoreAccessException | ModelUnmarshallingException e) {
            log.error("Can't fetch ServiceProviders from {}", twinsGraphURI, e);
            throw new IllegalStateException(e);
        } catch (IllegalArgumentException e) {
            // TODO Andrew@2019-01-23: better handling, preferably with a more concrete exception
            return new LinkedList<>();
        }
    }

    @Override
    public ServiceProvider[] asSpArray() {
        return getServiceProviders().toArray(new ServiceProvider[0]);
    }

    @Override
    public void addServiceProvider(final ServiceProvider sp) {
        log.info("Persisting a ServiceProvider in the KB");
        try {
            store.updateResources(twinsGraphURI, sp);
            log.debug("Persisting COMPLETE");
        } catch (StoreAccessException e) {
            throw new IllegalStateException("Unable to persist the new SP in the KB");
        }
    }

}
