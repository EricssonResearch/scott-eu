package se.ericsson.cf.scott.sandbox.twin;

import java.net.URI;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import org.checkerframework.checker.nullness.qual.NonNull;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.store.ModelUnmarshallingException;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreAccessException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
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
        log.trace("Adding a ServiceProvider");
        final Collection<ServiceProvider> serviceProviders = getServiceProviders();
        try {
            log.trace("Persisting the newly added ServiceProvider...");
            store.putResources(twinsGraphURI, addOrReplaceProvider(serviceProviders, sp));
            log.trace("Persisting the newly added ServiceProvider... SUCCESS!");
        } catch (StoreAccessException e) {
            throw new IllegalStateException("Unable to persist the new SP in the KB");
        }
    }

    private Collection<ServiceProvider> addOrReplaceProvider(
        @NonNull final Collection<ServiceProvider> serviceProviders, @NonNull final ServiceProvider newProvider) {
        if (newProvider.getAbout() == null) {
            throw new IllegalArgumentException("The about URI of the ServiceProvider cannot be NULL");
        }
        Collection<ServiceProvider> providers = serviceProviders;
        if (providers.stream().anyMatch(p -> Objects.equals(p.getAbout(), newProvider.getAbout()))) {
            log.debug("Replacing an existing ServiceProvider <{}> ", newProvider.getAbout());
            // TODO Andrew@2019-01-24: put a printing tracer for equality
            providers = serviceProviders.stream()
                                        .filter(p -> !eqURI(newProvider, p))
                                        .collect(Collectors.toList());
        }
        log.trace("Adding a new ServiceProvider to the list");
        providers.add(newProvider);
        return providers;

        // TODO Andrew@2019-01-24: later https://github.com/eclipse/lyo.core/issues/64#issuecomment-457157158
        /*if (!serviceProviders.add(sp)) {
            serviceProviders.remove(sp);
            serviceProviders.add(sp);
        }*/
    }

    private boolean eqURI(final @NonNull ServiceProvider newProvider, final @NonNull ServiceProvider p) {
        final boolean sameURI = Objects.equals(p.getAbout(), newProvider.getAbout());
        log.trace("SP '{}' == '{}': {}", p.getAbout(), newProvider.getAbout(), sameURI);
        return sameURI;
    }
}
