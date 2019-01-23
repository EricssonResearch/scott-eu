package se.ericsson.cf.scott.sandbox.twin;

import java.net.URI;
import java.util.Collection;
import java.util.Observable;
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
public class TwinRepositoryStoreImpl extends Observable implements TwinRepository {
    private final static Logger log = LoggerFactory.getLogger(TwinRepositoryStoreImpl.class);
    private final Store store;
    private final URI twinsGraphURI;

    public TwinRepositoryStoreImpl(Store _store, URI _twinsGraphURI) {
        store = _store;
        twinsGraphURI = _twinsGraphURI;
    }

    @Override
    public Collection<ServiceProvider> getServiceProviders() {
        try {
            return store.getResources(twinsGraphURI, ServiceProvider.class);
        } catch (StoreAccessException | ModelUnmarshallingException e) {
            log.error("Can't fetch ServiceProviders from {}", twinsGraphURI, e);
            throw new IllegalStateException(e);
        }
    }

    @Override
    public ServiceProvider[] asSpArray() {
        return getServiceProviders().toArray(new ServiceProvider[0]);
    }

    @Override
    public ServiceProvider registerTwinSP(final TwinsServiceProviderInfo spInfo) {
//        String key = spInfo.twinKind + '/' + spInfo.twinId;
        final ServiceProvider serviceProvider = TwinAdaptorHelper.registerProvider(spInfo);

        // FIXME Andrew@2019-01-21: use the Java Bean events instead
        TwinAdaptorHelper.changeHistories.addResource(serviceProvider);
//        notifyObservers(serviceProvider);

        return serviceProvider;
    }
}
