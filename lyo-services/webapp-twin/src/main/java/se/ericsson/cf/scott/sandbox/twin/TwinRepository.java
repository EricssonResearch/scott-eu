package se.ericsson.cf.scott.sandbox.twin;

import java.net.URI;
import java.util.Collection;
import java.util.Observable;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.store.ModelUnmarshallingException;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreAccessException;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class TwinRepository extends Observable {

    private final Store store;
    private final URI twinsGraphURI;

    public TwinRepository(Store _store, URI _twinsGraphURI) {
        store = _store;
        twinsGraphURI = _twinsGraphURI;
    }

    public Collection<ServiceProvider> getServiceProviders()
        throws StoreAccessException, ModelUnmarshallingException {
        return store.getResources(twinsGraphURI, ServiceProvider.class);
    }

    public ServiceProvider[] asSpArray() {
        // FIXME Andrew@2019-01-21: deal with the lack of the TwinsServiceProviderInfo itself
        try {
            return getServiceProviders().toArray(new ServiceProvider[0]);
        } catch (StoreAccessException | ModelUnmarshallingException e) {
            throw new IllegalStateException(e);
        }
    }

    public ServiceProvider registerTwinSP(final TwinsServiceProviderInfo spInfo) {
//        String key = spInfo.twinKind + '/' + spInfo.twinId;
        final ServiceProvider serviceProvider = TwinAdaptorHelper.registerProvider(spInfo);

        // FIXME Andrew@2019-01-21: use the Observable instead
        TwinAdaptorHelper.changeHistories.addResource(serviceProvider);
//        notifyObservers(serviceProvider);

        return serviceProvider;
    }
}
