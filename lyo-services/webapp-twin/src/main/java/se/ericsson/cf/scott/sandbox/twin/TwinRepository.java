package se.ericsson.cf.scott.sandbox.twin;

import java.util.Collection;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;

/**
 * TBD
 *
 * @since FIXME
 */
public interface TwinRepository {
    Collection<ServiceProvider> getServiceProviders();

    ServiceProvider[] asSpArray();

    ServiceProvider registerTwinSP(TwinsServiceProviderInfo spInfo);
}
