package se.ericsson.cf.scott.sandbox.twin;

import java.util.Collection;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;

/**
 * TBD
 *
 * @since FIXME
 */
public interface ServiceProviderRepository {
    Collection<ServiceProvider> getServiceProviders();

    ServiceProvider[] asSpArray();

    void addServiceProvider(ServiceProvider sp);
}
