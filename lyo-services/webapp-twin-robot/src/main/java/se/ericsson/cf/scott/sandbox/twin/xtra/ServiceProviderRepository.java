package se.ericsson.cf.scott.sandbox.twin.xtra;

import java.util.Collection;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;

public interface ServiceProviderRepository {
    Collection<ServiceProvider> getServiceProviders();

    ServiceProvider[] asSpArray();

    void addServiceProvider(ServiceProvider sp);
}
