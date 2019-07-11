package eu.scottproject.wp10.gw.api;

import java.util.Collection;
import java.util.Map;
import org.eclipse.lyo.oslc4j.core.model.IResource;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public interface GatewayDiscoveryService {
    /**
     * @return a map with a Marshalling provider of a specific generic return type per message.
     */
    Map<String, Marshaller<? extends IResource>> getMarshallingProviders();

    /**
     * The method above won't give us the generic signatures. I expect this method just to return a
     * set of all generic types used by the marshallers shown above. I am planning to use it for
     * defining simple Service Providers to form a set of Tracked Resources.
     */
    Collection<Class<IResource>> getTrackedResourceClasses();

    /**
     * Should allow us to avoid scanning should any specific filtering need arise. For example,
     * Ricardo's patch would require this if inheritance is used (we'd need to supply subclasses).
     */
    Collection<Class<IResource>> getAllResourceClasses();
}
