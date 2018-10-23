package eu.scottproject.wp10.gw.svc.sample;

import eu.scottproject.wp10.gw.api.GatewayDiscoveryService;
import eu.scottproject.wp10.gw.api.Marshaller;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import org.eclipse.lyo.oslc4j.core.model.IResource;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class GatewayDiscoveryServiceImpl implements GatewayDiscoveryService {
    public Map<String, Marshaller<? extends IResource>> getMarshallingProviders() {
        final HashMap<String, Marshaller<? extends IResource>> m = new HashMap<>();
        m.put("sample", new SampleMarshaller());
        return m;
    }

    public Collection<IResource> getTrackedResourceClasses() {
        // TODO Andrew@2018-10-23: better than returning NULL but needs to be fixed later
        throw new UnsupportedOperationException();
    }

    public Collection<IResource> getAllResourceClasses() {
        throw new UnsupportedOperationException();
    }
}
