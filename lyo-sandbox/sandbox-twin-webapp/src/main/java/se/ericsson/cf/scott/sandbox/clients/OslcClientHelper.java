package se.ericsson.cf.scott.sandbox.clients;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import javax.ws.rs.core.UriBuilder;
import net.oauth.OAuthException;
import org.apache.wink.client.ClientResponse;
import org.eclipse.lyo.client.oslc.OSLCConstants;
import org.eclipse.lyo.client.oslc.OslcClient;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.resources.Waypoint;
//import scott.lyo.domain.warehouse.Waypoint;

/**
 * Created on 2017-07-10
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class OslcClientHelper {

    private final static Logger log = LoggerFactory.getLogger(OslcClientHelper.class);

    private final OslcClient client = new OslcClient();
//    private final String base;

//    public OslcClientHelper(final String base) {
////        this.base = base;
//    }

    public <R extends AbstractResource> R fetchResource(URI uri, Class<R> clazz)
            throws OslcClientException {
        try {
            final String url = String.valueOf(uri);
            log.debug("Fetching an OSLC resource from the URI '{}'", url);
            final ClientResponse response = client.getResource(url,
                    OSLCConstants.CT_RDF);
            return response.getEntity(clazz);
        } catch (URISyntaxException | IOException | OAuthException e) {
            throw new OslcClientException("Failed to fetch Waypoint OSLC resource", e);
        }
    }

//    private URI constructWaypointURI(final String basePath, final String serviceProviderId,
//            final String waypointId) {
//        Map<String, Object> pathParameters = new HashMap<String, Object>();
//        pathParameters.put("serviceProviderId", serviceProviderId);
//        pathParameters.put("waypointId", waypointId);
//        String instanceURI = "serviceProviders/{serviceProviderId}/resources/waypoints" +
//                "/{waypointId}";
//
//        final UriBuilder builder = UriBuilder.fromUri(basePath);
//        return builder.path(instanceURI).buildFromMap(pathParameters);
//    }
}