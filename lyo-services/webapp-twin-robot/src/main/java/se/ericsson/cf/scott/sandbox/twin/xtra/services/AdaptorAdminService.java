package se.ericsson.cf.scott.sandbox.twin.xtra.services;

import java.net.URISyntaxException;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.Consumes;
import javax.ws.rs.POST;
import javax.ws.rs.Path;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.Response;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twin.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.twin.xtra.TwinAdaptorHelper;
import se.ericsson.cf.scott.sandbox.twin.TwinsServiceProviderInfo;
import se.ericsson.cf.scott.sandbox.twin.xtra.TwinRegistrationClient;
import se.ericsson.cf.scott.sandbox.twin.servlet.TwinsServiceProvidersFactory;

@Path("admin")
public class AdaptorAdminService {
    private final static Logger log = LoggerFactory.getLogger(AdaptorAdminService.class);

    @Context private HttpServletRequest httpServletRequest;
    @Context private HttpServletResponse httpServletResponse;

    final TwinRegistrationClient registrationClient = TwinAdaptorHelper.createTwinRegistrationClient();

    // TODO Andrew@2019-04-23: add a Store wipe endpoint

    @POST
    @Path("initRDF")
    @Consumes({OslcMediaType.APPLICATION_RDF_XML, OslcMediaType.APPLICATION_XML, OslcMediaType.APPLICATION_JSON, OslcMediaType.TEXT_TURTLE})
    public Response getServiceProvider(ServiceProvider sp)
    {
        log.info("Processing the init call (RDF input)");

        final TwinsServiceProviderInfo twinInfo = new TwinsServiceProviderInfo(sp.getTitle(), "robot", sp.getIdentifier());
        try {
            final ServiceProvider serviceProvider = TwinsServiceProvidersFactory.createServiceProvider(
                twinInfo);
            TwinAdaptorHelper.getServiceProviderRepository().addServiceProvider(serviceProvider);

            registrationClient.registerTwin(serviceProvider);
        } catch (OslcCoreApplicationException | URISyntaxException e) {
            log.error("Can't init a dummy ServiceProvider", e);
            return Response.status(Response.Status.BAD_REQUEST).build();
        }

        httpServletResponse.addHeader("Oslc-Core-Version","2.0");
        return Response.noContent().build();
    }
}
