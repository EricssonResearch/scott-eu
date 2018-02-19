/*******************************************************************************
 * Copyright (c) 2012 IBM Corporation and others.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v. 1.0 which accompanies this distribution.
 *
 * The Eclipse Public License is available at http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *
 *     Michael Fiedler     - initial API and implementation for Bugzilla adapter
 *     Jad El-khoury       - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 *     Jim Amsden          - Support for UI Preview (494303)
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/
package se.ericsson.cf.scott.sandbox.twins.shelf.services;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.text.SimpleDateFormat;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import javax.servlet.RequestDispatcher;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.Consumes;
import javax.ws.rs.DELETE;
import javax.ws.rs.FormParam;
import javax.ws.rs.GET;
import javax.ws.rs.HeaderParam;
import javax.ws.rs.POST;
import javax.ws.rs.PUT;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.QueryParam;
import javax.ws.rs.WebApplicationException;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.Response.Status;
import javax.ws.rs.core.UriInfo;
import javax.ws.rs.core.UriBuilder;

import org.apache.wink.json4j.JSONObject;
import org.eclipse.lyo.oslc4j.provider.json4j.JsonHelper;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.core.annotation.OslcCreationFactory;
import org.eclipse.lyo.oslc4j.core.annotation.OslcDialog;
import org.eclipse.lyo.oslc4j.core.annotation.OslcDialogs;
import org.eclipse.lyo.oslc4j.core.annotation.OslcQueryCapability;
import org.eclipse.lyo.oslc4j.core.annotation.OslcService;
import org.eclipse.lyo.oslc4j.core.model.Compact;
import org.eclipse.lyo.oslc4j.core.model.OslcConstants;
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType;
import org.eclipse.lyo.oslc4j.core.model.Preview;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;

import se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinManager;
import se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinConstants;
import eu.scott.warehouse.domains.blocksworld.BworldDomainConstants;
import eu.scott.warehouse.domains.blocksworld.BworldDomainConstants;
import eu.scott.warehouse.domains.blocksworld.BworldDomainConstants;
import eu.scott.warehouse.domains.pddl.PddlDomainConstants;
import se.ericsson.cf.scott.sandbox.twins.shelf.servlet.ServiceProviderCatalogSingleton;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.PrimitiveType;
import eu.scott.warehouse.domains.pddl.Step;
import eu.scott.warehouse.domains.blocksworld.Block;
import eu.scott.warehouse.domains.blocksworld.Location;

// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code
@OslcService(BworldDomainConstants.BLOCKSWORLD_DOMAIN_DOMAIN)
@Path("serviceProviders/{serviceProviderId}/resources")
public class ServiceProviderService1
{
    @Context private HttpServletRequest httpServletRequest;
    @Context private HttpServletResponse httpServletResponse;
    @Context private UriInfo uriInfo;

    // Start of user code class_attributes
    // End of user code

    // Start of user code class_methods
    // End of user code

    public ServiceProviderService1()
    {
        super();
    }

    @GET
    @Path("plans/{planId}")
    @Produces({OslcMediaType.APPLICATION_RDF_XML, OslcMediaType.APPLICATION_XML, OslcMediaType.APPLICATION_JSON, OslcMediaType.TEXT_TURTLE})
    public Plan getPlan(
                @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planId") final String planId
        ) throws IOException, ServletException, URISyntaxException
    {
        // Start of user code getResource_init
        // End of user code

        final Plan aPlan = ShelfTwinManager.getPlan(httpServletRequest, serviceProviderId, planId);

        if (aPlan != null) {
            // Start of user code getPlan
            // End of user code
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            return aPlan;
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("plans/{planId}")
    @Produces({ MediaType.TEXT_HTML })
    public Response getPlanAsHtml(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planId") final String planId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getPlanAsHtml_init
        // End of user code

        final Plan aPlan = ShelfTwinManager.getPlan(httpServletRequest, serviceProviderId, planId);

        if (aPlan != null) {
            httpServletRequest.setAttribute("aPlan", aPlan);
            // Start of user code getPlanAsHtml_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twins/shelf/plan.jsp");
            rd.forward(httpServletRequest,httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("plans/{planId}")
    @Produces({OslcMediaType.APPLICATION_X_OSLC_COMPACT_XML})
    public Compact getPlanCompact(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planId") final String planId
        ) throws ServletException, IOException, URISyntaxException
    {
        String iconUri = OSLC4JUtils.getPublicURI() + "/images/ui_preview_icon.gif";
        String smallPreviewHintHeight = "10em";
        String smallPreviewHintWidth = "45em";
        String largePreviewHintHeight = "20em";
        String largePreviewHintWidth = "45em";

        // Start of user code getPlanCompact_init
        //TODO: adjust the preview height & width values from the default values provided above.
        // End of user code

        final Plan aPlan = ShelfTwinManager.getPlan(httpServletRequest, serviceProviderId, planId);

        if (aPlan != null) {
            final Compact compact = new Compact();

            compact.setAbout(aPlan.getAbout());
            compact.setTitle(aPlan.toString());

            compact.setIcon(new URI(iconUri));

            //Create and set attributes for OSLC preview resource
            final Preview smallPreview = new Preview();
            smallPreview.setHintHeight(smallPreviewHintHeight);
            smallPreview.setHintWidth(smallPreviewHintWidth);
            smallPreview.setDocument(UriBuilder.fromUri(aPlan.getAbout()).path("smallPreview").build());
            compact.setSmallPreview(smallPreview);

            //Use the HTML representation of a change request as the large preview as well
            final Preview largePreview = new Preview();
            largePreview.setHintHeight(largePreviewHintHeight);
            largePreview.setHintWidth(largePreviewHintWidth);
            largePreview.setDocument(aPlan.getAbout());
            compact.setLargePreview(largePreview);
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            return compact;
        }
        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("plans/{planId}/smallPreview")
    @Produces({ MediaType.TEXT_HTML })
    public void getPlanAsHtmlSmallPreview(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planId") final String planId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getPlanAsHtmlSmallPreview_init
        // End of user code

        final Plan aPlan = ShelfTwinManager.getPlan(httpServletRequest, serviceProviderId, planId);

        if (aPlan != null) {
            httpServletRequest.setAttribute("aPlan", aPlan);
            // Start of user code getPlanAsHtmlSmallPreview_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twins/shelf/plansmallpreview.jsp");
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            rd.forward(httpServletRequest, httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }
    @GET
    @Path("locations/{locationId}")
    @Produces({OslcMediaType.APPLICATION_RDF_XML, OslcMediaType.APPLICATION_XML, OslcMediaType.APPLICATION_JSON, OslcMediaType.TEXT_TURTLE})
    public Location getLocation(
                @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("locationId") final String locationId
        ) throws IOException, ServletException, URISyntaxException
    {
        // Start of user code getResource_init
        // End of user code

        final Location aLocation = ShelfTwinManager.getLocation(httpServletRequest, serviceProviderId, locationId);

        if (aLocation != null) {
            // Start of user code getLocation
            // End of user code
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            return aLocation;
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("locations/{locationId}")
    @Produces({ MediaType.TEXT_HTML })
    public Response getLocationAsHtml(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("locationId") final String locationId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getLocationAsHtml_init
        // End of user code

        final Location aLocation = ShelfTwinManager.getLocation(httpServletRequest, serviceProviderId, locationId);

        if (aLocation != null) {
            httpServletRequest.setAttribute("aLocation", aLocation);
            // Start of user code getLocationAsHtml_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twins/shelf/location.jsp");
            rd.forward(httpServletRequest,httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("locations/{locationId}")
    @Produces({OslcMediaType.APPLICATION_X_OSLC_COMPACT_XML})
    public Compact getLocationCompact(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("locationId") final String locationId
        ) throws ServletException, IOException, URISyntaxException
    {
        String iconUri = OSLC4JUtils.getPublicURI() + "/images/ui_preview_icon.gif";
        String smallPreviewHintHeight = "10em";
        String smallPreviewHintWidth = "45em";
        String largePreviewHintHeight = "20em";
        String largePreviewHintWidth = "45em";

        // Start of user code getLocationCompact_init
        //TODO: adjust the preview height & width values from the default values provided above.
        // End of user code

        final Location aLocation = ShelfTwinManager.getLocation(httpServletRequest, serviceProviderId, locationId);

        if (aLocation != null) {
            final Compact compact = new Compact();

            compact.setAbout(aLocation.getAbout());
            compact.setTitle(aLocation.toString());

            compact.setIcon(new URI(iconUri));

            //Create and set attributes for OSLC preview resource
            final Preview smallPreview = new Preview();
            smallPreview.setHintHeight(smallPreviewHintHeight);
            smallPreview.setHintWidth(smallPreviewHintWidth);
            smallPreview.setDocument(UriBuilder.fromUri(aLocation.getAbout()).path("smallPreview").build());
            compact.setSmallPreview(smallPreview);

            //Use the HTML representation of a change request as the large preview as well
            final Preview largePreview = new Preview();
            largePreview.setHintHeight(largePreviewHintHeight);
            largePreview.setHintWidth(largePreviewHintWidth);
            largePreview.setDocument(aLocation.getAbout());
            compact.setLargePreview(largePreview);
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            return compact;
        }
        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("locations/{locationId}/smallPreview")
    @Produces({ MediaType.TEXT_HTML })
    public void getLocationAsHtmlSmallPreview(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("locationId") final String locationId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getLocationAsHtmlSmallPreview_init
        // End of user code

        final Location aLocation = ShelfTwinManager.getLocation(httpServletRequest, serviceProviderId, locationId);

        if (aLocation != null) {
            httpServletRequest.setAttribute("aLocation", aLocation);
            // Start of user code getLocationAsHtmlSmallPreview_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twins/shelf/locationsmallpreview.jsp");
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            rd.forward(httpServletRequest, httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }
    @GET
    @Path("blocks/{blockId}")
    @Produces({OslcMediaType.APPLICATION_RDF_XML, OslcMediaType.APPLICATION_XML, OslcMediaType.APPLICATION_JSON, OslcMediaType.TEXT_TURTLE})
    public Block getBlock(
                @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("blockId") final String blockId
        ) throws IOException, ServletException, URISyntaxException
    {
        // Start of user code getResource_init
        // End of user code

        final Block aBlock = ShelfTwinManager.getBlock(httpServletRequest, serviceProviderId, blockId);

        if (aBlock != null) {
            // Start of user code getBlock
            // End of user code
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            return aBlock;
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("blocks/{blockId}")
    @Produces({ MediaType.TEXT_HTML })
    public Response getBlockAsHtml(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("blockId") final String blockId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getBlockAsHtml_init
        // End of user code

        final Block aBlock = ShelfTwinManager.getBlock(httpServletRequest, serviceProviderId, blockId);

        if (aBlock != null) {
            httpServletRequest.setAttribute("aBlock", aBlock);
            // Start of user code getBlockAsHtml_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twins/shelf/block.jsp");
            rd.forward(httpServletRequest,httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("blocks/{blockId}")
    @Produces({OslcMediaType.APPLICATION_X_OSLC_COMPACT_XML})
    public Compact getBlockCompact(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("blockId") final String blockId
        ) throws ServletException, IOException, URISyntaxException
    {
        String iconUri = OSLC4JUtils.getPublicURI() + "/images/ui_preview_icon.gif";
        String smallPreviewHintHeight = "10em";
        String smallPreviewHintWidth = "45em";
        String largePreviewHintHeight = "20em";
        String largePreviewHintWidth = "45em";

        // Start of user code getBlockCompact_init
        //TODO: adjust the preview height & width values from the default values provided above.
        // End of user code

        final Block aBlock = ShelfTwinManager.getBlock(httpServletRequest, serviceProviderId, blockId);

        if (aBlock != null) {
            final Compact compact = new Compact();

            compact.setAbout(aBlock.getAbout());
            compact.setTitle(aBlock.toString());

            compact.setIcon(new URI(iconUri));

            //Create and set attributes for OSLC preview resource
            final Preview smallPreview = new Preview();
            smallPreview.setHintHeight(smallPreviewHintHeight);
            smallPreview.setHintWidth(smallPreviewHintWidth);
            smallPreview.setDocument(UriBuilder.fromUri(aBlock.getAbout()).path("smallPreview").build());
            compact.setSmallPreview(smallPreview);

            //Use the HTML representation of a change request as the large preview as well
            final Preview largePreview = new Preview();
            largePreview.setHintHeight(largePreviewHintHeight);
            largePreview.setHintWidth(largePreviewHintWidth);
            largePreview.setDocument(aBlock.getAbout());
            compact.setLargePreview(largePreview);
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            return compact;
        }
        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("blocks/{blockId}/smallPreview")
    @Produces({ MediaType.TEXT_HTML })
    public void getBlockAsHtmlSmallPreview(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("blockId") final String blockId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getBlockAsHtmlSmallPreview_init
        // End of user code

        final Block aBlock = ShelfTwinManager.getBlock(httpServletRequest, serviceProviderId, blockId);

        if (aBlock != null) {
            httpServletRequest.setAttribute("aBlock", aBlock);
            // Start of user code getBlockAsHtmlSmallPreview_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twins/shelf/blocksmallpreview.jsp");
            httpServletResponse.addHeader(ShelfTwinConstants.HDR_OSLC_VERSION, ShelfTwinConstants.OSLC_VERSION_V2);
            rd.forward(httpServletRequest, httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }
}
