// Start of user code Copyright
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
// End of user code

package se.ericsson.cf.scott.sandbox.twin.services;

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

import se.ericsson.cf.scott.sandbox.twin.RobotTwinManager;
import se.ericsson.cf.scott.sandbox.twin.RobotTwinConstants;
import eu.scott.warehouse.domains.blocksworld.BworldDomainConstants;
import eu.scott.warehouse.domains.pddl.PddlDomainConstants;
import se.ericsson.cf.scott.sandbox.twin.servlet.ServiceProviderCatalogSingleton;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.PlanExecutionResult;
import eu.scott.warehouse.domains.pddl.Step;

// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code
@OslcService(BworldDomainConstants.BLOCKSWORLD_DOMAIN_DOMAIN)
@Path("serviceProviders/{serviceProviderId}/planExecutionResults")
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
    @Path("{planExecutionResultId}")
    @Produces({OslcMediaType.APPLICATION_RDF_XML, OslcMediaType.APPLICATION_XML, OslcMediaType.APPLICATION_JSON, OslcMediaType.TEXT_TURTLE})
    public PlanExecutionResult getPlanExecutionResult(
                @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planExecutionResultId") final String planExecutionResultId
        ) throws IOException, ServletException, URISyntaxException
    {
        // Start of user code getResource_init
        // End of user code

        final PlanExecutionResult aPlanExecutionResult = RobotTwinManager.getPlanExecutionResult(httpServletRequest, serviceProviderId, planExecutionResultId);

        if (aPlanExecutionResult != null) {
            // Start of user code getPlanExecutionResult
            // End of user code
            httpServletResponse.addHeader(RobotTwinConstants.HDR_OSLC_VERSION, RobotTwinConstants.OSLC_VERSION_V2);
            return aPlanExecutionResult;
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("{planExecutionResultId}")
    @Produces({ MediaType.TEXT_HTML })
    public Response getPlanExecutionResultAsHtml(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planExecutionResultId") final String planExecutionResultId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getPlanExecutionResultAsHtml_init
        // End of user code

        final PlanExecutionResult aPlanExecutionResult = RobotTwinManager.getPlanExecutionResult(httpServletRequest, serviceProviderId, planExecutionResultId);

        if (aPlanExecutionResult != null) {
            httpServletRequest.setAttribute("aPlanExecutionResult", aPlanExecutionResult);
            // Start of user code getPlanExecutionResultAsHtml_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twin/planexecutionresult.jsp");
            rd.forward(httpServletRequest,httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("{planExecutionResultId}")
    @Produces({OslcMediaType.APPLICATION_X_OSLC_COMPACT_XML})
    public Compact getPlanExecutionResultCompact(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planExecutionResultId") final String planExecutionResultId
        ) throws ServletException, IOException, URISyntaxException
    {
        String iconUri = OSLC4JUtils.getPublicURI() + "/images/ui_preview_icon.gif";
        String smallPreviewHintHeight = "10em";
        String smallPreviewHintWidth = "45em";
        String largePreviewHintHeight = "20em";
        String largePreviewHintWidth = "45em";

        // Start of user code getPlanExecutionResultCompact_init
        //TODO: adjust the preview height & width values from the default values provided above.
        // End of user code

        final PlanExecutionResult aPlanExecutionResult = RobotTwinManager.getPlanExecutionResult(httpServletRequest, serviceProviderId, planExecutionResultId);

        if (aPlanExecutionResult != null) {
            final Compact compact = new Compact();

            compact.setAbout(aPlanExecutionResult.getAbout());
            compact.setTitle(aPlanExecutionResult.toString());

            compact.setIcon(new URI(iconUri));

            //Create and set attributes for OSLC preview resource
            final Preview smallPreview = new Preview();
            smallPreview.setHintHeight(smallPreviewHintHeight);
            smallPreview.setHintWidth(smallPreviewHintWidth);
            smallPreview.setDocument(UriBuilder.fromUri(aPlanExecutionResult.getAbout()).path("smallPreview").build());
            compact.setSmallPreview(smallPreview);

            final Preview largePreview = new Preview();
            largePreview.setHintHeight(largePreviewHintHeight);
            largePreview.setHintWidth(largePreviewHintWidth);
            largePreview.setDocument(UriBuilder.fromUri(aPlanExecutionResult.getAbout()).path("largePreview").build());
            compact.setLargePreview(largePreview);

            httpServletResponse.addHeader(RobotTwinConstants.HDR_OSLC_VERSION, RobotTwinConstants.OSLC_VERSION_V2);
            return compact;
        }
        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("{planExecutionResultId}/smallPreview")
    @Produces({ MediaType.TEXT_HTML })
    public void getPlanExecutionResultAsHtmlSmallPreview(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planExecutionResultId") final String planExecutionResultId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getPlanExecutionResultAsHtmlSmallPreview_init
        // End of user code

        final PlanExecutionResult aPlanExecutionResult = RobotTwinManager.getPlanExecutionResult(httpServletRequest, serviceProviderId, planExecutionResultId);

        if (aPlanExecutionResult != null) {
            httpServletRequest.setAttribute("aPlanExecutionResult", aPlanExecutionResult);
            // Start of user code getPlanExecutionResultAsHtmlSmallPreview_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twin/planexecutionresultsmallpreview.jsp");
            httpServletResponse.addHeader(RobotTwinConstants.HDR_OSLC_VERSION, RobotTwinConstants.OSLC_VERSION_V2);
            rd.forward(httpServletRequest, httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }

    @GET
    @Path("{planExecutionResultId}/largePreview")
    @Produces({ MediaType.TEXT_HTML })
    public void getPlanExecutionResultAsHtmlLargePreview(
        @PathParam("serviceProviderId") final String serviceProviderId, @PathParam("planExecutionResultId") final String planExecutionResultId
        ) throws ServletException, IOException, URISyntaxException
    {
        // Start of user code getPlanExecutionResultAsHtmlLargePreview_init
        // End of user code

        final PlanExecutionResult aPlanExecutionResult = RobotTwinManager.getPlanExecutionResult(httpServletRequest, serviceProviderId, planExecutionResultId);

        if (aPlanExecutionResult != null) {
            httpServletRequest.setAttribute("aPlanExecutionResult", aPlanExecutionResult);
            // Start of user code getPlanExecutionResultAsHtmlLargePreview_setAttributes
            // End of user code

            RequestDispatcher rd = httpServletRequest.getRequestDispatcher("/se/ericsson/cf/scott/sandbox/twin/planexecutionresultlargepreview.jsp");
            httpServletResponse.addHeader(RobotTwinConstants.HDR_OSLC_VERSION, RobotTwinConstants.OSLC_VERSION_V2);
            rd.forward(httpServletRequest, httpServletResponse);
        }

        throw new WebApplicationException(Status.NOT_FOUND);
    }
}
