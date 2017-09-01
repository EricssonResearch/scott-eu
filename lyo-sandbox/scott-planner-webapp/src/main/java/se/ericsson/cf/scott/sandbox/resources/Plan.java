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
 *     Russell Boykin       - initial API and implementation
 *     Alberto Giammaria    - initial API and implementation
 *     Chris Peters         - initial API and implementation
 *     Gianluca Bernardini  - initial API and implementation
 *       Sam Padgett          - initial API and implementation
 *     Michael Fiedler      - adapted for OSLC4J
 *     Jad El-khoury        - initial implementation of code generator (422448)
 *     Matthieu Helleboid   - Support for multiple Service Providers.
 *     Anass Radouani       - Support for multiple Service Providers.
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/

package se.ericsson.cf.scott.sandbox.resources;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.text.SimpleDateFormat;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.Iterator;
import javax.servlet.http.HttpServletRequest;
import java.io.UnsupportedEncodingException;
import java.net.URLEncoder;
import javax.ws.rs.core.UriBuilder;

import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.annotation.OslcAllowedValue;
import org.eclipse.lyo.oslc4j.core.annotation.OslcDescription;
import org.eclipse.lyo.oslc4j.core.annotation.OslcMemberProperty;
import org.eclipse.lyo.oslc4j.core.annotation.OslcName;
import org.eclipse.lyo.oslc4j.core.annotation.OslcNamespace;
import org.eclipse.lyo.oslc4j.core.annotation.OslcOccurs;
import org.eclipse.lyo.oslc4j.core.annotation.OslcPropertyDefinition;
import org.eclipse.lyo.oslc4j.core.annotation.OslcRange;
import org.eclipse.lyo.oslc4j.core.annotation.OslcReadOnly;
import org.eclipse.lyo.oslc4j.core.annotation.OslcRepresentation;
import org.eclipse.lyo.oslc4j.core.annotation.OslcResourceShape;
import org.eclipse.lyo.oslc4j.core.annotation.OslcTitle;
import org.eclipse.lyo.oslc4j.core.annotation.OslcValueType;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.core.model.Occurs;
import org.eclipse.lyo.oslc4j.core.model.OslcConstants;
import org.eclipse.lyo.oslc4j.core.model.Representation;
import org.eclipse.lyo.oslc4j.core.model.ValueType;
import org.eclipse.lyo.oslc4j.core.model.ResourceShape;
import org.eclipse.lyo.oslc4j.core.model.ResourceShapeFactory;

import se.ericsson.cf.scott.sandbox.PlannerReasonerConstants;
import se.ericsson.cf.scott.sandbox.resources.Mission;
import se.ericsson.cf.scott.sandbox.resources.Action;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PlannerReasonerConstants.PLANNING_NAMSPACE)
@OslcName(PlannerReasonerConstants.PLAN)
@OslcResourceShape(title = "Plan Resource Shape", describes = PlannerReasonerConstants.TYPE_PLAN)
public class Plan
    extends AbstractResource
    implements IPlan
{
    // Start of user code attributeAnnotation:forMission
    // End of user code
    private Link forMission = new Link();
    // Start of user code attributeAnnotation:relatesToAction
    // End of user code
    private Action relatesToAction = new Action();
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public Plan()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public Plan(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    public Plan(final String serviceProviderId, final String planId)
           throws URISyntaxException
    {
        this (constructURI(serviceProviderId, planId));
        // Start of user code constructor3
        // End of user code
    }
    
    public static URI constructURI(final String serviceProviderId, final String planId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("planId", planId);
        String instanceURI = "serviceProviders/{serviceProviderId}/plans/{planId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLink(final String serviceProviderId, final String planId , final String label)
    {
        return new Link(constructURI(serviceProviderId, planId), label);
    }
    
    public static Link constructLink(final String serviceProviderId, final String planId)
    {
        return new Link(constructURI(serviceProviderId, planId));
    }
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PlannerReasonerConstants.PATH_PLAN,
        Plan.class);
    }
    
    
    public String toString()
    {
        return toString(false);
    }
    
    public String toString(boolean asLocalResource)
    {
        String result = "";
        // Start of user code toString_init
        // End of user code
    
        if (asLocalResource) {
            result = result + "{a Local Plan Resource} - update Plan.toString() to present resource as desired.";
            // Start of user code toString_bodyForLocalResource
            // End of user code
        }
        else {
            result = getAbout().toString();
        }
    
        // Start of user code toString_finalize
        // End of user code
    
        return result;
    }
    
    public String toHtml()
    {
        return toHtml(false);
    }
    
    public String toHtml(boolean asLocalResource)
    {
        String result = "";
        // Start of user code toHtml_init
        // End of user code
    
        if (asLocalResource) {
            result = toString(true);
            // Start of user code toHtml_bodyForLocalResource
            // End of user code
        }
        else {
            result = "<a href=\"" + getAbout() + "\">" + toString() + "</a>";
        }
    
        // Start of user code toHtml_finalize
        // End of user code
    
        return result;
    }
    
    
    // Start of user code getterAnnotation:forMission
    // End of user code
    @OslcName("forMission")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "forMission")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_MISSION})
    @OslcReadOnly(false)
    public Link getForMission()
    {
        // Start of user code getterInit:forMission
        // End of user code
        return forMission;
    }
    
    // Start of user code getterAnnotation:relatesToAction
    // End of user code
    @OslcName("relatesToAction")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "relatesToAction")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.LocalResource)
    @OslcRange({PlannerReasonerConstants.TYPE_ACTION})
    @OslcReadOnly(false)
    public Action getRelatesToAction()
    {
        // Start of user code getterInit:relatesToAction
        // End of user code
        return relatesToAction;
    }
    
    
    // Start of user code setterAnnotation:forMission
    // End of user code
    public void setForMission(final Link forMission )
    {
        // Start of user code setterInit:forMission
        // End of user code
        this.forMission = forMission;
    
        // Start of user code setterFinalize:forMission
        // End of user code
    }
    
    // Start of user code setterAnnotation:relatesToAction
    // End of user code
    public void setRelatesToAction(final Action relatesToAction )
    {
        // Start of user code setterInit:relatesToAction
        // End of user code
        this.relatesToAction = relatesToAction;
    
        // Start of user code setterFinalize:relatesToAction
        // End of user code
    }
    
    
    static public String forMissionToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:forMissionToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"forMission\">forMission: </LABEL>";
    
        // Start of user code "Mid:forMissionToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:forMissionToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String relatesToActionToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:relatesToActionToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"relatesToAction\">relatesToAction: </LABEL>";
    
        // Start of user code "Mid:relatesToActionToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:relatesToActionToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String forMissionToHtml()
    {
        String s = "";
    
        // Start of user code forMissiontoHtml_mid
        // End of user code
    
        try {
            if ((forMission == null) || (forMission.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new Mission (forMission.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code forMissiontoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String relatesToActionToHtml()
    {
        String s = "";
    
        // Start of user code relatesToActiontoHtml_mid
        // End of user code
    
        try {
            if (relatesToAction == null) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + relatesToAction.toHtml(true);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code relatesToActiontoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
