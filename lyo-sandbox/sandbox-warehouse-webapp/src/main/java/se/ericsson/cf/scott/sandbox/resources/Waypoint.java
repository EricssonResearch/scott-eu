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

import se.ericsson.cf.scott.sandbox.WarehouseControllerConstants;
import se.ericsson.cf.scott.sandbox.resources.Waypoint;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(WarehouseControllerConstants.WAREHOUSE_NAMSPACE)
@OslcName(WarehouseControllerConstants.WAYPOINT)
@OslcResourceShape(title = "Waypoint Resource Shape", describes = WarehouseControllerConstants.TYPE_WAYPOINT)
public class Waypoint
    extends AbstractResource
    implements IWaypoint
{
    // Start of user code attributeAnnotation:canMove
    // End of user code
    private HashSet<Link> canMove = new HashSet<Link>();
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public Waypoint()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public Waypoint(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    public Waypoint(final String serviceProviderId, final String waypointId)
           throws URISyntaxException
    {
        this (constructURI(serviceProviderId, waypointId));
        // Start of user code constructor3
        // End of user code
    }
    
    public static URI constructURI(final String serviceProviderId, final String waypointId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("waypointId", waypointId);
        String instanceURI = "serviceProviders/{serviceProviderId}/resources/waypoints/{waypointId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLink(final String serviceProviderId, final String waypointId , final String label)
    {
        return new Link(constructURI(serviceProviderId, waypointId), label);
    }
    
    public static Link constructLink(final String serviceProviderId, final String waypointId)
    {
        return new Link(constructURI(serviceProviderId, waypointId));
    }
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        WarehouseControllerConstants.PATH_WAYPOINT,
        Waypoint.class);
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
            result = result + "{a Local Waypoint Resource} - update Waypoint.toString() to present resource as desired.";
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
    
    public void addCanMove(final Link canMove)
    {
        this.canMove.add(canMove);
    }
    
    
    // Start of user code getterAnnotation:canMove
    // End of user code
    @OslcName("canMove")
    @OslcPropertyDefinition(WarehouseControllerConstants.WAREHOUSE_NAMSPACE + "canMove")
    @OslcOccurs(Occurs.ZeroOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({WarehouseControllerConstants.TYPE_WAYPOINT})
    @OslcReadOnly(false)
    public HashSet<Link> getCanMove()
    {
        // Start of user code getterInit:canMove
        // End of user code
        return canMove;
    }
    
    
    // Start of user code setterAnnotation:canMove
    // End of user code
    public void setCanMove(final HashSet<Link> canMove )
    {
        // Start of user code setterInit:canMove
        // End of user code
        this.canMove.clear();
        if (canMove != null)
        {
            this.canMove.addAll(canMove);
        }
    
        // Start of user code setterFinalize:canMove
        // End of user code
    }
    
    
    static public String canMoveToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:canMoveToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"canMove\">canMove: </LABEL>";
    
        // Start of user code "Mid:canMoveToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:canMoveToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String canMoveToHtml()
    {
        String s = "";
    
        // Start of user code canMovetoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : canMove) {
                s = s + "<li>";
                s = s + (new Waypoint (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code canMovetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
