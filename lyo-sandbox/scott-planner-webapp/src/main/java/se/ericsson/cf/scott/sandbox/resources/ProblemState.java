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
import se.ericsson.cf.scott.sandbox.resources.WhObject;
import se.ericsson.cf.scott.sandbox.resources.Place;
import se.ericsson.cf.scott.sandbox.resources.Robot;
import se.ericsson.cf.scott.sandbox.resources.Waypoint;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PlannerReasonerConstants.PLANNING_NAMSPACE)
@OslcName(PlannerReasonerConstants.PROBLEMSTATE)
@OslcResourceShape(title = "ProblemState Resource Shape", describes = PlannerReasonerConstants.TYPE_PROBLEMSTATE)
public class ProblemState
    extends AbstractResource
    implements IProblemState
{
    // Start of user code attributeAnnotation:robots
    // End of user code
    private HashSet<Link> robots = new HashSet<Link>();
    // Start of user code attributeAnnotation:places
    // End of user code
    private HashSet<Link> places = new HashSet<Link>();
    // Start of user code attributeAnnotation:objects
    // End of user code
    private HashSet<Link> objects = new HashSet<Link>();
    // Start of user code attributeAnnotation:waypoints
    // End of user code
    private HashSet<Link> waypoints = new HashSet<Link>();
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public ProblemState()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public ProblemState(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PlannerReasonerConstants.PATH_PROBLEMSTATE,
        ProblemState.class);
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
            result = result + "{a Local ProblemState Resource} - update ProblemState.toString() to present resource as desired.";
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
    
    public void addRobots(final Link robots)
    {
        this.robots.add(robots);
    }
    
    public void addPlaces(final Link places)
    {
        this.places.add(places);
    }
    
    public void addObjects(final Link objects)
    {
        this.objects.add(objects);
    }
    
    public void addWaypoints(final Link waypoints)
    {
        this.waypoints.add(waypoints);
    }
    
    
    // Start of user code getterAnnotation:robots
    // End of user code
    @OslcName("robots")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "robots")
    @OslcDescription("")
    @OslcOccurs(Occurs.OneOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_ROBOT})
    @OslcReadOnly(false)
    public HashSet<Link> getRobots()
    {
        // Start of user code getterInit:robots
        // End of user code
        return robots;
    }
    
    // Start of user code getterAnnotation:places
    // End of user code
    @OslcName("places")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "places")
    @OslcOccurs(Occurs.OneOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_PLACE})
    @OslcReadOnly(false)
    public HashSet<Link> getPlaces()
    {
        // Start of user code getterInit:places
        // End of user code
        return places;
    }
    
    // Start of user code getterAnnotation:objects
    // End of user code
    @OslcName("objects")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "objects")
    @OslcOccurs(Occurs.ZeroOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_WHOBJECT})
    @OslcReadOnly(false)
    public HashSet<Link> getObjects()
    {
        // Start of user code getterInit:objects
        // End of user code
        return objects;
    }
    
    // Start of user code getterAnnotation:waypoints
    // End of user code
    @OslcName("waypoints")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "waypoints")
    @OslcOccurs(Occurs.OneOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_WAYPOINT})
    @OslcReadOnly(false)
    public HashSet<Link> getWaypoints()
    {
        // Start of user code getterInit:waypoints
        // End of user code
        return waypoints;
    }
    
    
    // Start of user code setterAnnotation:robots
    // End of user code
    public void setRobots(final HashSet<Link> robots )
    {
        // Start of user code setterInit:robots
        // End of user code
        this.robots.clear();
        if (robots != null)
        {
            this.robots.addAll(robots);
        }
    
        // Start of user code setterFinalize:robots
        // End of user code
    }
    
    // Start of user code setterAnnotation:places
    // End of user code
    public void setPlaces(final HashSet<Link> places )
    {
        // Start of user code setterInit:places
        // End of user code
        this.places.clear();
        if (places != null)
        {
            this.places.addAll(places);
        }
    
        // Start of user code setterFinalize:places
        // End of user code
    }
    
    // Start of user code setterAnnotation:objects
    // End of user code
    public void setObjects(final HashSet<Link> objects )
    {
        // Start of user code setterInit:objects
        // End of user code
        this.objects.clear();
        if (objects != null)
        {
            this.objects.addAll(objects);
        }
    
        // Start of user code setterFinalize:objects
        // End of user code
    }
    
    // Start of user code setterAnnotation:waypoints
    // End of user code
    public void setWaypoints(final HashSet<Link> waypoints )
    {
        // Start of user code setterInit:waypoints
        // End of user code
        this.waypoints.clear();
        if (waypoints != null)
        {
            this.waypoints.addAll(waypoints);
        }
    
        // Start of user code setterFinalize:waypoints
        // End of user code
    }
    
    
    static public String robotsToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:robotsToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"robots\">robots: </LABEL>";
    
        // Start of user code "Mid:robotsToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:robotsToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String placesToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:placesToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"places\">places: </LABEL>";
    
        // Start of user code "Mid:placesToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:placesToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String objectsToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:objectsToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"objects\">objects: </LABEL>";
    
        // Start of user code "Mid:objectsToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:objectsToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String waypointsToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:waypointsToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"waypoints\">waypoints: </LABEL>";
    
        // Start of user code "Mid:waypointsToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:waypointsToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String robotsToHtml()
    {
        String s = "";
    
        // Start of user code robotstoHtml_init
        // End of user code
    
        s = s + "<label for=\"robots\"><strong>robots</strong>: &nbsp;</LABEL>";
    
        // Start of user code robotstoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : robots) {
                s = s + "<li>";
                s = s + (new Robot (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code robotstoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String placesToHtml()
    {
        String s = "";
    
        // Start of user code placestoHtml_init
        // End of user code
    
        s = s + "<label for=\"places\"><strong>places</strong>: &nbsp;</LABEL>";
    
        // Start of user code placestoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : places) {
                s = s + "<li>";
                s = s + (new Place (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code placestoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String objectsToHtml()
    {
        String s = "";
    
        // Start of user code objectstoHtml_init
        // End of user code
    
        s = s + "<label for=\"objects\"><strong>objects</strong>: &nbsp;</LABEL>";
    
        // Start of user code objectstoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : objects) {
                s = s + "<li>";
                s = s + (new WhObject (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code objectstoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String waypointsToHtml()
    {
        String s = "";
    
        // Start of user code waypointstoHtml_init
        // End of user code
    
        s = s + "<label for=\"waypoints\"><strong>waypoints</strong>: &nbsp;</LABEL>";
    
        // Start of user code waypointstoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : waypoints) {
                s = s + "<li>";
                s = s + (new Waypoint (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code waypointstoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
