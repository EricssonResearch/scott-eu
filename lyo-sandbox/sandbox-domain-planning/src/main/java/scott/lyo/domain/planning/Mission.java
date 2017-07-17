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
 *	   Sam Padgett          - initial API and implementation
 *     Michael Fiedler      - adapted for OSLC4J
 *     Jad El-khoury        - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 *     Matthieu Helleboid   - Support for multiple Service Providers.
 *     Anass Radouani       - Support for multiple Service Providers.
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/

package scott.lyo.domain.planning;

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

import scott.lyo.domain.planning.PpConstants;
import scott.lyo.domain.planning.PpConstants;
import se.ericsson.cf.scott.sandbox.resources.RdfsConstants;
import scott.lyo.domain.planning.Predicate;
import scott.lyo.domain.planning.ProblemState;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PpConstants.PLANNING_NAMSPACE)
@OslcName(PpConstants.MISSION)
@OslcResourceShape(title = "Mission Resource Shape", describes = PpConstants.TYPE_MISSION)
public class Mission
    extends AbstractResource
    implements IMission
{
    // Start of user code attributeAnnotation:planningDeadline
    // End of user code
    private Date planningDeadline;
    // Start of user code attributeAnnotation:label
    // End of user code
    private String label;
    // Start of user code attributeAnnotation:missionDeadline
    // End of user code
    private Date missionDeadline;
    // Start of user code attributeAnnotation:goalPredicate
    // End of user code
    private HashSet<Link> goalPredicate = new HashSet<Link>();
    // Start of user code attributeAnnotation:initialState
    // End of user code
    private ProblemState initialState = new ProblemState();
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public Mission()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public Mission(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PpConstants.PATH_MISSION,  
        Mission.class);
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
            result = result + "{a Local Mission Resource} - update Mission.toString() to present resource as desired.";
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
    
    public void addGoalPredicate(final Link goalPredicate)
    {
        this.goalPredicate.add(goalPredicate);
    }
    
    
    // Start of user code getterAnnotation:planningDeadline
    // End of user code
    @OslcName("planningDeadline")
    @OslcPropertyDefinition(PpConstants.PLANNING_NAMSPACE + "planningDeadline")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.DateTime)
    @OslcReadOnly(false)
    public Date getPlanningDeadline()
    {
        // Start of user code getterInit:planningDeadline
        // End of user code
        return planningDeadline;
    }
    
    // Start of user code getterAnnotation:label
    // End of user code
    @OslcName("label")
    @OslcPropertyDefinition(RdfsConstants.RDFS_NAMSPACE + "label")
    @OslcDescription("May be used to provide a human-readable version of a resource's name.")
    @OslcOccurs(Occurs.ZeroOrOne)
    @OslcValueType(ValueType.XMLLiteral)
    @OslcReadOnly(false)
    public String getLabel()
    {
        // Start of user code getterInit:label
        // End of user code
        return label;
    }
    
    // Start of user code getterAnnotation:missionDeadline
    // End of user code
    @OslcName("missionDeadline")
    @OslcPropertyDefinition(PpConstants.PLANNING_NAMSPACE + "missionDeadline")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.DateTime)
    @OslcReadOnly(false)
    public Date getMissionDeadline()
    {
        // Start of user code getterInit:missionDeadline
        // End of user code
        return missionDeadline;
    }
    
    // Start of user code getterAnnotation:goalPredicate
    // End of user code
    @OslcName("goalPredicate")
    @OslcPropertyDefinition(PpConstants.PLANNING_NAMSPACE + "goalPredicate")
    @OslcOccurs(Occurs.OneOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PpConstants.TYPE_PREDICATE})
    @OslcReadOnly(false)
    public HashSet<Link> getGoalPredicate()
    {
        // Start of user code getterInit:goalPredicate
        // End of user code
        return goalPredicate;
    }
    
    // Start of user code getterAnnotation:initialState
    // End of user code
    @OslcName("initialState")
    @OslcPropertyDefinition(PpConstants.PLANNING_NAMSPACE + "initialState")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.LocalResource)
    @OslcRange({PpConstants.TYPE_PROBLEMSTATE})
    @OslcReadOnly(false)
    public ProblemState getInitialState()
    {
        // Start of user code getterInit:initialState
        // End of user code
        return initialState;
    }
    
    
    // Start of user code setterAnnotation:planningDeadline
    // End of user code
    public void setPlanningDeadline(final Date planningDeadline )
    {
        // Start of user code setterInit:planningDeadline
        // End of user code
        this.planningDeadline = planningDeadline;
    
        // Start of user code setterFinalize:planningDeadline
        // End of user code
    }
    
    // Start of user code setterAnnotation:label
    // End of user code
    public void setLabel(final String label )
    {
        // Start of user code setterInit:label
        // End of user code
        this.label = label;
    
        // Start of user code setterFinalize:label
        // End of user code
    }
    
    // Start of user code setterAnnotation:missionDeadline
    // End of user code
    public void setMissionDeadline(final Date missionDeadline )
    {
        // Start of user code setterInit:missionDeadline
        // End of user code
        this.missionDeadline = missionDeadline;
    
        // Start of user code setterFinalize:missionDeadline
        // End of user code
    }
    
    // Start of user code setterAnnotation:goalPredicate
    // End of user code
    public void setGoalPredicate(final HashSet<Link> goalPredicate )
    {
        // Start of user code setterInit:goalPredicate
        // End of user code
        this.goalPredicate.clear();
        if (goalPredicate != null)
        {
            this.goalPredicate.addAll(goalPredicate);
        }
    
        // Start of user code setterFinalize:goalPredicate
        // End of user code
    }
    
    // Start of user code setterAnnotation:initialState
    // End of user code
    public void setInitialState(final ProblemState initialState )
    {
        // Start of user code setterInit:initialState
        // End of user code
        this.initialState = initialState;
    
        // Start of user code setterFinalize:initialState
        // End of user code
    }
    
    
    static public String planningDeadlineToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:planningDeadlineToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"planningDeadline\">planningDeadline: </LABEL>";
    
        // Start of user code "Mid:planningDeadlineToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"planningDeadline\" type=\"text\" style=\"width: 400px\" id=\"planningDeadline\" >";
        // Start of user code "Finalize:planningDeadlineToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String labelToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:labelToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"label\">label: </LABEL>";
    
        // Start of user code "Mid:labelToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"label\" type=\"text\" style=\"width: 400px\" id=\"label\" >";
        // Start of user code "Finalize:labelToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String missionDeadlineToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:missionDeadlineToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"missionDeadline\">missionDeadline: </LABEL>";
    
        // Start of user code "Mid:missionDeadlineToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"missionDeadline\" type=\"text\" style=\"width: 400px\" id=\"missionDeadline\" >";
        // Start of user code "Finalize:missionDeadlineToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String goalPredicateToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:goalPredicateToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"goalPredicate\">goalPredicate: </LABEL>";
    
        // Start of user code "Mid:goalPredicateToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:goalPredicateToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String initialStateToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:initialStateToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"initialState\">initialState: </LABEL>";
    
        // Start of user code "Mid:initialStateToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:initialStateToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String planningDeadlineToHtml()
    {
        String s = "";
    
        // Start of user code planningDeadlinetoHtml_init
        // End of user code
    
        s = s + "<label for=\"planningDeadline\"><strong>planningDeadline</strong>: </LABEL>";
    
        // Start of user code planningDeadlinetoHtml_mid
        // End of user code
    
        try {
            if (planningDeadline == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + planningDeadline.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code planningDeadlinetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String labelToHtml()
    {
        String s = "";
    
        // Start of user code labeltoHtml_init
        // End of user code
    
        s = s + "<label for=\"label\"><strong>label</strong>: </LABEL>";
    
        // Start of user code labeltoHtml_mid
        // End of user code
    
        try {
            if (label == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + label.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code labeltoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String missionDeadlineToHtml()
    {
        String s = "";
    
        // Start of user code missionDeadlinetoHtml_init
        // End of user code
    
        s = s + "<label for=\"missionDeadline\"><strong>missionDeadline</strong>: </LABEL>";
    
        // Start of user code missionDeadlinetoHtml_mid
        // End of user code
    
        try {
            if (missionDeadline == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + missionDeadline.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code missionDeadlinetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String goalPredicateToHtml()
    {
        String s = "";
    
        // Start of user code goalPredicatetoHtml_init
        // End of user code
    
        s = s + "<label for=\"goalPredicate\"><strong>goalPredicate</strong>: </LABEL>";
    
        // Start of user code goalPredicatetoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : goalPredicate) {
                s = s + "<li>";
                s = s + (new Predicate (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code goalPredicatetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String initialStateToHtml()
    {
        String s = "";
    
        // Start of user code initialStatetoHtml_init
        // End of user code
    
        s = s + "<label for=\"initialState\"><strong>initialState</strong>: </LABEL>";
    
        // Start of user code initialStatetoHtml_mid
        // End of user code
    
        try {
            if (initialState == null) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + initialState.toHtml(true);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code initialStatetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
