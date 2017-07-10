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
import se.ericsson.cf.scott.sandbox.resources.ActionType;
import se.ericsson.cf.scott.sandbox.resources.VariableInstance;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PlannerReasonerConstants.PLANNING_NAMSPACE)
@OslcName(PlannerReasonerConstants.ACTION)
@OslcResourceShape(title = "Action Resource Shape", describes = PlannerReasonerConstants.TYPE_ACTION)
public class Action
    extends AbstractResource
    implements IAction
{
    // Start of user code attributeAnnotation:ofActionType
    // End of user code
    private Link ofActionType = new Link();
    // Start of user code attributeAnnotation:withVariable
    // End of user code
    private HashSet<VariableInstance> withVariable = new HashSet<VariableInstance>();
    // Start of user code attributeAnnotation:order
    // End of user code
    private Integer order;
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public Action()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public Action(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PlannerReasonerConstants.PATH_ACTION,  
        Action.class);
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
            result = result + "{a Local Action Resource} - update Action.toString() to present resource as desired.";
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
    
    public void addWithVariable(final VariableInstance withVariable)
    {
        this.withVariable.add(withVariable);
    }
    
    
    // Start of user code getterAnnotation:ofActionType
    // End of user code
    @OslcName("ofActionType")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "ofActionType")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_ACTIONTYPE})
    @OslcReadOnly(false)
    public Link getOfActionType()
    {
        // Start of user code getterInit:ofActionType
        // End of user code
        return ofActionType;
    }
    
    // Start of user code getterAnnotation:withVariable
    // End of user code
    @OslcName("withVariable")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "withVariable")
    @OslcOccurs(Occurs.ZeroOrMany)
    @OslcValueType(ValueType.LocalResource)
    @OslcRange({PlannerReasonerConstants.TYPE_VARIABLEINSTANCE})
    @OslcReadOnly(false)
    public HashSet<VariableInstance> getWithVariable()
    {
        // Start of user code getterInit:withVariable
        // End of user code
        return withVariable;
    }
    
    // Start of user code getterAnnotation:order
    // End of user code
    @OslcName("order")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "order")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    public Integer getOrder()
    {
        // Start of user code getterInit:order
        // End of user code
        return order;
    }
    
    
    // Start of user code setterAnnotation:ofActionType
    // End of user code
    public void setOfActionType(final Link ofActionType )
    {
        // Start of user code setterInit:ofActionType
        // End of user code
        this.ofActionType = ofActionType;
    
        // Start of user code setterFinalize:ofActionType
        // End of user code
    }
    
    // Start of user code setterAnnotation:withVariable
    // End of user code
    public void setWithVariable(final HashSet<VariableInstance> withVariable )
    {
        // Start of user code setterInit:withVariable
        // End of user code
        this.withVariable.clear();
        if (withVariable != null)
        {
            this.withVariable.addAll(withVariable);
        }
    
        // Start of user code setterFinalize:withVariable
        // End of user code
    }
    
    // Start of user code setterAnnotation:order
    // End of user code
    public void setOrder(final Integer order )
    {
        // Start of user code setterInit:order
        // End of user code
        this.order = order;
    
        // Start of user code setterFinalize:order
        // End of user code
    }
    
    
    static public String ofActionTypeToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:ofActionTypeToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"ofActionType\">ofActionType: </LABEL>";
    
        // Start of user code "Mid:ofActionTypeToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:ofActionTypeToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String withVariableToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:withVariableToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"withVariable\">withVariable: </LABEL>";
    
        // Start of user code "Mid:withVariableToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:withVariableToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String orderToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:orderToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"order\">order: </LABEL>";
    
        // Start of user code "Mid:orderToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"order\" type=\"text\" style=\"width: 400px\" id=\"order\" >";
        // Start of user code "Finalize:orderToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String ofActionTypeToHtml()
    {
        String s = "";
    
        // Start of user code ofActionTypetoHtml_init
        // End of user code
    
        s = s + "<label for=\"ofActionType\"><strong>ofActionType</strong>: </LABEL>";
    
        // Start of user code ofActionTypetoHtml_mid
        // End of user code
    
        try {
            if ((ofActionType == null) || (ofActionType.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new ActionType (ofActionType.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code ofActionTypetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String withVariableToHtml()
    {
        String s = "";
    
        // Start of user code withVariabletoHtml_init
        // End of user code
    
        s = s + "<label for=\"withVariable\"><strong>withVariable</strong>: </LABEL>";
    
        // Start of user code withVariabletoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(VariableInstance next : withVariable) {
                s = s + "<li>";
                s = s + next.toHtml(true);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code withVariabletoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String orderToHtml()
    {
        String s = "";
    
        // Start of user code ordertoHtml_init
        // End of user code
    
        s = s + "<label for=\"order\"><strong>order</strong>: </LABEL>";
    
        // Start of user code ordertoHtml_mid
        // End of user code
    
        try {
            if (order == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + order.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code ordertoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
