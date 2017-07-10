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
import se.ericsson.cf.scott.sandbox.resources.Robot;
import se.ericsson.cf.scott.sandbox.resources.Place;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PlannerReasonerConstants.WAREHOUSE_NAMSPACE)
@OslcName(PlannerReasonerConstants.WHOBJECT)
@OslcResourceShape(title = "WhObject Resource Shape", describes = PlannerReasonerConstants.TYPE_WHOBJECT)
public class WhObject
    extends AbstractResource
    implements IWhObject
{
    // Start of user code attributeAnnotation:isOn
    // End of user code
    private Link isOn = new Link();
    // Start of user code attributeAnnotation:carriedBy
    // End of user code
    private Link carriedBy = new Link();
    // Start of user code attributeAnnotation:type
    // End of user code
    private String type;
    // Start of user code attributeAnnotation:capacity
    // End of user code
    private Integer capacity;
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public WhObject()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public WhObject(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PlannerReasonerConstants.PATH_WHOBJECT,  
        WhObject.class);
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
            result = result + "{a Local WhObject Resource} - update WhObject.toString() to present resource as desired.";
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
    
    
    // Start of user code getterAnnotation:isOn
    // End of user code
    @OslcName("isOn")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "isOn")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_PLACE})
    @OslcReadOnly(false)
    public Link getIsOn()
    {
        // Start of user code getterInit:isOn
        // End of user code
        return isOn;
    }
    
    // Start of user code getterAnnotation:carriedBy
    // End of user code
    @OslcName("carriedBy")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "carriedBy")
    @OslcOccurs(Occurs.ZeroOrOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_ROBOT})
    @OslcReadOnly(false)
    public Link getCarriedBy()
    {
        // Start of user code getterInit:carriedBy
        // End of user code
        return carriedBy;
    }
    
    // Start of user code getterAnnotation:type
    // End of user code
    @OslcName("type")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "type")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.String)
    @OslcReadOnly(false)
    public String getType()
    {
        // Start of user code getterInit:type
        // End of user code
        return type;
    }
    
    // Start of user code getterAnnotation:capacity
    // End of user code
    @OslcName("capacity")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "capacity")
    @OslcOccurs(Occurs.ZeroOrOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    @OslcTitle("")
    public Integer getCapacity()
    {
        // Start of user code getterInit:capacity
        // End of user code
        return capacity;
    }
    
    
    // Start of user code setterAnnotation:isOn
    // End of user code
    public void setIsOn(final Link isOn )
    {
        // Start of user code setterInit:isOn
        // End of user code
        this.isOn = isOn;
    
        // Start of user code setterFinalize:isOn
        // End of user code
    }
    
    // Start of user code setterAnnotation:carriedBy
    // End of user code
    public void setCarriedBy(final Link carriedBy )
    {
        // Start of user code setterInit:carriedBy
        // End of user code
        this.carriedBy = carriedBy;
    
        // Start of user code setterFinalize:carriedBy
        // End of user code
    }
    
    // Start of user code setterAnnotation:type
    // End of user code
    public void setType(final String type )
    {
        // Start of user code setterInit:type
        // End of user code
        this.type = type;
    
        // Start of user code setterFinalize:type
        // End of user code
    }
    
    // Start of user code setterAnnotation:capacity
    // End of user code
    public void setCapacity(final Integer capacity )
    {
        // Start of user code setterInit:capacity
        // End of user code
        this.capacity = capacity;
    
        // Start of user code setterFinalize:capacity
        // End of user code
    }
    
    
    static public String isOnToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:isOnToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"isOn\">isOn: </LABEL>";
    
        // Start of user code "Mid:isOnToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:isOnToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String carriedByToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:carriedByToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"carriedBy\">carriedBy: </LABEL>";
    
        // Start of user code "Mid:carriedByToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:carriedByToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String typeToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:typeToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"type\">type: </LABEL>";
    
        // Start of user code "Mid:typeToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"type\" type=\"text\" style=\"width: 400px\" id=\"type\" >";
        // Start of user code "Finalize:typeToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String capacityToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:capacityToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"capacity\">capacity: </LABEL>";
    
        // Start of user code "Mid:capacityToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"capacity\" type=\"text\" style=\"width: 400px\" id=\"capacity\" >";
        // Start of user code "Finalize:capacityToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String isOnToHtml()
    {
        String s = "";
    
        // Start of user code isOntoHtml_init
        // End of user code
    
        s = s + "<label for=\"isOn\"><strong>isOn</strong>: </LABEL>";
    
        // Start of user code isOntoHtml_mid
        // End of user code
    
        try {
            if ((isOn == null) || (isOn.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new Place (isOn.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code isOntoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String carriedByToHtml()
    {
        String s = "";
    
        // Start of user code carriedBytoHtml_init
        // End of user code
    
        s = s + "<label for=\"carriedBy\"><strong>carriedBy</strong>: </LABEL>";
    
        // Start of user code carriedBytoHtml_mid
        // End of user code
    
        try {
            if ((carriedBy == null) || (carriedBy.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new Robot (carriedBy.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code carriedBytoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String typeToHtml()
    {
        String s = "";
    
        // Start of user code typetoHtml_init
        // End of user code
    
        s = s + "<label for=\"type\"><strong>type</strong>: </LABEL>";
    
        // Start of user code typetoHtml_mid
        // End of user code
    
        try {
            if (type == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + type.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code typetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String capacityToHtml()
    {
        String s = "";
    
        // Start of user code capacitytoHtml_init
        // End of user code
    
        s = s + "<label for=\"capacity\"><strong>capacity</strong>: </LABEL>";
    
        // Start of user code capacitytoHtml_mid
        // End of user code
    
        try {
            if (capacity == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + capacity.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code capacitytoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
