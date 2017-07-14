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

import se.ericsson.cf.scott.sandbox.TwinConstants;
import se.ericsson.cf.scott.sandbox.resources.Waypoint;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(TwinConstants.WAREHOUSE_NAMSPACE)
@OslcName(TwinConstants.PLACE)
@OslcResourceShape(title = "Place Resource Shape", describes = TwinConstants.TYPE_PLACE)
public class Place
    extends AbstractResource
    implements IPlace
{
    // Start of user code attributeAnnotation:situatedAt
    // End of user code
    private Link situatedAt = new Link();
    // Start of user code attributeAnnotation:isChargingStation
    // End of user code
    private Boolean isChargingStation;
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
    public Place()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public Place(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        TwinConstants.PATH_PLACE,
        Place.class);
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
            result = result + "{a Local Place Resource} - update Place.toString() to present resource as desired.";
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
    
    
    // Start of user code getterAnnotation:situatedAt
    // End of user code
    @OslcName("situatedAt")
    @OslcPropertyDefinition(TwinConstants.WAREHOUSE_NAMSPACE + "situatedAt")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({TwinConstants.TYPE_WAYPOINT})
    @OslcReadOnly(false)
    public Link getSituatedAt()
    {
        // Start of user code getterInit:situatedAt
        // End of user code
        return situatedAt;
    }
    
    // Start of user code getterAnnotation:isChargingStation
    // End of user code
    @OslcName("isChargingStation")
    @OslcPropertyDefinition(TwinConstants.WAREHOUSE_NAMSPACE + "isChargingStation")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Boolean)
    @OslcReadOnly(false)
    @OslcTitle("")
    public Boolean isIsChargingStation()
    {
        // Start of user code getterInit:isChargingStation
        // End of user code
        return isChargingStation;
    }
    
    // Start of user code getterAnnotation:type
    // End of user code
    @OslcName("type")
    @OslcPropertyDefinition(TwinConstants.WAREHOUSE_NAMSPACE + "type")
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
    @OslcPropertyDefinition(TwinConstants.WAREHOUSE_NAMSPACE + "capacity")
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
    
    
    // Start of user code setterAnnotation:situatedAt
    // End of user code
    public void setSituatedAt(final Link situatedAt )
    {
        // Start of user code setterInit:situatedAt
        // End of user code
        this.situatedAt = situatedAt;
    
        // Start of user code setterFinalize:situatedAt
        // End of user code
    }
    
    // Start of user code setterAnnotation:isChargingStation
    // End of user code
    public void setIsChargingStation(final Boolean isChargingStation )
    {
        // Start of user code setterInit:isChargingStation
        // End of user code
        this.isChargingStation = isChargingStation;
    
        // Start of user code setterFinalize:isChargingStation
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
    
    
    static public String situatedAtToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:situatedAtToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"situatedAt\">situatedAt: </LABEL>";
    
        // Start of user code "Mid:situatedAtToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:situatedAtToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String isChargingStationToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:isChargingStationToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"isChargingStation\">isChargingStation: </LABEL>";
    
        // Start of user code "Mid:isChargingStationToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"isChargingStation\" type=\"radio\" value=\"true\">True<input name=\"isChargingStation\" type=\"radio\" value=\"false\">False";
        // Start of user code "Finalize:isChargingStationToHtmlForCreation(...)"
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
    
    
    public String situatedAtToHtml()
    {
        String s = "";
    
        // Start of user code situatedAttoHtml_init
        // End of user code
    
        s = s + "<label for=\"situatedAt\"><strong>situatedAt</strong>: &nbsp;</LABEL>";
    
        // Start of user code situatedAttoHtml_mid
        // End of user code
    
        try {
            if ((situatedAt == null) || (situatedAt.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new Waypoint (situatedAt.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code situatedAttoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String isChargingStationToHtml()
    {
        String s = "";
    
        // Start of user code isChargingStationtoHtml_init
        // End of user code
    
        s = s + "<label for=\"isChargingStation\"><strong>isChargingStation</strong>: &nbsp;</LABEL>";
    
        // Start of user code isChargingStationtoHtml_mid
        // End of user code
    
        try {
            if (isChargingStation == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + isChargingStation.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code isChargingStationtoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String typeToHtml()
    {
        String s = "";
    
        // Start of user code typetoHtml_init
        // End of user code
    
        s = s + "<label for=\"type\"><strong>type</strong>: &nbsp;</LABEL>";
    
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
    
        s = s + "<label for=\"capacity\"><strong>capacity</strong>: &nbsp;</LABEL>";
    
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
