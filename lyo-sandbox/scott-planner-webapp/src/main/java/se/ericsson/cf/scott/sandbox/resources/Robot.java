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
import se.ericsson.cf.scott.sandbox.resources.Waypoint;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PlannerReasonerConstants.WAREHOUSE_NAMSPACE)
@OslcName(PlannerReasonerConstants.ROBOT)
@OslcResourceShape(title = "Robot Resource Shape", describes = PlannerReasonerConstants.TYPE_ROBOT)
public class Robot
    extends AbstractResource
    implements IRobot
{
    // Start of user code attributeAnnotation:capacity
    // End of user code
    private Integer capacity;
    // Start of user code attributeAnnotation:chargeLevel
    // End of user code
    private Integer chargeLevel;
    // Start of user code attributeAnnotation:isAt
    // End of user code
    private Link isAt = new Link();
    // Start of user code attributeAnnotation:isCharging
    // End of user code
    private Boolean isCharging;
    // Start of user code attributeAnnotation:maxCharge
    // End of user code
    private Integer maxCharge;
    // Start of user code attributeAnnotation:highCharge
    // End of user code
    private Integer highCharge;
    // Start of user code attributeAnnotation:lowCharge
    // End of user code
    private Integer lowCharge;
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public Robot()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public Robot(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PlannerReasonerConstants.PATH_ROBOT,  
        Robot.class);
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
            result = result + "{a Local Robot Resource} - update Robot.toString() to present resource as desired.";
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
    
    // Start of user code getterAnnotation:chargeLevel
    // End of user code
    @OslcName("chargeLevel")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "chargeLevel")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    public Integer getChargeLevel()
    {
        // Start of user code getterInit:chargeLevel
        // End of user code
        return chargeLevel;
    }
    
    // Start of user code getterAnnotation:isAt
    // End of user code
    @OslcName("isAt")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "isAt")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_WAYPOINT})
    @OslcReadOnly(false)
    public Link getIsAt()
    {
        // Start of user code getterInit:isAt
        // End of user code
        return isAt;
    }
    
    // Start of user code getterAnnotation:isCharging
    // End of user code
    @OslcName("isCharging")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "isCharging")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Boolean)
    @OslcReadOnly(false)
    public Boolean isIsCharging()
    {
        // Start of user code getterInit:isCharging
        // End of user code
        return isCharging;
    }
    
    // Start of user code getterAnnotation:maxCharge
    // End of user code
    @OslcName("maxCharge")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "maxCharge")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    public Integer getMaxCharge()
    {
        // Start of user code getterInit:maxCharge
        // End of user code
        return maxCharge;
    }
    
    // Start of user code getterAnnotation:highCharge
    // End of user code
    @OslcName("highCharge")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "highCharge")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    public Integer getHighCharge()
    {
        // Start of user code getterInit:highCharge
        // End of user code
        return highCharge;
    }
    
    // Start of user code getterAnnotation:lowCharge
    // End of user code
    @OslcName("lowCharge")
    @OslcPropertyDefinition(PlannerReasonerConstants.WAREHOUSE_NAMSPACE + "lowCharge")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    public Integer getLowCharge()
    {
        // Start of user code getterInit:lowCharge
        // End of user code
        return lowCharge;
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
    
    // Start of user code setterAnnotation:chargeLevel
    // End of user code
    public void setChargeLevel(final Integer chargeLevel )
    {
        // Start of user code setterInit:chargeLevel
        // End of user code
        this.chargeLevel = chargeLevel;
    
        // Start of user code setterFinalize:chargeLevel
        // End of user code
    }
    
    // Start of user code setterAnnotation:isAt
    // End of user code
    public void setIsAt(final Link isAt )
    {
        // Start of user code setterInit:isAt
        // End of user code
        this.isAt = isAt;
    
        // Start of user code setterFinalize:isAt
        // End of user code
    }
    
    // Start of user code setterAnnotation:isCharging
    // End of user code
    public void setIsCharging(final Boolean isCharging )
    {
        // Start of user code setterInit:isCharging
        // End of user code
        this.isCharging = isCharging;
    
        // Start of user code setterFinalize:isCharging
        // End of user code
    }
    
    // Start of user code setterAnnotation:maxCharge
    // End of user code
    public void setMaxCharge(final Integer maxCharge )
    {
        // Start of user code setterInit:maxCharge
        // End of user code
        this.maxCharge = maxCharge;
    
        // Start of user code setterFinalize:maxCharge
        // End of user code
    }
    
    // Start of user code setterAnnotation:highCharge
    // End of user code
    public void setHighCharge(final Integer highCharge )
    {
        // Start of user code setterInit:highCharge
        // End of user code
        this.highCharge = highCharge;
    
        // Start of user code setterFinalize:highCharge
        // End of user code
    }
    
    // Start of user code setterAnnotation:lowCharge
    // End of user code
    public void setLowCharge(final Integer lowCharge )
    {
        // Start of user code setterInit:lowCharge
        // End of user code
        this.lowCharge = lowCharge;
    
        // Start of user code setterFinalize:lowCharge
        // End of user code
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
    
    static public String chargeLevelToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:chargeLevelToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"chargeLevel\">chargeLevel: </LABEL>";
    
        // Start of user code "Mid:chargeLevelToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"chargeLevel\" type=\"text\" style=\"width: 400px\" id=\"chargeLevel\" >";
        // Start of user code "Finalize:chargeLevelToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String isAtToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:isAtToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"isAt\">isAt: </LABEL>";
    
        // Start of user code "Mid:isAtToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:isAtToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String isChargingToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:isChargingToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"isCharging\">isCharging: </LABEL>";
    
        // Start of user code "Mid:isChargingToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"isCharging\" type=\"radio\" value=\"true\">True<input name=\"isCharging\" type=\"radio\" value=\"false\">False";
        // Start of user code "Finalize:isChargingToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String maxChargeToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:maxChargeToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"maxCharge\">maxCharge: </LABEL>";
    
        // Start of user code "Mid:maxChargeToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"maxCharge\" type=\"text\" style=\"width: 400px\" id=\"maxCharge\" >";
        // Start of user code "Finalize:maxChargeToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String highChargeToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:highChargeToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"highCharge\">highCharge: </LABEL>";
    
        // Start of user code "Mid:highChargeToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"highCharge\" type=\"text\" style=\"width: 400px\" id=\"highCharge\" >";
        // Start of user code "Finalize:highChargeToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String lowChargeToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:lowChargeToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"lowCharge\">lowCharge: </LABEL>";
    
        // Start of user code "Mid:lowChargeToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"lowCharge\" type=\"text\" style=\"width: 400px\" id=\"lowCharge\" >";
        // Start of user code "Finalize:lowChargeToHtmlForCreation(...)"
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
    
    public String chargeLevelToHtml()
    {
        String s = "";
    
        // Start of user code chargeLeveltoHtml_init
        // End of user code
    
        s = s + "<label for=\"chargeLevel\"><strong>chargeLevel</strong>: </LABEL>";
    
        // Start of user code chargeLeveltoHtml_mid
        // End of user code
    
        try {
            if (chargeLevel == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + chargeLevel.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code chargeLeveltoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String isAtToHtml()
    {
        String s = "";
    
        // Start of user code isAttoHtml_init
        // End of user code
    
        s = s + "<label for=\"isAt\"><strong>isAt</strong>: </LABEL>";
    
        // Start of user code isAttoHtml_mid
        // End of user code
    
        try {
            if ((isAt == null) || (isAt.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new Waypoint (isAt.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code isAttoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String isChargingToHtml()
    {
        String s = "";
    
        // Start of user code isChargingtoHtml_init
        // End of user code
    
        s = s + "<label for=\"isCharging\"><strong>isCharging</strong>: </LABEL>";
    
        // Start of user code isChargingtoHtml_mid
        // End of user code
    
        try {
            if (isCharging == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + isCharging.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code isChargingtoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String maxChargeToHtml()
    {
        String s = "";
    
        // Start of user code maxChargetoHtml_init
        // End of user code
    
        s = s + "<label for=\"maxCharge\"><strong>maxCharge</strong>: </LABEL>";
    
        // Start of user code maxChargetoHtml_mid
        // End of user code
    
        try {
            if (maxCharge == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + maxCharge.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code maxChargetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String highChargeToHtml()
    {
        String s = "";
    
        // Start of user code highChargetoHtml_init
        // End of user code
    
        s = s + "<label for=\"highCharge\"><strong>highCharge</strong>: </LABEL>";
    
        // Start of user code highChargetoHtml_mid
        // End of user code
    
        try {
            if (highCharge == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + highCharge.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code highChargetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String lowChargeToHtml()
    {
        String s = "";
    
        // Start of user code lowChargetoHtml_init
        // End of user code
    
        s = s + "<label for=\"lowCharge\"><strong>lowCharge</strong>: </LABEL>";
    
        // Start of user code lowChargetoHtml_mid
        // End of user code
    
        try {
            if (lowCharge == null) {
                s= s + "<em>null</em>";
            }
            else {
                s= s + lowCharge.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code lowChargetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
