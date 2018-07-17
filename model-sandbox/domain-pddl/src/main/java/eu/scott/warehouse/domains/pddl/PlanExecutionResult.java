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
// End of user code

package eu.scott.warehouse.domains.pddl;

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

import eu.scott.warehouse.domains.pddl.PddlDomainConstants;
import eu.scott.warehouse.domains.pddl.PddlDomainConstants;
import eu.scott.warehouse.domains.pddl.Plan;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PddlDomainConstants.PLANEXECUTIONRESULT_NAMESPACE)
@OslcName(PddlDomainConstants.PLANEXECUTIONRESULT_LOCALNAME)
@OslcResourceShape(title = "PlanExecutionResult Resource Shape", describes = PddlDomainConstants.PLANEXECUTIONRESULT_TYPE)
public class PlanExecutionResult
    extends AbstractResource
    implements IPlanExecutionResult
{
    // Start of user code attributeAnnotation:plan
    // End of user code
    private Link plan = new Link();
    // Start of user code attributeAnnotation:isSuccessful
    // End of user code
    private Boolean isSuccessful;
    // Start of user code attributeAnnotation:durationSeconds
    // End of user code
    private Double durationSeconds;
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public PlanExecutionResult()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public PlanExecutionResult(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    /**
    * @deprecated Use the methods in class {@link se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinResourcesFactory} instead.
    */
    @Deprecated
    public PlanExecutionResult(final String serviceProviderId, final String planExecutionResultId)
           throws URISyntaxException
    {
        this (constructURI(serviceProviderId, planExecutionResultId));
        // Start of user code constructor3
        // End of user code
    }
    
    /**
    * @deprecated Use the methods in class {@link se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinResourcesFactory} instead.
    */
    @Deprecated
    public static URI constructURI(final String serviceProviderId, final String planExecutionResultId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("planExecutionResultId", planExecutionResultId);
        String instanceURI = "serviceProviders/{serviceProviderId}/planExecutionResults/{planExecutionResultId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    /**
    * @deprecated Use the methods in class {@link se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinResourcesFactory} instead.
    */
    @Deprecated
    public static Link constructLink(final String serviceProviderId, final String planExecutionResultId , final String label)
    {
        return new Link(constructURI(serviceProviderId, planExecutionResultId), label);
    }
    
    /**
    * @deprecated Use the methods in class {@link se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinResourcesFactory} instead.
    */
    @Deprecated
    public static Link constructLink(final String serviceProviderId, final String planExecutionResultId)
    {
        return new Link(constructURI(serviceProviderId, planExecutionResultId));
    }
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PddlDomainConstants.PLANEXECUTIONRESULT_PATH,
        PlanExecutionResult.class);
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
            result = result + "{a Local PlanExecutionResult Resource} - update PlanExecutionResult.toString() to present resource as desired.";
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
            result = "<a href=\"" + getAbout() + "\" class=\"oslc-resource-link\">" + toString() + "</a>";
        }
    
        // Start of user code toHtml_finalize
        // End of user code
    
        return result;
    }
    
    
    // Start of user code getterAnnotation:plan
    // End of user code
    @OslcName("plan")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "plan")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PddlDomainConstants.PLAN_TYPE})
    @OslcReadOnly(false)
    public Link getPlan()
    {
        // Start of user code getterInit:plan
        // End of user code
        return plan;
    }
    
    // Start of user code getterAnnotation:isSuccessful
    // End of user code
    @OslcName("isSuccessful")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "isSuccessful")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Boolean)
    @OslcReadOnly(false)
    public Boolean isIsSuccessful()
    {
        // Start of user code getterInit:isSuccessful
        // End of user code
        return isSuccessful;
    }
    
    // Start of user code getterAnnotation:durationSeconds
    // End of user code
    @OslcName("durationSeconds")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "durationSeconds")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Double)
    @OslcReadOnly(false)
    public Double getDurationSeconds()
    {
        // Start of user code getterInit:durationSeconds
        // End of user code
        return durationSeconds;
    }
    
    
    // Start of user code setterAnnotation:plan
    // End of user code
    public void setPlan(final Link plan )
    {
        // Start of user code setterInit:plan
        // End of user code
        this.plan = plan;
    
        // Start of user code setterFinalize:plan
        // End of user code
    }
    
    // Start of user code setterAnnotation:isSuccessful
    // End of user code
    public void setIsSuccessful(final Boolean isSuccessful )
    {
        // Start of user code setterInit:isSuccessful
        // End of user code
        this.isSuccessful = isSuccessful;
    
        // Start of user code setterFinalize:isSuccessful
        // End of user code
    }
    
    // Start of user code setterAnnotation:durationSeconds
    // End of user code
    public void setDurationSeconds(final Double durationSeconds )
    {
        // Start of user code setterInit:durationSeconds
        // End of user code
        this.durationSeconds = durationSeconds;
    
        // Start of user code setterFinalize:durationSeconds
        // End of user code
    }
    
    
    static public String planToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:planToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"plan\">plan: </LABEL>";
    
        // Start of user code "Mid:planToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:planToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String isSuccessfulToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:isSuccessfulToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"isSuccessful\">isSuccessful: </LABEL>";
    
        // Start of user code "Mid:isSuccessfulToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"isSuccessful\" type=\"radio\" value=\"true\">True<input name=\"isSuccessful\" type=\"radio\" value=\"false\">False";
        // Start of user code "Finalize:isSuccessfulToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String durationSecondsToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:durationSecondsToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"durationSeconds\">durationSeconds: </LABEL>";
    
        // Start of user code "Mid:durationSecondsToHtmlForCreation(...)"
        // End of user code
    
        s= s + "<input name=\"durationSeconds\" type=\"text\" style=\"width: 400px\" id=\"durationSeconds\" >";
        // Start of user code "Finalize:durationSecondsToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String planToHtml()
    {
        String s = "";
    
        // Start of user code plantoHtml_mid
        // End of user code
    
        try {
            if ((plan == null) || (plan.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new Plan (plan.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code plantoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String isSuccessfulToHtml()
    {
        String s = "";
    
        // Start of user code isSuccessfultoHtml_mid
        // End of user code
    
        try {
            if (isSuccessful == null) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + isSuccessful.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code isSuccessfultoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String durationSecondsToHtml()
    {
        String s = "";
    
        // Start of user code durationSecondstoHtml_mid
        // End of user code
    
        try {
            if (durationSeconds == null) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + durationSeconds.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code durationSecondstoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
