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
import eu.scott.warehouse.domains.pddl.Domain;
import eu.scott.warehouse.domains.pddl.PddlObject;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PddlDomainConstants.PROBLEM_NAMESPACE)
@OslcName(PddlDomainConstants.PROBLEM_LOCALNAME)
@OslcResourceShape(title = "Problem Resource Shape", describes = PddlDomainConstants.PROBLEM_TYPE)
public class Problem
    extends AbstractResource
    implements IProblem
{
    // Start of user code attributeAnnotation:domain
    // End of user code
    private Link domain = new Link();
    // Start of user code attributeAnnotation:goal
    // End of user code
    private Link goal = new Link();
    // Start of user code attributeAnnotation:init
    // End of user code
    private HashSet<Link> init = new HashSet<Link>();
    // Start of user code attributeAnnotation:maximize
    // End of user code
    private Link maximize = new Link();
    // Start of user code attributeAnnotation:minimize
    // End of user code
    private Link minimize = new Link();
    // Start of user code attributeAnnotation:pddlObject
    // End of user code
    private HashSet<Link> pddlObject = new HashSet<Link>();
    // Start of user code attributeAnnotation:label
    // End of user code
    private String label;
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public Problem()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public Problem(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PddlDomainConstants.PROBLEM_PATH,
        Problem.class);
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
            result = result + "{a Local Problem Resource} - update Problem.toString() to present resource as desired.";
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
    
    public void addInit(final Link init)
    {
        this.init.add(init);
    }
    
    public void addPddlObject(final Link pddlObject)
    {
        this.pddlObject.add(pddlObject);
    }
    
    
    // Start of user code getterAnnotation:domain
    // End of user code
    @OslcName("domain")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "domain")
    @OslcDescription("Problem planning domain.")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PddlDomainConstants.DOMAIN_TYPE})
    @OslcReadOnly(false)
    public Link getDomain()
    {
        // Start of user code getterInit:domain
        // End of user code
        return domain;
    }
    
    // Start of user code getterAnnotation:goal
    // End of user code
    @OslcName("goal")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "goal")
    @OslcDescription("Problem goal.")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcReadOnly(false)
    public Link getGoal()
    {
        // Start of user code getterInit:goal
        // End of user code
        return goal;
    }
    
    // Start of user code getterAnnotation:init
    // End of user code
    @OslcName("init")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "init")
    @OslcDescription("Problem init.")
    @OslcOccurs(Occurs.ZeroOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcReadOnly(false)
    public HashSet<Link> getInit()
    {
        // Start of user code getterInit:init
        // End of user code
        return init;
    }
    
    // Start of user code getterAnnotation:maximize
    // End of user code
    @OslcName("maximize")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "maximize")
    @OslcDescription("Maximization metric.")
    @OslcOccurs(Occurs.ZeroOrOne)
    @OslcValueType(ValueType.Resource)
    @OslcReadOnly(false)
    public Link getMaximize()
    {
        // Start of user code getterInit:maximize
        // End of user code
        return maximize;
    }
    
    // Start of user code getterAnnotation:minimize
    // End of user code
    @OslcName("minimize")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "minimize")
    @OslcDescription("Minimization metric.")
    @OslcOccurs(Occurs.ZeroOrOne)
    @OslcValueType(ValueType.Resource)
    @OslcReadOnly(false)
    public Link getMinimize()
    {
        // Start of user code getterInit:minimize
        // End of user code
        return minimize;
    }
    
    // Start of user code getterAnnotation:pddlObject
    // End of user code
    @OslcName("pddlObject")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "pddlObject")
    @OslcDescription("Problem objects.")
    @OslcOccurs(Occurs.ZeroOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PddlDomainConstants.OBJECT_TYPE})
    @OslcReadOnly(false)
    public HashSet<Link> getPddlObject()
    {
        // Start of user code getterInit:pddlObject
        // End of user code
        return pddlObject;
    }
    
    // Start of user code getterAnnotation:label
    // End of user code
    @OslcName("label")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "label")
    @OslcDescription("Parameter name.")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.String)
    @OslcReadOnly(false)
    public String getLabel()
    {
        // Start of user code getterInit:label
        // End of user code
        return label;
    }
    
    
    // Start of user code setterAnnotation:domain
    // End of user code
    public void setDomain(final Link domain )
    {
        // Start of user code setterInit:domain
        // End of user code
        this.domain = domain;
    
        // Start of user code setterFinalize:domain
        // End of user code
    }
    
    // Start of user code setterAnnotation:goal
    // End of user code
    public void setGoal(final Link goal )
    {
        // Start of user code setterInit:goal
        // End of user code
        this.goal = goal;
    
        // Start of user code setterFinalize:goal
        // End of user code
    }
    
    // Start of user code setterAnnotation:init
    // End of user code
    public void setInit(final Set<Link> init )
    {
        // Start of user code setterInit:init
        // End of user code
        this.init.clear();
        if (init != null)
        {
            this.init.addAll(init);
        }
    
        // Start of user code setterFinalize:init
        // End of user code
    }
    
    // Start of user code setterAnnotation:maximize
    // End of user code
    public void setMaximize(final Link maximize )
    {
        // Start of user code setterInit:maximize
        // End of user code
        this.maximize = maximize;
    
        // Start of user code setterFinalize:maximize
        // End of user code
    }
    
    // Start of user code setterAnnotation:minimize
    // End of user code
    public void setMinimize(final Link minimize )
    {
        // Start of user code setterInit:minimize
        // End of user code
        this.minimize = minimize;
    
        // Start of user code setterFinalize:minimize
        // End of user code
    }
    
    // Start of user code setterAnnotation:pddlObject
    // End of user code
    public void setPddlObject(final Set<Link> pddlObject )
    {
        // Start of user code setterInit:pddlObject
        // End of user code
        this.pddlObject.clear();
        if (pddlObject != null)
        {
            this.pddlObject.addAll(pddlObject);
        }
    
        // Start of user code setterFinalize:pddlObject
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
    
    
    static public String domainToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:domainToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"domain\">domain: </LABEL>";
    
        // Start of user code "Mid:domainToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:domainToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String goalToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:goalToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"goal\">goal: </LABEL>";
    
        // Start of user code "Mid:goalToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:goalToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String initToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:initToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"init\">init: </LABEL>";
    
        // Start of user code "Mid:initToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:initToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String maximizeToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:maximizeToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"maximize\">maximize: </LABEL>";
    
        // Start of user code "Mid:maximizeToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:maximizeToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String minimizeToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:minimizeToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"minimize\">minimize: </LABEL>";
    
        // Start of user code "Mid:minimizeToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:minimizeToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    static public String pddlObjectToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:pddlObjectToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"object\">object: </LABEL>";
    
        // Start of user code "Mid:pddlObjectToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:pddlObjectToHtmlForCreation(...)"
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
    
    
    public String domainToHtml()
    {
        String s = "";
    
        // Start of user code domaintoHtml_mid
        // End of user code
    
        try {
            if ((domain == null) || (domain.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + (new Domain (domain.getValue())).toHtml(false);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code domaintoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String goalToHtml()
    {
        String s = "";
    
        // Start of user code goaltoHtml_mid
        // End of user code
    
        try {
            if ((goal == null) || (goal.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + goal.getValue().toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code goaltoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String initToHtml()
    {
        String s = "";
    
        // Start of user code inittoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : init) {
                s = s + "<li>";
                if (next.getValue() == null) {
                    s= s + "<em>null</em>";
                }
                else {
                    s = s + "<a href=\"" + next.getValue().toString() + "\">" + next.getValue().toString() + "</a>";
                }
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code inittoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String maximizeToHtml()
    {
        String s = "";
    
        // Start of user code maximizetoHtml_mid
        // End of user code
    
        try {
            if ((maximize == null) || (maximize.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + maximize.getValue().toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code maximizetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String minimizeToHtml()
    {
        String s = "";
    
        // Start of user code minimizetoHtml_mid
        // End of user code
    
        try {
            if ((minimize == null) || (minimize.getValue() == null)) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + minimize.getValue().toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code minimizetoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String pddlObjectToHtml()
    {
        String s = "";
    
        // Start of user code pddlObjecttoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : pddlObject) {
                s = s + "<li>";
                s = s + (new PddlObject (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code pddlObjecttoHtml_finalize
        // End of user code
    
        return s;
    }
    
    public String labelToHtml()
    {
        String s = "";
    
        // Start of user code labeltoHtml_mid
        // End of user code
    
        try {
            if (label == null) {
                s = s + "<em>null</em>";
            }
            else {
                s = s + label.toString();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code labeltoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
