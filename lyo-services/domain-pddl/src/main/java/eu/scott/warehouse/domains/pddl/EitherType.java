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
import eu.scott.warehouse.domains.pddl.PrimitiveType;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(PddlDomainConstants.EITHERTYPE_NAMESPACE)
@OslcName(PddlDomainConstants.EITHERTYPE_LOCALNAME)
@OslcResourceShape(title = "EitherType Resource Shape", describes = PddlDomainConstants.EITHERTYPE_TYPE)
public class EitherType
    extends AbstractResource
    implements IEitherType
{
    // Start of user code attributeAnnotation:member
    // End of user code
    private HashSet<Link> member = new HashSet<Link>();
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public EitherType()
           throws URISyntaxException
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public EitherType(final URI about)
           throws URISyntaxException
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        PddlDomainConstants.EITHERTYPE_PATH,
        EitherType.class);
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
            result = result + "{a Local EitherType Resource} - update EitherType.toString() to present resource as desired.";
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
    
    public void addMember(final Link member)
    {
        this.member.add(member);
    }
    
    
    // Start of user code getterAnnotation:member
    // End of user code
    @OslcName("member")
    @OslcPropertyDefinition(PddlDomainConstants.SCOTT_PDDL_2_1_SUBSET_SPEC_NAMSPACE + "member")
    @OslcDescription("Type member (primitive type).")
    @OslcOccurs(Occurs.OneOrMany)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PddlDomainConstants.PRIMITIVETYPE_TYPE})
    @OslcReadOnly(false)
    public HashSet<Link> getMember()
    {
        // Start of user code getterInit:member
        // End of user code
        return member;
    }
    
    
    // Start of user code setterAnnotation:member
    // End of user code
    public void setMember(final Set<Link> member )
    {
        // Start of user code setterInit:member
        // End of user code
        this.member.clear();
        if (member != null)
        {
            this.member.addAll(member);
        }
    
        // Start of user code setterFinalize:member
        // End of user code
    }
    
    
    static public String memberToHtmlForCreation (final HttpServletRequest httpServletRequest)
    {
        String s = "";
    
        // Start of user code "Init:memberToHtmlForCreation(...)"
        // End of user code
    
        s = s + "<label for=\"member\">member: </LABEL>";
    
        // Start of user code "Mid:memberToHtmlForCreation(...)"
        // End of user code
    
        // Start of user code "Finalize:memberToHtmlForCreation(...)"
        // End of user code
    
        return s;
    }
    
    
    public String memberToHtml()
    {
        String s = "";
    
        // Start of user code membertoHtml_mid
        // End of user code
    
        try {
            s = s + "<ul>";
            for(Link next : member) {
                s = s + "<li>";
                s = s + (new PrimitiveType (next.getValue())).toHtml(false);
                s = s + "</li>";
            }
            s = s + "</ul>";
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        // Start of user code membertoHtml_finalize
        // End of user code
    
        return s;
    }
    
    
}
