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

package eu.scott.warehouse.domains.scott;

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

import eu.scott.warehouse.domains.scott.ScottDomainConstants;


import eu.scott.warehouse.domains.scott.ScottDomainConstants;
import eu.scott.warehouse.domains.scott.Coord;
import eu.scott.warehouse.domains.scott.Coord;

// Start of user code imports
// End of user code

// Start of user code preClassCode
// End of user code

// Start of user code classAnnotations
// End of user code
@OslcNamespace(ScottDomainConstants.CONVEYORBELT_NAMESPACE)
@OslcName(ScottDomainConstants.CONVEYORBELT_LOCALNAME)
@OslcResourceShape(title = "ConveyorBelt Resource Shape", describes = ScottDomainConstants.CONVEYORBELT_TYPE)
public class ConveyorBelt
    extends AbstractResource
    implements IConveyorBelt
{
    // Start of user code attributeAnnotation:atX
    // End of user code
    private Link atX;
    // Start of user code attributeAnnotation:atY
    // End of user code
    private Link atY;
    
    // Start of user code classAttributes
    // End of user code
    // Start of user code classMethods
    // End of user code
    public ConveyorBelt()
    {
        super();
    
        // Start of user code constructor1
        // End of user code
    }
    
    public ConveyorBelt(final URI about)
    {
        super(about);
    
        // Start of user code constructor2
        // End of user code
    }
    
    public static ResourceShape createResourceShape() throws OslcCoreApplicationException, URISyntaxException {
        return ResourceShapeFactory.createResourceShape(OSLC4JUtils.getServletURI(),
        OslcConstants.PATH_RESOURCE_SHAPES,
        ScottDomainConstants.CONVEYORBELT_PATH,
        ConveyorBelt.class);
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
            result = result + "{a Local ConveyorBelt Resource} - update ConveyorBelt.toString() to present resource as desired.";
            // Start of user code toString_bodyForLocalResource
            // End of user code
        }
        else {
            result = String.valueOf(getAbout());
        }
    
        // Start of user code toString_finalize
        // End of user code
    
        return result;
    }
    
    
    // Start of user code getterAnnotation:atX
    // End of user code
    @OslcName("at-x")
    @OslcPropertyDefinition(ScottDomainConstants.SCOTT_WAREHOUSE_NAMSPACE + "at-x")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({ScottDomainConstants.COORD_TYPE})
    @OslcReadOnly(false)
    public Link getAtX()
    {
        // Start of user code getterInit:atX
        // End of user code
        return atX;
    }
    
    // Start of user code getterAnnotation:atY
    // End of user code
    @OslcName("at-y")
    @OslcPropertyDefinition(ScottDomainConstants.SCOTT_WAREHOUSE_NAMSPACE + "at-y")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({ScottDomainConstants.COORD_TYPE})
    @OslcReadOnly(false)
    public Link getAtY()
    {
        // Start of user code getterInit:atY
        // End of user code
        return atY;
    }
    
    
    // Start of user code setterAnnotation:atX
    // End of user code
    public void setAtX(final Link atX )
    {
        // Start of user code setterInit:atX
        // End of user code
        this.atX = atX;
    
        // Start of user code setterFinalize:atX
        // End of user code
    }
    
    // Start of user code setterAnnotation:atY
    // End of user code
    public void setAtY(final Link atY )
    {
        // Start of user code setterInit:atY
        // End of user code
        this.atY = atY;
    
        // Start of user code setterFinalize:atY
        // End of user code
    }
    
    
}
