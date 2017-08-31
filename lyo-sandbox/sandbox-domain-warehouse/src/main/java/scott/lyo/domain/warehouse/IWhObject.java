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
 *	   Sam Padgett	       - initial API and implementation
 *     Michael Fiedler     - adapted for OSLC4J
 *     Jad El-khoury        - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/

package scott.lyo.domain.warehouse;

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

import scott.lyo.domain.warehouse.WhConstants;
import scott.lyo.domain.warehouse.WhConstants;
import scott.lyo.domain.warehouse.IRobot;
import scott.lyo.domain.warehouse.IPlace;

// Start of user code imports
// End of user code

@OslcNamespace(WhConstants.WAREHOUSE_NAMSPACE)
@OslcName(WhConstants.WHOBJECT)
@OslcResourceShape(title = "WhObject Resource Shape", describes = WhConstants.TYPE_WHOBJECT)
public interface IWhObject
{


    @OslcName("isOn")
    @OslcPropertyDefinition(WhConstants.WAREHOUSE_NAMSPACE + "isOn")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({WhConstants.TYPE_PLACE})
    @OslcReadOnly(false)
    public Link getIsOn();

    @OslcName("carriedBy")
    @OslcPropertyDefinition(WhConstants.WAREHOUSE_NAMSPACE + "carriedBy")
    @OslcOccurs(Occurs.ZeroOrOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({WhConstants.TYPE_ROBOT})
    @OslcReadOnly(false)
    public Link getCarriedBy();

    @OslcName("type")
    @OslcPropertyDefinition(WhConstants.WAREHOUSE_NAMSPACE + "type")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.String)
    @OslcReadOnly(false)
    public String getType();

    @OslcName("capacity")
    @OslcPropertyDefinition(WhConstants.WAREHOUSE_NAMSPACE + "capacity")
    @OslcOccurs(Occurs.ZeroOrOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    @OslcTitle("")
    public Integer getCapacity();


    public void setIsOn(final Link isOn );
    public void setCarriedBy(final Link carriedBy );
    public void setType(final String type );
    public void setCapacity(final Integer capacity );
}
