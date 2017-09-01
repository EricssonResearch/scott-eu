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

import se.ericsson.cf.scott.sandbox.PlannerReasonerConstants;
import se.ericsson.cf.scott.sandbox.resources.IActionType;
import se.ericsson.cf.scott.sandbox.resources.IVariableInstance;

// Start of user code imports
// End of user code

@OslcNamespace(PlannerReasonerConstants.PLANNING_NAMSPACE)
@OslcName(PlannerReasonerConstants.ACTION)
@OslcResourceShape(title = "Action Resource Shape", describes = PlannerReasonerConstants.TYPE_ACTION)
public interface IAction
{

    public void addWithVariable(final VariableInstance withVariable );

    @OslcName("ofActionType")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "ofActionType")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Resource)
    @OslcRange({PlannerReasonerConstants.TYPE_ACTIONTYPE})
    @OslcReadOnly(false)
    public Link getOfActionType();

    @OslcName("withVariable")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "withVariable")
    @OslcOccurs(Occurs.ZeroOrMany)
    @OslcValueType(ValueType.LocalResource)
    @OslcRange({PlannerReasonerConstants.TYPE_VARIABLEINSTANCE})
    @OslcReadOnly(false)
    public HashSet<VariableInstance> getWithVariable();

    @OslcName("order")
    @OslcPropertyDefinition(PlannerReasonerConstants.PLANNING_NAMSPACE + "order")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Integer)
    @OslcReadOnly(false)
    public Integer getOrder();


    public void setOfActionType(final Link ofActionType );
    public void setWithVariable(final HashSet<VariableInstance> withVariable );
    public void setOrder(final Integer order );
}

