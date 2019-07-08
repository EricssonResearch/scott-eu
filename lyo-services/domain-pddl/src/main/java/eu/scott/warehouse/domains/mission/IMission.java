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
 *	   Sam Padgett	       - initial API and implementation
 *     Michael Fiedler     - adapted for OSLC4J
 *     Jad El-khoury        - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/
// End of user code

package eu.scott.warehouse.domains.mission;

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

import eu.scott.warehouse.domains.mission.MissionDomainConstants;
import eu.scott.warehouse.domains.mission.MissionDomainConstants;
import eu.scott.warehouse.domains.mission.IGoal;

// Start of user code imports
// End of user code

@OslcNamespace(MissionDomainConstants.MISSION_NAMESPACE)
@OslcName(MissionDomainConstants.MISSION_LOCALNAME)
@OslcResourceShape(title = "Mission Resource Shape", describes = MissionDomainConstants.MISSION_TYPE)
public interface IMission
{


    @OslcName("goal")
    @OslcPropertyDefinition(MissionDomainConstants.MISSIONONTOLOGY_NAMSPACE + "goal")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.LocalResource)
    @OslcRange({MissionDomainConstants.GOAL_TYPE})
    @OslcReadOnly(false)
    public Goal getGoal();

    @OslcName("responseTimeout")
    @OslcPropertyDefinition(MissionDomainConstants.MISSIONONTOLOGY_NAMSPACE + "responseTimeout")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.Double)
    @OslcReadOnly(false)
    public Double getResponseTimeout();

    @OslcName("missionDeadline")
    @OslcPropertyDefinition(MissionDomainConstants.MISSIONONTOLOGY_NAMSPACE + "missionDeadline")
    @OslcOccurs(Occurs.ExactlyOne)
    @OslcValueType(ValueType.DateTime)
    @OslcReadOnly(false)
    public Date getMissionDeadline();


    public void setGoal(final Goal goal );
    public void setResponseTimeout(final Double responseTimeout );
    public void setMissionDeadline(final Date missionDeadline );
}

