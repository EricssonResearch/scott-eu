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
 *     Michael Fiedler      - Bugzilla adpater implementations
 *     Jad El-khoury        - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 * 
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/

package scott.lyo.domain.planning;

import org.eclipse.lyo.oslc4j.core.model.OslcConstants;

// Start of user code imports
// End of user code

public interface PpConstants
{
    // Start of user code user constants
    // End of user code

    public static String PLANNING_DOMAIN = "http://ontology.cf.ericsson.net/planning_problem#";
    public static String PLANNING_NAMSPACE = "http://ontology.cf.ericsson.net/planning_problem#";
    public static String PLANNING_NAMSPACE_PREFIX = "pp";

    public static String ACTION = "Action";
    public static String PATH_ACTION = "action";
    public static String TYPE_ACTION = PLANNING_NAMSPACE + "action";
    public static String ACTIONTYPE = "ActionType";
    public static String PATH_ACTIONTYPE = "actionType";
    public static String TYPE_ACTIONTYPE = PLANNING_NAMSPACE + "actionType";
    public static String MISSION = "Mission";
    public static String PATH_MISSION = "mission";
    public static String TYPE_MISSION = PLANNING_NAMSPACE + "mission";
    public static String PLAN = "Plan";
    public static String PATH_PLAN = "plan";
    public static String TYPE_PLAN = PLANNING_NAMSPACE + "plan";
    public static String PREDICATE = "Predicate";
    public static String PATH_PREDICATE = "predicate";
    public static String TYPE_PREDICATE = PLANNING_NAMSPACE + "predicate";
    public static String PROBLEMSTATE = "ProblemState";
    public static String PATH_PROBLEMSTATE = "problemState";
    public static String TYPE_PROBLEMSTATE = PLANNING_NAMSPACE + "problemState";
    public static String VARIABLE = "Variable";
    public static String PATH_VARIABLE = "variable";
    public static String TYPE_VARIABLE = PLANNING_NAMSPACE + "variable";
    public static String VARIABLEINSTANCE = "VariableInstance";
    public static String PATH_VARIABLEINSTANCE = "variableInstance";
    public static String TYPE_VARIABLEINSTANCE = PLANNING_NAMSPACE + "variableInstance";
}