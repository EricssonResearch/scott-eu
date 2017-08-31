/*******************************************************************************
 * Copyright (c) 2011, 2012 IBM Corporation and others.
 *
 *  All rights reserved. This program and the accompanying materials
 *  are made available under the terms of the Eclipse Public License v1.0
 *  and Eclipse Distribution License v. 1.0 which accompanies this distribution.
 *  
 *  The Eclipse Public License is available at http://www.eclipse.org/legal/epl-v10.html
 *  and the Eclipse Distribution License is available at
 *  http://www.eclipse.org/org/documents/edl-v10.php.
 *  
 *  Contributors:
 *  
 *	   Sam Padgett	       - initial API and implementation
 *     Michael Fiedler     - adapted for OSLC4J
 *     Jad El-khoury        - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 *     Matthieu Helleboid   - Support for multiple Service Providers.
 *     Anass Radouani       - Support for multiple Service Providers.
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/

package se.ericsson.cf.scott.sandbox;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;
import java.util.List;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import se.ericsson.cf.scott.sandbox.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.ServiceProviderInfo;
import scott.lyo.domain.planning.Action;
import scott.lyo.domain.planning.ActionType;
import scott.lyo.domain.planning.Mission;
import scott.lyo.domain.warehouse.Place;
import scott.lyo.domain.planning.Plan;
import scott.lyo.domain.planning.Predicate;
import scott.lyo.domain.planning.ProblemState;
import scott.lyo.domain.warehouse.Robot;
import scott.lyo.domain.planning.Variable;
import scott.lyo.domain.planning.VariableInstance;
import scott.lyo.domain.warehouse.Waypoint;
import scott.lyo.domain.warehouse.WhObject;


// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code

public class PlannerReasonerManager {

    // Start of user code class_attributes
    // End of user code
    
    
    // Start of user code class_methods
    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {
        
        // Start of user code contextInitializeServletListener
        // TODO Implement code to establish connection to data backbone etc ...
        // End of user code
    }

    public static void contextDestroyServletListener(ServletContextEvent servletContextEvent) 
    {
        
        // Start of user code contextDestroyed
        // TODO Implement code to shutdown connections to data backbone etc...
        // End of user code
    }

    public static ServiceProviderInfo[] getServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        ServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "ServiceProviderInfo[] getServiceProviderInfos(...)"
        // TODO Implement code to return the set of ServiceProviders
        // End of user code
        return serviceProviderInfos;
    }



    public static Plan getPlan(HttpServletRequest httpServletRequest, final String serviceProviderId, final String planId)
    {
        Plan aResource = null;
        
        // Start of user code getPlan
        // TODO Implement code to return a resource
        // End of user code
        return aResource;
    }




    public static String getETagFromPlan(final Plan aResource)
    {
        String eTag = null;
        // Start of user code getETagFromPlan
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}