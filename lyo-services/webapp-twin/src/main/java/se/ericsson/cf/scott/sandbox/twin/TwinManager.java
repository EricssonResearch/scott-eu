// Start of user code Copyright
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
// End of user code

package se.ericsson.cf.scott.sandbox.twin;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.ServletContextEvent;
import java.util.List;

import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.AbstractResource;
import se.ericsson.cf.scott.sandbox.twin.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.twin.RobotsServiceProviderInfo;
import se.ericsson.cf.scott.sandbox.twin.BeltsServiceProviderInfo;
import se.ericsson.cf.scott.sandbox.twin.ShelvesServiceProviderInfo;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.twins.PlanExecutionRequest;
import eu.scott.warehouse.domains.pddl.Step;


// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code

public class TwinManager {

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

    public static RobotsServiceProviderInfo[] getRobotsServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        RobotsServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "RobotsServiceProviderInfo[] getRobotsServiceProviderInfos(...)"
        // TODO Implement code to return the set of ServiceProviders
        // End of user code
        return serviceProviderInfos;
    }
    public static BeltsServiceProviderInfo[] getBeltsServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        BeltsServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "BeltsServiceProviderInfo[] getBeltsServiceProviderInfos(...)"
        // TODO Implement code to return the set of ServiceProviders
        // End of user code
        return serviceProviderInfos;
    }
    public static ShelvesServiceProviderInfo[] getShelvesServiceProviderInfos(HttpServletRequest httpServletRequest)
    {
        ShelvesServiceProviderInfo[] serviceProviderInfos = {};
        
        // Start of user code "ShelvesServiceProviderInfo[] getShelvesServiceProviderInfos(...)"
        // TODO Implement code to return the set of ServiceProviders
        // End of user code
        return serviceProviderInfos;
    }

    public static PlanExecutionRequest createPlanExecutionRequest(HttpServletRequest httpServletRequest, final PlanExecutionRequest aResource, final String serviceProviderId)
    {
        PlanExecutionRequest newResource = null;
        
        // Start of user code createPlanExecutionRequest
        // TODO Implement code to create a resource
        // FIXME Andrew@2018-08-29: will be called for all SP kinds
        // End of user code
        return newResource;
    }



/*

    public static PlanExecutionRequest createPlanExecutionRequest(HttpServletRequest httpServletRequest, final PlanExecutionRequest aResource, final String beltId)
    {
        PlanExecutionRequest newResource = null;
        
        // Start of user code createPlanExecutionRequest
        // TODO Implement code to create a resource
        // End of user code
        return newResource;
    }



    public static PlanExecutionRequest createPlanExecutionRequest(HttpServletRequest httpServletRequest, final PlanExecutionRequest aResource, final String serviceProviderId)
    {
        PlanExecutionRequest newResource = null;
        
        // Start of user code createPlanExecutionRequest
        // TODO Implement code to create a resource
        // End of user code
        return newResource;
    }
*/




    public static String getETagFromPlanExecutionRequest(final PlanExecutionRequest aResource)
    {
        String eTag = null;
        // Start of user code getETagFromPlanExecutionRequest
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}
