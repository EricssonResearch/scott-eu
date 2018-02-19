/*******************************************************************************
 * Copyright (c) 2017 Jad El-khoury.
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
 *     Jad El-khoury        - initial implementation
 *     
 *******************************************************************************/

package se.ericsson.cf.scott.sandbox.twin;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import javax.ws.rs.core.UriBuilder;

import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.pddl.PrimitiveType;
import eu.scott.warehouse.domains.pddl.Step;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.blocksworld.Location;

// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code

public class RobotTwinResourcesFactory {

    // Start of user code class_attributes
    // End of user code
    
    // Start of user code class_methods
    // End of user code

    //methods for Plan resource
    public static Plan createPlan(final String serviceProviderId, final String planId)
           throws URISyntaxException
    {
        return new Plan(constructURIForPlan(serviceProviderId, planId));
    }
    
    public static URI constructURIForPlan(final String serviceProviderId, final String planId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("planId", planId);
        String instanceURI = "serviceProviders/{serviceProviderId}/resources/plans/{planId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForPlan(final String serviceProviderId, final String planId , final String label)
    {
        return new Link(constructURIForPlan(serviceProviderId, planId), label);
    }
    
    public static Link constructLinkForPlan(final String serviceProviderId, final String planId)
    {
        return new Link(constructURIForPlan(serviceProviderId, planId));
    }
    

    //methods for Location resource
    public static Location createLocation(final String serviceProviderId, final String locationId)
           throws URISyntaxException
    {
        return new Location(constructURIForLocation(serviceProviderId, locationId));
    }
    
    public static URI constructURIForLocation(final String serviceProviderId, final String locationId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("locationId", locationId);
        String instanceURI = "serviceProviders/{serviceProviderId}/resources/locations/{locationId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForLocation(final String serviceProviderId, final String locationId , final String label)
    {
        return new Link(constructURIForLocation(serviceProviderId, locationId), label);
    }
    
    public static Link constructLinkForLocation(final String serviceProviderId, final String locationId)
    {
        return new Link(constructURIForLocation(serviceProviderId, locationId));
    }
    

}
