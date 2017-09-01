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

package se.ericsson.cf.scott.sandbox;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import javax.ws.rs.core.UriBuilder;

import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import se.ericsson.cf.scott.sandbox.resources.Mission;
import se.ericsson.cf.scott.sandbox.resources.Place;
import se.ericsson.cf.scott.sandbox.resources.Predicate;
import se.ericsson.cf.scott.sandbox.resources.WhObject;
import se.ericsson.cf.scott.sandbox.resources.ProblemState;
import se.ericsson.cf.scott.sandbox.resources.Waypoint;
import se.ericsson.cf.scott.sandbox.resources.Robot;

// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code

public class WarehouseControllerResourcesFactory {

    // Start of user code class_attributes
    // End of user code
    
    // Start of user code class_methods
    // End of user code

    //methods for Mission resource
    public static Mission createMission(final String serviceProviderId, final String missionId)
           throws URISyntaxException
    {
        return new Mission(constructURIForMission(serviceProviderId, missionId));
    }
    
    public static URI constructURIForMission(final String serviceProviderId, final String missionId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("missionId", missionId);
        String instanceURI = "serviceProviders/{serviceProviderId}/resources/missions/{missionId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForMission(final String serviceProviderId, final String missionId , final String label)
    {
        return new Link(constructURIForMission(serviceProviderId, missionId), label);
    }
    
    public static Link constructLinkForMission(final String serviceProviderId, final String missionId)
    {
        return new Link(constructURIForMission(serviceProviderId, missionId));
    }
    

    //methods for Place resource
    public static Place createPlace(final String serviceProviderId, final String placeId)
           throws URISyntaxException
    {
        return new Place(constructURIForPlace(serviceProviderId, placeId));
    }
    
    public static URI constructURIForPlace(final String serviceProviderId, final String placeId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("placeId", placeId);
        String instanceURI = "serviceProviders/{serviceProviderId}/resources/places/{placeId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForPlace(final String serviceProviderId, final String placeId , final String label)
    {
        return new Link(constructURIForPlace(serviceProviderId, placeId), label);
    }
    
    public static Link constructLinkForPlace(final String serviceProviderId, final String placeId)
    {
        return new Link(constructURIForPlace(serviceProviderId, placeId));
    }
    

    //methods for WhObject resource
    public static WhObject createWhObject(final String serviceProviderId, final String whObjectId)
           throws URISyntaxException
    {
        return new WhObject(constructURIForWhObject(serviceProviderId, whObjectId));
    }
    
    public static URI constructURIForWhObject(final String serviceProviderId, final String whObjectId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("whObjectId", whObjectId);
        String instanceURI = "serviceProviders/{serviceProviderId}/resources/whObjects/{whObjectId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForWhObject(final String serviceProviderId, final String whObjectId , final String label)
    {
        return new Link(constructURIForWhObject(serviceProviderId, whObjectId), label);
    }
    
    public static Link constructLinkForWhObject(final String serviceProviderId, final String whObjectId)
    {
        return new Link(constructURIForWhObject(serviceProviderId, whObjectId));
    }
    

    //methods for Waypoint resource
    public static Waypoint createWaypoint(final String serviceProviderId, final String waypointId)
           throws URISyntaxException
    {
        return new Waypoint(constructURIForWaypoint(serviceProviderId, waypointId));
    }
    
    public static URI constructURIForWaypoint(final String serviceProviderId, final String waypointId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("waypointId", waypointId);
        String instanceURI = "serviceProviders/{serviceProviderId}/resources/waypoints/{waypointId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForWaypoint(final String serviceProviderId, final String waypointId , final String label)
    {
        return new Link(constructURIForWaypoint(serviceProviderId, waypointId), label);
    }
    
    public static Link constructLinkForWaypoint(final String serviceProviderId, final String waypointId)
    {
        return new Link(constructURIForWaypoint(serviceProviderId, waypointId));
    }
    

}
