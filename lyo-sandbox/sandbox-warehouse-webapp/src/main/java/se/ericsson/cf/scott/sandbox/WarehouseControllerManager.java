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
import se.ericsson.cf.scott.sandbox.resources.Mission;
import se.ericsson.cf.scott.sandbox.resources.Place;
import se.ericsson.cf.scott.sandbox.resources.Predicate;
import se.ericsson.cf.scott.sandbox.resources.ProblemState;
import se.ericsson.cf.scott.sandbox.resources.Robot;
import se.ericsson.cf.scott.sandbox.resources.Waypoint;
import se.ericsson.cf.scott.sandbox.resources.WhObject;


// Start of user code imports
import java.io.IOException;
import javax.servlet.ServletContext;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import com.google.common.collect.ImmutableList;
import java.util.Optional;
import java.net.URI;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreFactory;
import com.hp.hpl.jena.sparql.ARQException;
// End of user code

// Start of user code pre_class_code
// End of user code

public class WarehouseControllerManager {

    // Start of user code class_attributes
    private final static Logger log = LoggerFactory.getLogger(WarehouseControllerManager.class);
    public final static String spId = "dummy";
    private final static ResourceFactory factory = new ResourceFactory(OSLC4JUtils.getServletURI(),
            spId);
    private static Store store;
    // End of user code
    
    
    // Start of user code class_methods
    private static List<Waypoint> getWaypoints() {
        final List<Waypoint> resources;

        resources = ImmutableList.of(factory.buildWaypoint("wp1",
            ImmutableList.of(factory.wayPointLink("wp2"), factory.wayPointLink("wp2"))),
            factory.buildWaypoint("wp2",
                ImmutableList.of(factory.wayPointLink("wp3"), factory.wayPointLink("wp4"))),
            factory.buildWaypoint("wp3",
                ImmutableList.of(factory.wayPointLink("wp1"), factory.wayPointLink("wp4"))),
            factory.buildWaypoint("wp4", ImmutableList.of()));
        return resources;
    }

    private static List<Place> getPlaces() {
        final List<Place> resources;
        resources = ImmutableList.of(
            factory.buildPlace("cbIn1", "ConveyorBelt", factory.wayPointLink("wp1"), 3),
            factory.buildPlace("cbIn2", "ConveyorBelt", factory.wayPointLink("wp3"), 3),
            factory.buildPlace("shelf1", "Shelf", factory.wayPointLink("wp2"), 5),
            factory.buildPlace("chargingStation1", "ChargingStation",
                factory.wayPointLink("wp4")));
        return resources;
    }

    private static List<WhObject> getWhObjects() {
        final List<WhObject> resources;
        resources = ImmutableList.of(
            factory.buildObject("o1", "Simple", factory.placeLink("shelf1")),
            factory.buildObject("o2", "Simple", factory.placeLink("cbIn1")));
        return resources;
    }

//    private static List<Robot> getRobots() {
//        final List<Robot> resources;
//        resources = ImmutableList.of(
//            factory.buildRobot("robot1", factory.wayPointLink("wp2"), 7, 1, 10, 6, 3, false),
//            factory.buildRobot("robot2", factory.wayPointLink("wp3"), 7, 1, 10, 6, 3, false));
//        return resources;
//    }

    private static <T extends AbstractResource> T findResource(final List<T> resources,
        final String uri) {
        log.info("Looking up resource {}", uri);
        final Optional<T> opt = resources.stream().filter(
            resource -> resource.getAbout().equals(URI.create(uri))).findFirst();
        return opt.get();
    }

    /**
     * https://stackoverflow.com/a/2222268/464590
     */
    public static String getFullURL(HttpServletRequest request) {
        StringBuffer requestURL = request.getRequestURL();
        String queryString = request.getQueryString();

        if (queryString == null) {
            return requestURL.toString();
        } else {
            return requestURL.append('?').append(queryString).toString();
        }
    }

    // End of user code

    public static void contextInitializeServletListener(final ServletContextEvent servletContextEvent)
    {
        
        // Start of user code contextInitializeServletListener
        final ServletContext context = servletContextEvent.getServletContext();
        final String queryUri = context.getInitParameter(
                "se.ericsson.cf.scott.sandbox.store.query");
        final String updateUri = context.getInitParameter(
                "se.ericsson.cf.scott.sandbox.store.update");
        try {
            store = StoreFactory.sparql(queryUri, updateUri);
            // TODO Andrew@2017-07-18: Remember to deactivate when switch to more persistent arch
            store.removeAll();
        } catch (IOException |ARQException e) {
            log.error("SPARQL Store failed to initialise with the URIs query={};update={}",
                    queryUri, updateUri, e);
        }
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
        final ServiceProviderInfo serviceProviderInfo = new ServiceProviderInfo();
        serviceProviderInfo.serviceProviderId = spId;
        serviceProviderInfo.name = "Handcrafted warehouse";
        serviceProviderInfos = new ServiceProviderInfo[]{serviceProviderInfo};
        // End of user code
        return serviceProviderInfos;
    }

    public static List<WhObject> queryWhObjects(HttpServletRequest httpServletRequest, final String serviceProviderId, String where, int page, int limit)
    {
        List<WhObject> resources = null;
        
        // Start of user code queryWhObjects
        resources = getWhObjects();
        // End of user code
        return resources;
    }
    public static List<Place> queryPlaces(HttpServletRequest httpServletRequest, final String serviceProviderId, String where, int page, int limit)
    {
        List<Place> resources = null;
        
        // Start of user code queryPlaces
        resources = getPlaces();
        // End of user code
        return resources;
    }
    public static List<Waypoint> queryWaypoints(HttpServletRequest httpServletRequest, final String serviceProviderId, String where, int page, int limit)
    {
        List<Waypoint> resources = null;
        
        // Start of user code queryWaypoints
        resources = getWaypoints();
        // End of user code
        return resources;
    }


    public static Mission getMission(HttpServletRequest httpServletRequest, final String serviceProviderId, final String missionId)
    {
        Mission aResource = null;
        
        // Start of user code getMission
        // TODO Implement code to return a resource
        // End of user code
        return aResource;
    }
    public static Place getPlace(HttpServletRequest httpServletRequest, final String serviceProviderId, final String placeId)
    {
        Place aResource = null;
        
        // Start of user code getPlace
        aResource = findResource(getPlaces(), getFullURL(httpServletRequest));
        // End of user code
        return aResource;
    }
    public static Waypoint getWaypoint(HttpServletRequest httpServletRequest, final String serviceProviderId, final String waypointId)
    {
        Waypoint aResource = null;
        
        // Start of user code getWaypoint
        aResource = findResource(getWaypoints(), getFullURL(httpServletRequest));
        // End of user code
        return aResource;
    }
    public static WhObject getWhObject(HttpServletRequest httpServletRequest, final String serviceProviderId, final String whObjectId)
    {
        WhObject aResource = null;
        
        // Start of user code getWhObject
        aResource = findResource(getWhObjects(), getFullURL(httpServletRequest));
        // End of user code
        return aResource;
    }




    public static String getETagFromMission(final Mission aResource)
    {
        String eTag = null;
        // Start of user code getETagFromMission
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }
    public static String getETagFromPlace(final Place aResource)
    {
        String eTag = null;
        // Start of user code getETagFromPlace
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }
    public static String getETagFromWhObject(final WhObject aResource)
    {
        String eTag = null;
        // Start of user code getETagFromWhObject
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }
    public static String getETagFromWaypoint(final Waypoint aResource)
    {
        String eTag = null;
        // Start of user code getETagFromWaypoint
        // TODO Implement code to return an ETag for a particular resource
        // End of user code
        return eTag;
    }

}
