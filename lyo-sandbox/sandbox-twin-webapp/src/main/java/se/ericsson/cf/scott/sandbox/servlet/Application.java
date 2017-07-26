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
 *     Michael Fiedler     - initial API and implementation for Bugzilla adapter
 *     Jad El-khoury       - initial implementation of code generator (https://bugs.eclipse.org/bugs/show_bug.cgi?id=422448)
 *     Matthieu Helleboid  - Support for multiple Service Providers.
 *     Anass Radouani      - Support for multiple Service Providers.
 *
 * This file is generated by org.eclipse.lyo.oslc4j.codegenerator
 *******************************************************************************/

package se.ericsson.cf.scott.sandbox.servlet;

import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;


import org.eclipse.lyo.oslc4j.application.OslcWinkApplication;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.AllowedValues;
import org.eclipse.lyo.oslc4j.core.model.Compact;
import org.eclipse.lyo.oslc4j.core.model.CreationFactory;
import org.eclipse.lyo.oslc4j.core.model.Dialog;
import org.eclipse.lyo.oslc4j.core.model.Error;
import org.eclipse.lyo.oslc4j.core.model.ExtendedError;
import org.eclipse.lyo.oslc4j.core.model.OAuthConfiguration;
import org.eclipse.lyo.oslc4j.core.model.OslcConstants;
import org.eclipse.lyo.oslc4j.core.model.PrefixDefinition;
import org.eclipse.lyo.oslc4j.core.model.Preview;
import org.eclipse.lyo.oslc4j.core.model.Property;
import org.eclipse.lyo.oslc4j.core.model.Publisher;
import org.eclipse.lyo.oslc4j.core.model.QueryCapability;
import org.eclipse.lyo.oslc4j.core.model.ResourceShape;
import org.eclipse.lyo.oslc4j.core.model.Service;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.core.model.ServiceProviderCatalog;
import org.eclipse.lyo.oslc4j.provider.jena.JenaProvidersRegistry;
import org.eclipse.lyo.oslc4j.provider.json4j.Json4JProvidersRegistry;

import se.ericsson.cf.scott.sandbox.services.ServiceProviderCatalogService;
import se.ericsson.cf.scott.sandbox.services.ServiceProviderService;
import se.ericsson.cf.scott.sandbox.services.ResourceShapeService;

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
import se.ericsson.cf.scott.sandbox.resources.DctermsConstants;
import scott.lyo.domain.planning.PpConstants;
import se.ericsson.cf.scott.sandbox.resources.RdfsConstants;
import scott.lyo.domain.warehouse.WhConstants;
import se.ericsson.cf.scott.sandbox.services.ServiceProviderService1;

// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code

public class Application extends OslcWinkApplication {

    private static final Set<Class<?>>         RESOURCE_CLASSES                          = new HashSet<Class<?>>();
    private static final Map<String, Class<?>> RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP = new HashMap<String, Class<?>>();

    // Start of user code class_attributes
    // End of user code

    // Start of user code class_methods
    // End of user code

    static
    {
        try
        {
            RESOURCE_CLASSES.addAll(JenaProvidersRegistry.getProviders());
            RESOURCE_CLASSES.addAll(Json4JProvidersRegistry.getProviders());
            RESOURCE_CLASSES.add(ServiceProviderService1.class);
            RESOURCE_CLASSES.add(Action.class);
            RESOURCE_CLASSES.add(ActionType.class);
            RESOURCE_CLASSES.add(Mission.class);
            RESOURCE_CLASSES.add(Place.class);
            RESOURCE_CLASSES.add(Plan.class);
            RESOURCE_CLASSES.add(Predicate.class);
            RESOURCE_CLASSES.add(ProblemState.class);
            RESOURCE_CLASSES.add(Robot.class);
            RESOURCE_CLASSES.add(Variable.class);
            RESOURCE_CLASSES.add(VariableInstance.class);
            RESOURCE_CLASSES.add(Waypoint.class);
            RESOURCE_CLASSES.add(WhObject.class);
            RESOURCE_CLASSES.add(Class.forName("org.eclipse.lyo.server.oauth.webapp.services.ConsumersService"));
            RESOURCE_CLASSES.add(Class.forName("org.eclipse.lyo.server.oauth.webapp.services.OAuthService"));

            // Catalog resources
            RESOURCE_CLASSES.add(ServiceProviderCatalogService.class);
            RESOURCE_CLASSES.add(ServiceProviderService.class);
            RESOURCE_CLASSES.add(ResourceShapeService.class);

            // Start of user code Custom Resource Classes
            // End of user code

            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_ALLOWED_VALUES,           AllowedValues.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_COMPACT,                  Compact.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_CREATION_FACTORY,         CreationFactory.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_DIALOG,                   Dialog.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_ERROR,                    Error.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_EXTENDED_ERROR,           ExtendedError.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_OAUTH_CONFIGURATION,      OAuthConfiguration.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_PREFIX_DEFINITION,        PrefixDefinition.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_PREVIEW,                  Preview.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_PROPERTY,                 Property.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_PUBLISHER,                Publisher.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_QUERY_CAPABILITY,         QueryCapability.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_RESOURCE_SHAPE,           ResourceShape.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_SERVICE,                  Service.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_SERVICE_PROVIDER,         ServiceProvider.class);
            RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(OslcConstants.PATH_SERVICE_PROVIDER_CATALOG, ServiceProviderCatalog.class);
        } catch (ClassNotFoundException e)
        {
            e.printStackTrace();
            System.err.println("Application failed to initialize");
        }

//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_ACTION, Action.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_ACTIONTYPE, ActionType.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_MISSION, Mission.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(WhConstants.PATH_PLACE, Place.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_PLAN, Plan.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_PREDICATE, Predicate.class);
        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_PROBLEMSTATE, ProblemState.class);
        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(WhConstants.PATH_ROBOT, Robot.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_VARIABLE, Variable.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(PpConstants.PATH_VARIABLEINSTANCE, VariableInstance.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(WhConstants.PATH_WAYPOINT, Waypoint.class);
//        RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP.put(WhConstants.PATH_WHOBJECT, WhObject.class);
    }

    public Application()
           throws OslcCoreApplicationException,
                  URISyntaxException
    {
        super(RESOURCE_CLASSES,
              OslcConstants.PATH_RESOURCE_SHAPES,
              RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP);
    }

    public static Map<String, Class<?>> getResourceShapePathToResourceClassMap() {
        return RESOURCE_SHAPE_PATH_TO_RESOURCE_CLASS_MAP;
    }
}
