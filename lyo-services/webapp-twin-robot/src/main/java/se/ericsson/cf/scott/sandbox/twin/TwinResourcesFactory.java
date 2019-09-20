// Start of user code Copyright
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
// End of user code

package se.ericsson.cf.scott.sandbox.twin;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import javax.ws.rs.core.UriBuilder;

import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import eu.scott.warehouse.domains.pddl.Action;
import eu.scott.warehouse.domains.scott.ActionExecutionReport;
import eu.scott.warehouse.domains.twins.DeviceRegistrationMessage;
import eu.scott.warehouse.domains.scott.ExecutableAction;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.twins.PlanExecutionRequest;
import eu.scott.warehouse.domains.pddl.Step;

// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code

public class TwinResourcesFactory {

    // Start of user code class_attributes
    // End of user code
    
    // Start of user code class_methods
    // End of user code

    //methods for ActionExecutionReport resource
    public static ActionExecutionReport createActionExecutionReport(final String twinKind, final String twinId, final String actionExecutionReportId)
           throws URISyntaxException
    {
        return new ActionExecutionReport(constructURIForActionExecutionReport(twinKind, twinId, actionExecutionReportId));
    }
    
    public static URI constructURIForActionExecutionReport(final String twinKind, final String twinId, final String actionExecutionReportId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("twinKind", twinKind);
        pathParameters.put("twinId", twinId);
        pathParameters.put("actionExecutionReportId", actionExecutionReportId);
        String instanceURI = "twins/{twinKind}/{twinId}/service2/actionExecutionReports/{actionExecutionReportId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForActionExecutionReport(final String twinKind, final String twinId, final String actionExecutionReportId , final String label)
    {
        return new Link(constructURIForActionExecutionReport(twinKind, twinId, actionExecutionReportId), label);
    }
    
    public static Link constructLinkForActionExecutionReport(final String twinKind, final String twinId, final String actionExecutionReportId)
    {
        return new Link(constructURIForActionExecutionReport(twinKind, twinId, actionExecutionReportId));
    }
    

    //methods for DeviceRegistrationMessage resource
    

    //methods for PlanExecutionRequest resource
    

}
