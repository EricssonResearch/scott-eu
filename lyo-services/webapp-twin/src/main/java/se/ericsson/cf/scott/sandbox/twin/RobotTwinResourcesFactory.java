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
import eu.scott.warehouse.domains.mission.AgentRequest;
import eu.scott.warehouse.domains.mission.Goal;
import eu.scott.warehouse.domains.mission.Mission;
import eu.scott.warehouse.domains.pddl.Plan;
import eu.scott.warehouse.domains.pddl.PlanExecutionResult;
import eu.scott.warehouse.domains.pddl.Step;

// Start of user code imports
// End of user code

// Start of user code pre_class_code
// End of user code

public class RobotTwinResourcesFactory {

    // Start of user code class_attributes
    // End of user code
    
    // Start of user code class_methods
    // End of user code

    //methods for AgentRequest resource
    public static AgentRequest createAgentRequest(final String beltId, final String agentRequestId)
           throws URISyntaxException
    {
        return new AgentRequest(constructURIForAgentRequest(beltId, agentRequestId));
    }
    
    public static URI constructURIForAgentRequest(final String beltId, final String agentRequestId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("beltId", beltId);
        pathParameters.put("agentRequestId", agentRequestId);
        String instanceURI = "belts/{beltId}/resources/agentRequests/{agentRequestId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForAgentRequest(final String beltId, final String agentRequestId , final String label)
    {
        return new Link(constructURIForAgentRequest(beltId, agentRequestId), label);
    }
    
    public static Link constructLinkForAgentRequest(final String beltId, final String agentRequestId)
    {
        return new Link(constructURIForAgentRequest(beltId, agentRequestId));
    }
    

    //methods for Mission resource
    public static Mission createMission(final String beltId, final String missionId)
           throws URISyntaxException
    {
        return new Mission(constructURIForMission(beltId, missionId));
    }
    
    public static URI constructURIForMission(final String beltId, final String missionId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("beltId", beltId);
        pathParameters.put("missionId", missionId);
        String instanceURI = "belts/{beltId}/resources/missions/{missionId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForMission(final String beltId, final String missionId , final String label)
    {
        return new Link(constructURIForMission(beltId, missionId), label);
    }
    
    public static Link constructLinkForMission(final String beltId, final String missionId)
    {
        return new Link(constructURIForMission(beltId, missionId));
    }
    

    //methods for PlanExecutionResult resource
    public static PlanExecutionResult createPlanExecutionResult(final String serviceProviderId, final String planExecutionResultId)
           throws URISyntaxException
    {
        return new PlanExecutionResult(constructURIForPlanExecutionResult(serviceProviderId, planExecutionResultId));
    }
    
    public static URI constructURIForPlanExecutionResult(final String serviceProviderId, final String planExecutionResultId)
    {
        String basePath = OSLC4JUtils.getServletURI();
        Map<String, Object> pathParameters = new HashMap<String, Object>();
        pathParameters.put("serviceProviderId", serviceProviderId);
        pathParameters.put("planExecutionResultId", planExecutionResultId);
        String instanceURI = "robots/{serviceProviderId}/planExecutionResults/{planExecutionResultId}";
    
        final UriBuilder builder = UriBuilder.fromUri(basePath);
        return builder.path(instanceURI).buildFromMap(pathParameters);
    }
    
    public static Link constructLinkForPlanExecutionResult(final String serviceProviderId, final String planExecutionResultId , final String label)
    {
        return new Link(constructURIForPlanExecutionResult(serviceProviderId, planExecutionResultId), label);
    }
    
    public static Link constructLinkForPlanExecutionResult(final String serviceProviderId, final String planExecutionResultId)
    {
        return new Link(constructURIForPlanExecutionResult(serviceProviderId, planExecutionResultId));
    }
    

}
