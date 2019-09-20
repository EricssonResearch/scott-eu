/*
 * Copyright (c) 2019 Ericsson Research and others
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package se.ericsson.cf.scott.sandbox.whc.xtra.clients

import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.domains.twins.PlanExecutionRequest
import eu.scott.warehouse.lib.InstanceWithResources
import eu.scott.warehouse.lib.RdfHelpers.randomUuidUrn
import eu.scott.warehouse.lib.link
import eu.scott.warehouse.lib.toTurtle
import org.eclipse.lyo.oslc4j.client.OslcClient
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinInfo
import javax.ws.rs.core.Response


class TwinClient() {
    companion object {
        val log: Logger = LoggerFactory.getLogger(TwinClient::class.java)
    }

    private val client: OslcClient = OslcClient()

    fun requestPlanExecution(twinInfo: TwinInfo, plan: InstanceWithResources<Plan>) {
        val execRequest = PlanExecutionRequest(randomUuidUrn())
        execRequest.plan = plan.instance.link

        log.info("Dispatching a new Plan to {}", twinInfo.label)
        log.trace("POSTing PlanExecutionRequest to ${twinInfo.cfUri}:\n${execRequest.toTurtle}")

        val response = client.createResource(twinInfo.cfUri, execRequest,
            OslcMediaType.APPLICATION_RDF_XML)
        if (response.statusInfo.family == Response.Status.Family.SUCCESSFUL) {
            log.debug("Plan '${plan.instance.about}' was successfully POSTed")
        } else {
            log.warn("There was a problem with the plan '${plan.instance.about}'" +
                ": ${response.statusInfo} (${twinInfo.cfUri})")
        }
    }
}
