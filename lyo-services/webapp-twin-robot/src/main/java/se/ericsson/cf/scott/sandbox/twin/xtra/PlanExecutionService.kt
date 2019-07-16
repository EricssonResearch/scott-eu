/*
 * Copyright (c) 2019  Ericsson Research and others
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

package se.ericsson.cf.scott.sandbox.twin.xtra

import eu.scott.warehouse.domains.pddl.Action
import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.domains.pddl.Step
import eu.scott.warehouse.domains.scott.DropBelt
import eu.scott.warehouse.domains.scott.MoveToWp
import eu.scott.warehouse.domains.scott.PickShelf
import eu.scott.warehouse.domains.twins.PlanExecutionRequest
import eu.scott.warehouse.lib.OslcHelpers
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.eclipse.lyo.oslc4j.client.OslcClient
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.twin.xtra.plans.ExecutionInfo
import se.ericsson.cf.scott.sandbox.twin.xtra.plans.IActionStatusHandler
import java.io.BufferedInputStream
import java.util.concurrent.ScheduledExecutorService
import javax.ws.rs.core.Response
import java.io.InputStream
import java.net.ConnectException
import java.util.Date
import java.util.Optional


class PlanExecutionService(private val executor: ScheduledExecutorService,
                           private val client: OslcClient,
                           private val statusHandler: IActionStatusHandler) {
    companion object {
        val log: Logger = LoggerFactory.getLogger(PlanExecutionService::class.java)
    }

    fun fulfillRequest(request: PlanExecutionRequest,
                       twinKind: String, twinId: String) {
        if (request.plan == null || request.plan.value == null) {
            throw IllegalArgumentException("Plan link is missing from the $request")
        }
        val planURI = request.plan.value
        if (!planURI.isAbsolute) {
            throw IllegalArgumentException("Plan link URI is malformed: $planURI")
        }

        val uriString = planURI.toASCIIString()
        val planModelResult = requestModel(uriString)

        if (planModelResult.isPresent) {
            val plan = planModelResult.get()
            val planResource = JenaModelHelper.unmarshalSingle(plan, Plan::class.java)
            log.info("New Plan received: ${planResource.about} (cost ${planResource.cost})")
            val reportBuilder = StringBuilder()
            reportBuilder.appendln("Steps in the Plan ${planResource.about}:")
            val actionSteps = planResource.step.sortedBy { it.order }
            actionSteps.forEach { step: Step ->
                reportBuilder.append("\tStep ${step.order}: ")
                val action: IExtendedResource = OslcHelpers.navTry(plan, step.action,
                    Action::class.java, DropBelt::class.java, MoveToWp::class.java,
                    PickShelf::class.java)
                val executionBegin = Date()
                val executionInfo = ExecutionInfo(twinKind, twinId, executionBegin)
                when (action) {
                    is DropBelt  -> execDropBelt(action, reportBuilder, executionInfo)
                    is MoveToWp  -> execMoveToWp(action, reportBuilder, executionInfo)
                    is PickShelf -> execPickShelf(action, reportBuilder, executionInfo)
                }
            }
            log.debug(reportBuilder.toString())
        }
    }

    private fun execDropBelt(action: DropBelt, reportBuilder: StringBuilder, executionInfo: ExecutionInfo) {
        reportBuilder.appendln("action 'drop-belt'")
        statusHandler.actionCompleted(action, executionInfo)
    }

    private fun execMoveToWp(action: MoveToWp,
                             reportBuilder: StringBuilder,
                             executionInfo: ExecutionInfo) {
        reportBuilder.appendln("action 'move-to-wp'")
        statusHandler.actionCompleted(action, executionInfo)
    }

    private fun execPickShelf(action: PickShelf,
                              reportBuilder: StringBuilder,
                              executionInfo: ExecutionInfo) {
        reportBuilder.appendln("action 'pick-shelf'")
        statusHandler.actionCompleted(action, executionInfo)
    }

    private fun requestModel(uriString: String): Optional<Model> {
        return try {
            val response: Response = client.getResource(uriString,
                OslcMediaType.APPLICATION_RDF_XML)
            val inputStream = BufferedInputStream(response.readEntity(InputStream::class.java))
            val responseModel: Model = ModelFactory.createDefaultModel()
            RDFDataMgr.read(responseModel, inputStream, Lang.RDFXML)
            Optional.of(responseModel)
        } catch (e: ConnectException) {
            log.warn("Connection to $uriString failed")
            Optional.empty()
        }
    }

}
