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

package se.ericsson.cf.scott.sandbox.whc.xtra.managers

import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.lib.InstanceWithResources
import eu.scott.warehouse.lib.OslcHelper
import eu.scott.warehouse.lib.RdfHelpers
import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.oslc4j.core.exception.LyoModelException
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig
import se.ericsson.cf.scott.sandbox.whc.xtra.clients.TwinClient
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestBuilder
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelper
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.ProblemBuilder
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.PlanRepository
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinInfo
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinRepository
import java.util.concurrent.atomic.AtomicInteger


class PlanningManager(private val planRequestHelper: PlanRequestHelper,
                      private val twinClient: TwinClient,
                      private val planRepository: PlanRepository) {

    companion object {
        private val log: Logger = LoggerFactory.getLogger(PlanningManager::class.java)
    }

    private val domain: Model = RdfHelpers.modelFromTurtleResource(PlanningManager::class.java,
        "pddl/dom-connectivity.ttl")
    private val width = 8
    private val height = 8

    private var counter: AtomicInteger = AtomicInteger(1)

    fun planForEachTwin(twinsRepository: TwinRepository) {
        twinsRepository.twins.parallelStream()
            .forEach(::planForTwin)
    }

    private fun planForTwin(twinInfo: TwinInfo) {
        try {
            val requestModel: Model = buildPlanRequestFor(twinInfo)
            log.info("Requesting a new plan for {}", twinInfo.label)
            val planModel: Model = planRequestHelper.requestPlan(requestModel)
            tryRegisterPlan(twinInfo, planModel)
        } catch (e: Exception) {
            log.error("Unexpected exception while creating a plan for Twin '${twinInfo.id}': ", e)
        }
    }


    private fun buildPlanRequestFor(twinInfo: TwinInfo): Model {
        log.trace("Building planning request")

        val oslcHelper = OslcHelper(WhcConfig.getBaseUri())
        val requestBuilder = PlanRequestBuilder()
        val stateBuilder = ProblemBuilder(oslcHelper, PlanRequestHelper(oslcHelper))
        requestBuilder.domainFromModel(domain)
            .withStateBuilder(stateBuilder)

        defineStaticProblemPart(stateBuilder)

        val (x, y) = generateInitCoords()
        stateBuilder.robotsActive(listOf(twinInfo.id))
            .robotAtInit(twinInfo.id, x, y)

        return requestBuilder.build()
    }

    private fun generateInitCoords(): Pair<Int, Int> {
        val counterValue = counter.getAndIncrement()
        val x = counterValue % width + 1
        val y = counterValue / height + 1
        log.debug("Allocating ($y, $x) coordinate for an init position")
        return Pair(x, y)
    }

    private fun defineStaticProblemPart(stateBuilder: ProblemBuilder) {
        stateBuilder.genLabel("TRS-Safety prototype plan request")
            .problemDomain(Link(stateBuilder.oslcHelper.u("scott-warehouse")))
            .warehouseSize(width, height)

        stateBuilder.shelfAt("sh1", 1, 3)
            .beltAt("cb1", 3, 3)
//            .robotsInactive(Arrays.asList("ob1", "ob2", "ob3"))
//            .robotAtInit("ob1", 5, 6)
//            .robotAtInit("ob2", 5, 5)
//            .robotAtInit("ob3", 5, 4)

        stateBuilder.boxOnShelfInit("b1", "sh1")
            .boxOnBeltGoal("b1", "cb1")
    }

    private fun tryRegisterPlan(twinInfo: TwinInfo, planModel: Model) {
        try {
            val plan = unmarshalPlan(planModel)
            val key = plan.instance.about
            if (planRepository.existsPlan(key)) {
                log.warn("Overwriting an existing plan '{}'", key)
                planRepository.removePlan(key)
//                throw IllegalStateException("Plan '$key' has already been registered")
            }
            log.debug("Registering plan '$key'")

            planRepository.registerPlan(plan)

            // FIXME Andrew@2019-05-02: call the event listener

            twinClient.requestPlanExecution(twinInfo, plan)
        } catch (e: LyoModelException) {
            log.error("Cannot unmarshal the Plan")
        }
    }


    private fun unmarshalPlan(planModel: Model): InstanceWithResources<Plan> {
        val planResource = extractPlan(planModel)
        log.debug("Received plan: {}", planResource)
        val planResources = planRequestHelper.getPlanResources(planModel, planResource)

        return InstanceWithResources(planResource, planResources)
    }

    private fun extractPlan(planModel: Model): Plan {
        val plan = JenaModelHelper.unmarshalSingle(planModel, Plan::class.java)
        plan.about = planRepository.newURI()
        return plan
    }
}
