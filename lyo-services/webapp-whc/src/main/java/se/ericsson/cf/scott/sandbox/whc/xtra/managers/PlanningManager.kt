package se.ericsson.cf.scott.sandbox.whc.xtra.managers

import com.google.common.collect.ImmutableMap
import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.lib.OslcHelper
import eu.scott.warehouse.lib.RdfHelpers
import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerResourcesFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestBuilder
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelper
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.ProblemBuilder
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinInfo
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinRepository
import java.net.URI
import java.util.concurrent.atomic.AtomicInteger


class PlanningManager(private val planRequestHelper: PlanRequestHelper) {

    companion object {
        private val log: Logger = LoggerFactory.getLogger(PlanningManager::class.java)
    }

    private val domain: Model = RdfHelpers.modelFromTurtleResource(PlanningManager::class.java,
        "pddl/dom-connectivity.ttl")
    private val width = 4
    private val height = 4

    private var counter: AtomicInteger = AtomicInteger(1)

    @Suppress("MemberVisibilityCanBePrivate")
    public val plans: MutableMap<URI, Array<Any>> = HashMap()
        get() = ImmutableMap.copyOf(field)


    @Deprecated("use a builder-based approach instead")
    fun triggerLegacySamplePlanning() {
        val problemModel: Model = planRequestHelper.getProblem("sample-problem-request.ttl")
        val planModel: Model = planRequestHelper.requestPlan(problemModel)
        tryRegisterPlan(planModel)
    }

    fun planForEachTwin(twinsRepository: TwinRepository) {
        log.info("Creating one plan per Digital Twin")

        twinsRepository.twins.parallelStream()
            .forEach { twinInfo ->
                planForTwin(twinInfo)
            }
    }

    private fun planForTwin(twinInfo: TwinInfo) {
        try {
            val requestModel: Model = buildPlanRequestFor(twinInfo)
            val planModel: Model = planRequestHelper.requestPlan(requestModel)
            sendPlan(twinInfo, planModel)
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
        val x = counterValue % width
        val y = counterValue / height + 1
        log.debug("Allocating ($y, $x) coordinate for an init position")
        return Pair(x, y)
    }

    private fun defineStaticProblemPart(stateBuilder: ProblemBuilder) {
        stateBuilder.genLabel("TRS-Safety prototype plan request")
            .problemDomain(Link(stateBuilder.oslcHelper.u("scott-warehouse")))
            .warehouseSize(width, height);

        stateBuilder.shelfAt("sh1", 1, 3)
            .beltAt("cb1", 3, 3)
//            .robotsInactive(Arrays.asList("ob1", "ob2", "ob3"))
//            .robotAtInit("ob1", 5, 6)
//            .robotAtInit("ob2", 5, 5)
//            .robotAtInit("ob3", 5, 4)

        stateBuilder.boxOnShelfInit("b1", "sh1")
            .boxOnBeltGoal("b1", "cb1")
    }

    private fun sendPlan(twinInfo: TwinInfo, planModel: Model) {
        log.debug(
            "Sending the plan to the Digital Twin '${twinInfo.label}':\n${RdfHelpers.modelToString(
                planModel)}")
    }

    private fun tryRegisterPlan(planModel: Model) {
        try {
            log.trace("Registering a new plan")
            val (plan, resources) = unmarshalPlan(planModel)

            registerNewPlan(plan, resources)
        } catch (e: LyoJenaModelException) {
            log.error("Cannot unmarshal the Plan")
        }
    }


    @Throws(LyoJenaModelException::class)
    private fun unmarshalPlan(planModel: Model): Pair<Plan, Array<Any>> {
        val plan = extractPlan(planModel)
        log.debug("Received plan: {}", plan)
        val planResources = planRequestHelper.getPlanResources(planModel, plan)

        return Pair(plan, planResources)
    }

    private fun registerNewPlan(plan: Plan, planResources: Array<Any>) {
        val key = plan.about
        if (plans.containsKey(key)) {
            throw IllegalStateException("The plan with the same URI has already been registered")
        }
        log.info("Registering plan $key")
        plans[key] = planResources
        WarehouseControllerManager.getChangeHistories()
            .addResource(plan)
    }

    @Throws(LyoJenaModelException::class)
    private fun extractPlan(planModel: Model): Plan {
        val plan = JenaModelHelper.unmarshalSingle(planModel, Plan::class.java)
        // TODO Andrew@2018-02-26: extract method
        val planId = (plans.size + 1).toString()
        plan.about = WarehouseControllerResourcesFactory.constructURIForPlan(
            WhcConfig.DEFAULT_SP_ID, planId)
        return plan
    }
}
