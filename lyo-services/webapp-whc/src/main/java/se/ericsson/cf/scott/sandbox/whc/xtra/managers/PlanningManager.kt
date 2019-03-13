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
import java.util.Arrays
import java.util.concurrent.atomic.AtomicInteger

/**
 * TODO
 *
 * @since   TODO
 */
class PlanningManager(private val planRequestHelper: PlanRequestHelper) {

    companion object {
        private val log: Logger = LoggerFactory.getLogger(PlanningManager::class.java)
    }

    private val domain: Model = RdfHelpers.modelFromTurtleResource(PlanningManager::class.java,
        "pddl/dom-connectivity.ttl")
    private val width = 8
    private val height = 8

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
            .forEach { twinInfo: TwinInfo ->
                val requestModel: Model = buildPlanRequestFor(twinInfo)
                val planModel: Model = planRequestHelper.requestPlan(requestModel)
                sendPlan(twinInfo, planModel)
            }
    }


    private fun buildPlanRequestFor(twinInfo: TwinInfo): Model {
        log.trace("Building planning request")

        val oslcHelper = OslcHelper(WhcConfig.getBaseUri())
        val requestBuilder = PlanRequestBuilder()
        val stateBuilder = ProblemBuilder(oslcHelper, PlanRequestHelper(oslcHelper))
        // TODO Andrew@2019-02-19: set OslcRdfHelper base or use a real URI
        requestBuilder.domainFromModel(domain)
            .withStateBuilder(stateBuilder)

        defineStaticProblemPart(stateBuilder)

        val (x, y) = generateInitCoords()
        stateBuilder.robotsActive(listOf(twinInfo.label))
            .robotAtInit(twinInfo.label, x, y)

        return requestBuilder.build()
    }

    private fun generateInitCoords(): Pair<Int, Int> {
        val counterValue = counter.getAndIncrement()
        val x = counterValue % width
        val y = counterValue / height
        log.debug("Allocating ($x, $y) coordinate for an init position")
        return Pair(x, y)
    }

    private fun defineStaticProblemPart(stateBuilder: ProblemBuilder) {
        stateBuilder.genLabel("TRS-Safety prototype plan request")
            .problemDomain(Link(stateBuilder.oslcHelper.u("scott-warehouse")))
            .warehouseSize(width, height);

        stateBuilder.shelfAt("sh1", 1, 7)
            .beltAt("cb1", 7, 2)
            .robotsInactive(Arrays.asList("ob1", "ob2", "ob3"))
            .robotAtInit("ob1", 5, 6)
            .robotAtInit("ob2", 5, 5)
            .robotAtInit("ob3", 5, 4)

        stateBuilder.boxOnShelfInit("b1", "sh1")
            .boxOnBeltGoal("b1", "cb1")
    }

    private fun sendPlan(twinInfo: TwinInfo, planModel: Model) {
        log.debug(
            "Sending the plan to the Digital Twin '${twinInfo.label}':\n${RdfHelpers.modelToString(
                planModel)}")
    }

    fun triggerSamplePlanning(
        twinsRepository: TwinRepository) {
        log.info("Performing sample planning")
        val requestModel = buildSamplePlanRequest(twinsRepository.twins)

        val planModel = planRequestHelper.requestPlan(requestModel)
        val responseStr = RdfHelpers.modelToString(planModel)
        log.debug("Received Plan response: $responseStr")

        tryRegisterPlan(planModel)
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


    private fun buildSamplePlanRequest(twins: Set<TwinInfo>): Model {
        log.trace("Building planning request")
        val helper = OslcHelper(WhcConfig.getBaseUri())
        var requestBuilder = PlanRequestBuilder()
        val stateBuilder = ProblemBuilder(helper, PlanRequestHelper(helper))
        // TODO Andrew@2019-02-19: set OslcRdfHelper base or use a real URI
        requestBuilder = requestBuilder.domainFromModel(domain)
            .withStateBuilder(stateBuilder)

        // TODO Andrew@2019-03-12: use defineStaticProblemPart()
        val _width = 25
        val _height = 25
        stateBuilder//.problemUri(OslcHelpers.u("scott-warehouse-problem"))
            .genLabel("TRS-Safety prototype plan request")
            .problemDomain(Link(helper.u("scott-warehouse")))
            .warehouseSize(_width, _height)
            .robotsActive(twins.map { it.label })
            .robotsInactive(Arrays.asList("ob1", "ob2", "ob3"))
            .shelfAt("sh1", 0, 10)
            .beltAt("cb1", 0, 20)
            .boxOnShelfInit("b1", "sh1")
            .robotAtInit("ob1", 12, 12)
            .robotAtInit("ob2", 12, 11)
            .robotAtInit("ob3", 12, 10)
            .boxOnBeltGoal("b1", "cb1")
        // TODO Andrew@2019-02-19: do we need negation too?
        //.boxNotOnShelfGoal("b1", "sh1");

        if(twins.size % _width > 11) {
            log.warn("Too many robots; the robots inits may overlap with the static obstacles")
        }

        for ((i, it) in twins.withIndex()) {
            // TODO WARN Andrew@2019-03-12: not really sortable
            stateBuilder.robotAtInit(it.label, i % _width, i / _height + 1)
        }

        return requestBuilder.build()

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
        // TODO Andrew@2019-02-20: check the with Jad what hack from Yash did he mean
        // TODO Andrew@2018-02-23: here the plan needs to be put through lyo store update
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
