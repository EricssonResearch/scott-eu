package se.ericsson.cf.scott.sandbox.whc.xtra.managers

import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.lib.InstanceWithResources
import eu.scott.warehouse.lib.OslcHelper
import eu.scott.warehouse.lib.RdfHelpers
import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager
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
    private val width = 4
    private val height = 4

    private var counter: AtomicInteger = AtomicInteger(1)

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
        val x = counterValue % width
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
                throw IllegalStateException("Plan '$key' has already been registered")
            }
            log.info("Registering plan '$key'")

            planRepository.registerPlan(plan)

            // TODO Andrew@2019-04-22: have some event listener here
            WarehouseControllerManager.getChangeHistories()
                .addResource(plan.instance)
            twinClient.requestPlanExecution(twinInfo, plan)
        } catch (e: LyoJenaModelException) {
            log.error("Cannot unmarshal the Plan")
        }
    }


    @Throws(LyoJenaModelException::class)
    private fun unmarshalPlan(planModel: Model): InstanceWithResources<Plan> {
        val planResource = extractPlan(planModel)
        log.debug("Received plan: {}", planResource)
        val planResources = planRequestHelper.getPlanResources(planModel, planResource)

        val plan: InstanceWithResources<Plan> = InstanceWithResources(planResource, planResources)
        return plan
    }

    @Throws(LyoJenaModelException::class)
    private fun extractPlan(planModel: Model): Plan {
        val plan = JenaModelHelper.unmarshalSingle(planModel, Plan::class.java)
        plan.about = planRepository.newURI()
        return plan
    }
}
