package se.ericsson.cf.scott.sandbox.whc.xtra.managers

import com.google.common.collect.ImmutableMap
import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.lib.OslcRdfHelper
import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerResourcesFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestBuilder
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelper
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelperJava
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.ProblemBuilder
import java.net.URI
import java.util.Arrays

/**
 * TODO
 *
 * @since   TODO
 */
object PlanningManager {

    val log: Logger = LoggerFactory.getLogger(PlanningManager::class.java)

    @Suppress("MemberVisibilityCanBePrivate")
    public val plans: MutableMap<URI, Array<Any>> = HashMap()
        get() = ImmutableMap.copyOf(field)


    @Deprecated("use a builder-based approach instead")
    fun triggerLegacySamplePlanning() {
        val problemModel = PlanRequestHelperJava.getProblem("sample-problem-request.ttl")
        val planModel = PlanRequestHelperJava.requestPlan(problemModel)
        tryRegisterPlan(planModel)
    }

    fun triggerSamplePlanning() {
        val requestModel = buildSamplePlanRequest()
        val planModel = PlanRequestHelperJava.requestPlan(requestModel)
        tryRegisterPlan(planModel)
    }

    private fun tryRegisterPlan(planModel: Model) {
        try {
            val (first, second) = unmarshalPlan(planModel)
            registerNewPlan(first, second)
        } catch (e: LyoJenaModelException) {
            log.error("Cannot unmarshal the Plan")
        }

    }

    private fun buildSamplePlanRequest(): Model {
        val planRequestHelper = PlanRequestHelper()
        var requestBuilder = PlanRequestBuilder()
        val stateBuilder = ProblemBuilder()
        // TODO Andrew@2019-02-19: set OslcRdfHelper base or use a real URI
        requestBuilder = requestBuilder.problemUri(OslcRdfHelper.u("scott-warehouse-problem"))
            .genLabel("TRS-Safety prototype plan request")
            .problemDomain(Link(OslcRdfHelper.u("scott-warehouse")))
            .domainFromModel(planRequestHelper.genDomain()).withStateBuilder(stateBuilder)

        stateBuilder.warehouseSize(10, 10).robotsActive(Arrays.asList("rb1", "rb2", "rb3", "rb4"))
            .robotsInactive(Arrays.asList("ob1", "ob2", "ob3")).shelfAt("sh1", 0, 10)
            .beltAt("cb1", 0, 20).boxOnShelfInit("b1", "sh1").robotAtInit("rb1", 1, 1)
            .robotAtInit("rb2", 1, 2).robotAtInit("rb3", 1, 3).robotAtInit("rb4", 1, 4)
            .robotAtInit("ob1", 3, 2).robotAtInit("ob2", 3, 3).robotAtInit("ob3", 3, 4)
            .boxOnBeltGoal("b1", "cb1")
        // TODO Andrew@2019-02-19: do we need negation too?
        //.boxNotOnShelfGoal("b1", "sh1");

        return requestBuilder.build(URI.create("blablaBASE"))
    }

    @Throws(LyoJenaModelException::class)
    private fun unmarshalPlan(planModel: Model): Pair<Plan, Array<Any>> {
        val plan = extractPlan(planModel)
        log.debug("Received plan: {}", plan)
        val planResources = PlanRequestHelperJava.getPlanResources(planModel, plan)

        return Pair(plan, planResources)
    }

    private fun registerNewPlan(plan: Plan, planResources: Array<Any>) {
        val key = plan.about
        if (plans.containsKey(key)) {
            throw IllegalStateException("The plan with the same URI has already been registered")
        }
        plans[key] = planResources
        // TODO Andrew@2019-02-20: check the with Jad what hack from Yash did he mean
        // TODO Andrew@2018-02-23: here the plan needs to be put through lyo store update
        WarehouseControllerManager.getChangeHistories().addResource(plan)
    }

    @Throws(LyoJenaModelException::class)
    private fun extractPlan(planModel: Model): Plan {
        val plan = JenaModelHelper.unmarshalSingle(planModel, Plan::class.java)
        // TODO Andrew@2018-02-26: extract method
        val planId = (plans.size + 1).toString()
        plan.about = WarehouseControllerResourcesFactory.constructURIForPlan(
            AdaptorHelper.DEFAULT_SP_ID, planId)
        return plan
    }


}
