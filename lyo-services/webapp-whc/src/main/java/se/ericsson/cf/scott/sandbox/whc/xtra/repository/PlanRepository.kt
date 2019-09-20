package se.ericsson.cf.scott.sandbox.whc.xtra.repository

import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.lib.InstanceWithResources
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerResourcesFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig
import java.net.URI
import java.util.Optional
import java.util.concurrent.atomic.AtomicLong


interface PlanRepository {
    fun registerPlan(plan: InstanceWithResources<Plan>): URI
    fun removePlan(planURI: URI)
    fun getPlanByURI(planURI: URI): Optional<InstanceWithResources<Plan>>
    fun existsPlan(planURI: URI): Boolean
    fun newURI(): URI
}

class PlanRepositoryMem : PlanRepository {
    private val idCounter = AtomicLong()
    private val plansByURI: MutableMap<URI, InstanceWithResources<Plan>> = LinkedHashMap()

    override fun registerPlan(plan: InstanceWithResources<Plan>): URI {
        if (plan.instance.about == null) {
            plan.instance.about = newURI()
        } else if (!plan.instance.about.isAbsolute) {
            throw IllegalArgumentException(
                "Plan URI must be absolute (current: '$plan.instance.about')")
        }
        plansByURI[plan.instance.about] = plan
        return plan.instance.about
    }

    // not really thread-safe but we can deal with it for now
    override fun existsPlan(planURI: URI): Boolean = plansByURI.containsKey(planURI)

    override fun getPlanByURI(planURI: URI): Optional<InstanceWithResources<Plan>> =
        Optional.ofNullable(plansByURI[planURI])

    override fun newURI(): URI {
        return WarehouseControllerResourcesFactory.constructURIForPlan(WhcConfig.DEFAULT_SP_ID,
            idCounter.incrementAndGet().toString())
    }

    override fun removePlan(planURI: URI): Unit = TODO("not implemented")
}
