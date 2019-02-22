package se.ericsson.cf.scott.sandbox.whc.xtra.services

import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager
import se.ericsson.cf.scott.sandbox.whc.xtra.managers.PlanningManager
import java.util.concurrent.TimeUnit
import javax.ws.rs.POST
import javax.ws.rs.Path
import javax.ws.rs.core.Response

/**
 * A REST resource with misc admin endpoints
 *
 */
@Path("admin")
class AdminResource {
    companion object {
        val log: Logger = LoggerFactory.getLogger(AdminResource::class.java)
    }

    @POST
    @Path("plan_trigger")
    fun triggerPlanning(): Response {
        triggerPlanningDirect()
//        runAsync(this::triggerPlanningDirect)
        return Response.noContent().build()
    }

    private fun runAsync(function: () -> Unit) {
        log.trace("Scheduling function execution w/o delay")
        WarehouseControllerManager.getExecService().schedule({
            try {
                function.invoke()
            } catch (e: Throwable) {
                log.warn("PlanningManager threw an exception: $e")
            }
        }, 0, TimeUnit.MILLISECONDS)
    }

    private fun triggerPlanningDirect() {
        log.trace("triggerSamplePlanning() called")
        PlanningManager.triggerSamplePlanning()
        log.trace("triggerSamplePlanning() finished")
    }
}
