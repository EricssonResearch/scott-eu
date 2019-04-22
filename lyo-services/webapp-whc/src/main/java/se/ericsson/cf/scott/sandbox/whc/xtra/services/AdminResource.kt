package se.ericsson.cf.scott.sandbox.whc.xtra.services

import eu.scott.warehouse.lib.OslcHelper
import org.apache.commons.lang3.exception.ExceptionUtils
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager
import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig
import se.ericsson.cf.scott.sandbox.whc.xtra.clients.TwinClient
import se.ericsson.cf.scott.sandbox.whc.xtra.managers.PlanningManager
import se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelper
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinRepository
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

    // TODO Andrew@2019-03-12: inject this
    private val twinsRepository: TwinRepository
        get() = WarehouseControllerManager.getTwinRepository()

    /**
     * POSTING to this resource shall create one plan per robot and send them out.
     */
    @POST
    @Path("plan_trigger")
    fun plan(): Response {
//        triggerPlanning()
        runAsync(this::triggerPlanning)
        return Response.noContent().build()
    }

    // TODO Andrew@2019-03-12: move to the lib
    private fun runAsync(function: () -> Unit) {
        log.trace("Scheduling function execution w/o delay")
        WarehouseControllerManager.getExecService().schedule({
            try {
                function.invoke()
            } catch (e: Throwable) {
                log.warn("PlanningManager threw an exception: ${ExceptionUtils.getStackTrace(e)}")
            }
        }, 0, TimeUnit.MILLISECONDS)
    }

    fun triggerPlanning() {
        log.trace("triggerSamplePlanning() called")
        val planRequestHelper = PlanRequestHelper(OslcHelper(WhcConfig.getBaseUri()))
        val twinClient = TwinClient()
        val planRepository = WarehouseControllerManager.getPlanRepository()
        PlanningManager(planRequestHelper, twinClient, planRepository).planForEachTwin(
            twinsRepository)
        log.trace("triggerSamplePlanning() finished")
    }
}
