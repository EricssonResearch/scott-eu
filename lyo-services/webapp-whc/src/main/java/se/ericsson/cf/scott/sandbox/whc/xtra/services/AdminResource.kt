package se.ericsson.cf.scott.sandbox.whc.xtra.services

import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.managers.TRSManager
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
        val log = LoggerFactory.getLogger(AdminResource::class.java)
    }

    @POST
    @Path("plan_trigger")
    fun triggerPlanning(): Response {
        log.warn("Planning trigger not implemented yet.")
        // FIXME Andrew@2019-02-12: replace with the impl discussed in W6'19
        TRSManager.fetchPlanForProblem("sample-problem-request.ttl")
        return Response.noContent().build()
    }
}
