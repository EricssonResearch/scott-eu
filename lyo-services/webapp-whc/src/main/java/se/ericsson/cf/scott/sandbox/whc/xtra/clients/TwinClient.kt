package se.ericsson.cf.scott.sandbox.whc.xtra.clients

import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.domains.twins.PlanExecutionRequest
import eu.scott.warehouse.lib.InstanceWithResources
import eu.scott.warehouse.lib.link
import org.eclipse.lyo.oslc4j.client.OslcClient
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.repository.TwinInfo
import java.net.URI
import javax.ws.rs.core.Response

/**
 * TODO
 *
 * @since   TODO
 */
class TwinClient() {
    companion object {
        val log: Logger = LoggerFactory.getLogger(TwinClient::class.java)
    }

    private val client: OslcClient = OslcClient()


    fun requestPlanExecution(twinInfo: TwinInfo, plan: InstanceWithResources<Plan>) {
        val execRequest = PlanExecutionRequest()
        execRequest.plan = plan.instance.link

        val response = client.createResource(twinInfo.cfUri, execRequest,
            OslcMediaType.TEXT_TURTLE)
        if (response.statusInfo.family == Response.Status.Family.SUCCESSFUL) {
            log.info("Plan '${plan.instance.about}' was successfully POSTed")
        } else {
            log.warn("There was a problem with the plan '${plan.instance.about}'" +
                ": ${response.statusInfo} (${twinInfo.cfUri})")
        }
    }
}
