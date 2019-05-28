package se.ericsson.cf.scott.sandbox.twin.xtra

import eu.scott.warehouse.domains.pddl.Action
import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.domains.pddl.Step
import eu.scott.warehouse.domains.scott.DropBelt
import eu.scott.warehouse.domains.scott.MoveToWp
import eu.scott.warehouse.domains.scott.PickShelf
import eu.scott.warehouse.domains.twins.PlanExecutionRequest
import eu.scott.warehouse.lib.OslcHelpers
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.eclipse.lyo.oslc4j.client.OslcClient
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.io.BufferedInputStream
import java.util.concurrent.ScheduledExecutorService
import javax.ws.rs.core.Response
import java.io.InputStream
import java.net.ConnectException
import java.util.Optional


class PlanExecutionService(private val executor: ScheduledExecutorService,
                           private val client: OslcClient) {
    companion object {
        val log: Logger = LoggerFactory.getLogger(PlanExecutionService::class.java)
    }

    fun fulfillRequest(request: PlanExecutionRequest) {
        if (request.plan == null || request.plan.value == null) {
            throw IllegalArgumentException("Plan link is missing from the $request")
        }
        val planURI = request.plan.value
        if (!planURI.isAbsolute) {
            throw IllegalArgumentException("Plan link URI is malformed: $planURI")
        }

        val uriString = planURI.toASCIIString()
        val planModelResult = requestModel(uriString)

        if (planModelResult.isPresent) {
            val plan = planModelResult.get()
            val planResource = JenaModelHelper.unmarshalSingle(plan, Plan::class.java)
            log.info("New Plan received: ${planResource.about} (cost ${planResource.cost})")
            val builder = StringBuilder()
            builder.appendln("Steps in the Plan ${planResource.about}:")
            val actionSteps = planResource.step.sortedBy { it.order }
            actionSteps.forEach { step: Step ->
                builder.append("\tStep ${step.order}: ")
                val action: IExtendedResource = OslcHelpers.navTry(plan, step.action,
                    Action::class.java, DropBelt::class.java, MoveToWp::class.java,
                    PickShelf::class.java)
                when (action) {
                    is DropBelt  -> builder.appendln("action 'drop-belt'")
                    is MoveToWp  -> builder.appendln("action 'move-to-wp'")
                    is PickShelf -> builder.appendln("action 'pick-shelf'")
                }
            }
            log.debug(builder.toString())
        }
    }

    private fun requestModel(uriString: String): Optional<Model> {
        return try {
            val response: Response = client.getResource(uriString,
                OslcMediaType.APPLICATION_RDF_XML)
            val inputStream = BufferedInputStream(response.readEntity(InputStream::class.java))
            val responseModel: Model = ModelFactory.createDefaultModel()
            RDFDataMgr.read(responseModel, inputStream, Lang.RDFXML)
            Optional.of(responseModel)
        } catch (e: ConnectException) {
            log.warn("Connection to $uriString failed")
            Optional.empty()
        }
    }

}
