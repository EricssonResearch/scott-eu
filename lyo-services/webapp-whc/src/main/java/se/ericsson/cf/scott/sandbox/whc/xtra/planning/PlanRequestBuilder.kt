package se.ericsson.cf.scott.sandbox.whc.xtra.planning

import org.apache.jena.rdf.model.Model
import org.slf4j.LoggerFactory

/**
 * TODO
 *
 * @since   TODO
 */
class PlanRequestBuilder {
    companion object {
        val log = LoggerFactory.getLogger(javaClass)
    }

    fun build() {
        log.error("Nothing to build yet")
    }

    fun domainFromModel(domain: Model): PlanRequestBuilder {
        log.error("Domain init not implemented")
        return this
    }

    fun warehouseSize(width: Int, height: Int): PlanRequestBuilder {
        log.error("create width x height waypoints")
        return this
    }

    fun robotsActive(activeCount: Int): PlanRequestBuilder {
        log.error("create active & inactive robots with random placement")
        return this
    }

    fun robotsInactive(inactiveCount: Int): PlanRequestBuilder {
        log.error("create active & inactive robots with random placement")
        return this
    }
}
