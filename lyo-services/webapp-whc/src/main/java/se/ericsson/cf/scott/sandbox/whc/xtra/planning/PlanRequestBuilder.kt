package se.ericsson.cf.scott.sandbox.whc.xtra.planning

import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.eclipse.lyo.oslc4j.core.model.Link
import org.slf4j.LoggerFactory
import java.net.URI

/**
 * TODO
 *
 * @since   TODO
 */
class PlanRequestBuilder {
    companion object {
        val log = LoggerFactory.getLogger(javaClass)
    }

    private lateinit var problemBuilder: ProblemBuilder

    private lateinit var domain: Model

    fun build(baseURI: URI): Model {
        // TODO Andrew@2019-02-19: remove all use of URIs beforehand?

        val m = ModelFactory.createDefaultModel()

        m.add(domain)

        val state: ProblemRequestState = problemBuilder.build(baseURI)
        m.add(state.model)

        // FIXME Andrew@2019-02-20: where is the min fn added?

        return m
    }

    fun domainFromModel(domain: Model): PlanRequestBuilder {
        this.domain = domain
        return this
    }

    fun withStateBuilder(problemBuilder: ProblemBuilder): PlanRequestBuilder {
        this.problemBuilder = problemBuilder
        return this
    }

    fun problemUri(uri: URI): PlanRequestBuilder {
        TODO("not implemented")
        return this
    }

    fun genLabel(s: String): PlanRequestBuilder {
        // TODO Andrew@2019-02-19: append the details about the plan here, eg size and no. robots
        TODO("not implemented")
        return this
    }

    fun problemDomain(link: Link): PlanRequestBuilder {
        // TODO Andrew@2019-02-19: why do we need this? can we just read it from the domain model?
        TODO("not implement")
        return this
    }

}
