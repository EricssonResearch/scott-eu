@file:Suppress("unused")

package se.ericsson.cf.scott.sandbox.whc.xtra.planning

import com.google.common.collect.ImmutableSet
import eu.scott.warehouse.domains.blocksworld.Block
import eu.scott.warehouse.domains.blocksworld.Move
import eu.scott.warehouse.domains.pddl.Action
import eu.scott.warehouse.domains.pddl.Plan
import eu.scott.warehouse.domains.pddl.PrimitiveType
import eu.scott.warehouse.lib.InstanceWithResources
import eu.scott.warehouse.lib.OslcHelpers
import eu.scott.warehouse.lib.OslcHelpers.OSLC
import eu.scott.warehouse.lib.OslcHelpers.PDDL
import eu.scott.warehouse.lib.OslcHelpers.RDFS
import eu.scott.warehouse.lib.OslcHelpers.ns
import eu.scott.warehouse.lib.OslcHelpers.nsSh
import eu.scott.warehouse.lib.OslcHelpers.u
import eu.scott.warehouse.lib.RawResource
import eu.scott.warehouse.lib.RdfHelpers
import eu.scott.warehouse.lib.setLabel
import eu.scott.warehouse.lib.setProperty
import org.apache.http.client.methods.HttpPost
import org.apache.http.entity.ByteArrayEntity
import org.apache.http.impl.client.HttpClientBuilder
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper
import java.io.ByteArrayOutputStream
import java.io.IOException
import java.io.InputStream
import java.net.URI
import java.util.ArrayList
import java.util.UUID
import javax.xml.namespace.QName

@Suppress("MemberVisibilityCanBePrivate")
class PlanRequestHelper {

    companion object {

        val log: Logger = LoggerFactory.getLogger(PlanRequestHelper::class.java)

        fun minFn(): RawResource {
            val minFn = RawResource(u(UUID.randomUUID()))
            minFn.addType(ns(PDDL, "total-time"))
            return minFn
        }

        fun freeRobot(robot: IExtendedResource): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("free-robot"))
            r.setProperty(u("free-robot-rb"), robot.about)
            return InstanceWithResources(r, ImmutableSet.of(r, robot))
        }

        fun canMove(wpFrom: IExtendedResource,
                    wpTo: IExtendedResource): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("CanMovePredicate"))
            r.setProperty(u("CanMoveW1Param"), wpFrom.about)
            r.setProperty(u("CanMoveW2Param"), wpTo.about)

            return InstanceWithResources(r, ImmutableSet.of(r, wpFrom, wpTo))
        }

        fun and(
            vararg predicates: InstanceWithResources<IExtendedResource>): InstanceWithResources<IExtendedResource> {
            // I don't want to use Kotlin spread operator '*predicates' because it only works with arrays.
            return and(predicates.asList())
        }

        fun and(
            predicates: Collection<InstanceWithResources<IExtendedResource>>): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(ns(PDDL, "And"))
            r.setProperty(ns(PDDL, "argument"), predicates.map { it.instance.about })
            val resources = HashSet(predicates.flatMap { it.resources })
            resources.add(r)
            return InstanceWithResources(r, resources)
        }

        fun robotAt(robot: IExtendedResource, wp: IExtendedResource): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("robot-at"))
            r.setProperty(u("robot-at-rb"), robot.about)
            r.setProperty(u("robot-at-wp"), wp.about)
            return InstanceWithResources(r, ImmutableSet.of(r, robot, wp))
        }

        fun beltAt(belt: IExtendedResource, wp: IExtendedResource): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("belt-at"))
            r.setProperty(u("belt-at-cb"), belt.about)
            r.setProperty(u("belt-at-wp"), wp.about)
            return InstanceWithResources(r, ImmutableSet.of(r, belt, wp))

        }

        fun shelfAt(shelf: IExtendedResource,                    wp: IExtendedResource): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("shelf-at"))
            r.setProperty(u("shelf-at-sh"), shelf.about)
            r.setProperty(u("shelf-at-wp"), wp.about)
            return InstanceWithResources(r, ImmutableSet.of(r, shelf, wp))
        }


        fun onShelf(box: IExtendedResource,
                    shelf: IExtendedResource): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("on-shelf"))
            r.setProperty(u("on-shelf-b"), box.about)
            r.setProperty(u("on-shelf-sh"), shelf.about)
            return InstanceWithResources(r, ImmutableSet.of(r, shelf, box))
        }

        fun onBelt(box: IExtendedResource,
                   belt: IExtendedResource): InstanceWithResources<IExtendedResource> {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("on-belt"))
            r.setProperty(u("on-belt-cb"), belt.about)
            r.setProperty(u("on-belt-b"), box.about)
            return InstanceWithResources(r, ImmutableSet.of(r, belt, box))
        }

        fun waypointLabel(x: Int, y: Int): String = "x${x}y${y}"

        fun waypoint(x: Int, y: Int): IExtendedResource {
            val id = waypointLabel(x, y)
            return pddlInstance(id, "Waypoint")
        }

        fun shelf(id: String): IExtendedResource {
            return pddlInstance(id, "Shelf")
        }

        fun box(id: String): IExtendedResource {
            return pddlInstance(id, "Box")
        }

        fun robot(id: String): IExtendedResource {
            return pddlInstance(id, "Robot")
        }

        fun belt(id: String): IExtendedResource {
            return pddlInstance(id, "ConveyorBelt")
        }

        fun pddlInstance(id: String, type: String): IExtendedResource {
            val pddlResource = RawResource(u(id))
            pddlResource.setLabel(id)
            pddlResource.addType(u(type))

            return pddlResource
        }

        @Deprecated("not using coordinates any more + this predicate is from the BoxWorld domain")
        fun clear(what: URI): IExtendedResource {
            val clear = RawResource(u(UUID.randomUUID()))
            clear.addType(u("clear"))
            clear.setProperty(u("clear-x"), what)
            return clear
        }

        fun moved(b: Block): InstanceWithResources<IExtendedResource> {
            val m = RawResource(u(UUID.randomUUID()))
            m.addType(u("moved"))
            m.setProperty(u("moved-m"), b.about)

            return InstanceWithResources(m, ImmutableSet.of(m, b))
        }

        fun buildLocation(): RawResource {
            /**
            :location
            a rdfs:Class ;
            rdfs:subClassOf pddl:PrimitiveType ;
            oslc:instanceShape pddl:PrimitiveTypeShape ;
            rdfs:label "location" .
             */
            val location = RawResource(u("location"))
            location.types.add(ns(RDFS, "Class"))
            location.setProperty(ns(RDFS, "subClassOf"), ns(PrimitiveType::class.java))
            location.setProperty(ns(OSLC, "instanceShape"), nsSh(PrimitiveType::class.java))
            location.setProperty(ns(RDFS, "label"), "location")
            return location
        }

        @Deprecated("not using coordinates any more + this predicate is from the BoxWorld domain")
        fun on(x: URI, y: URI): IExtendedResource {
            val r = RawResource(u(UUID.randomUUID()))
            r.addType(u("on"))
            r.setProperty(u("on-x"), x)
            r.setProperty(u("on-y"), y)
            return r
        }

        fun eq(l: IExtendedResource, r: Any): IExtendedResource {
            // TODO Andrew@2019-02-18: https://github.com/EricssonResearch/scott-eu/issues/155
            val equal = RawResource(u(UUID.randomUUID()))
            equal.addType(ns(PDDL, "EQ"))
            equal.setProperty(ns(PDDL, "left"), l.about)
            equal.setProperty(ns(PDDL, "right"), r)

            return equal
        }

        fun eq(l: InstanceWithResources<IExtendedResource>,
               r: Any): InstanceWithResources<IExtendedResource> {
            val eqResource = eq(l.instance, r)
            val builder = ImmutableSet.builder<IExtendedResource>()
            builder.add(eqResource)
            builder.addAll(l.resources)
            builder.addAll(filterResources(r))
            return InstanceWithResources(eqResource, builder.build())
        }

        private fun filterResources(r: Any): Collection<IExtendedResource> {
            return when (r) {
                is IExtendedResource -> ImmutableSet.of(r)
                is Collection<*>     -> ImmutableSet.copyOf(
                    r.filterIsInstance(IExtendedResource::class.java))
                is Array<*>          -> ImmutableSet.copyOf(
                    r.filterIsInstance(IExtendedResource::class.java))
                else                 -> ImmutableSet.of()
            }
        }

        fun getProblem(resourceName: String): Model {
            return AdaptorHelper.loadJenaModelFromResource(resourceName, Lang.TURTLE)
        }

        private fun planForProblem(problemModel: Model): Model {
            log.trace("Problem request\n{}", RdfHelpers.modelToString(problemModel))
            try {
                val response = requestPlanManually(problemModel)
                val responsePlan = ModelFactory.createDefaultModel()
                RDFDataMgr.read(responsePlan, response, Lang.TURTLE)
                log.info("Plan response\n{}", RdfHelpers.modelToString(responsePlan))
                return responsePlan
            } catch (e: IOException) {
                log.error("Something went wrong", e)
                throw IllegalStateException(e)
            }

        }

        @Throws(IOException::class)
        private fun requestPlanManually(problemModel: Model): InputStream {
            val out = ByteArrayOutputStream()
            RDFDataMgr.write(out, problemModel, RDFFormat.TURTLE_BLOCKS)

            val client = HttpClientBuilder.create()
                .build()
            val uri = AdaptorHelper.p("planner.cf_uri")
            log.debug("Using $uri as a Planner CF endpoint")
            val post = HttpPost(uri)

            post.setHeader("Content-type", AdaptorHelper.MIME_TURTLE)
            post.setHeader("Accept", AdaptorHelper.MIME_TURTLE)
            post.entity = ByteArrayEntity(out.toByteArray())

            val response = client.execute(post)
            val statusCode = response.statusLine.statusCode
            log.debug("Status code from the Planner CF endpoint response: $statusCode")
            if (statusCode >= 400) {
                throw IllegalStateException(
                    "Planning request on $uri failed with the code $statusCode")
            }
            return response.entity.content
        }

        @Throws(LyoJenaModelException::class)
        fun getPlanResources(planModel: Model, plan: Plan): Array<Any> {
            val planResources = ArrayList<IResource>()
            planResources.add(plan)
            // TODO Andrew@2018-02-23: why not getSteps?
            val planSteps = plan.step
            for (step in planSteps) {
                step.order = (step.extendedProperties as Map<QName, Any>).getOrDefault(
                    QName(AdaptorHelper.NS_SHACL, "order"), null) as Int
                val action = OslcHelpers.navTry(planModel, step.action, Action::class.java,
                    Move::class.java)
                planResources.add(step)
                planResources.add(action)
                log.info("Step {}: {}", step.order, action)
            }
            return planResources.toTypedArray()
        }

        fun requestPlan(problemModel: Model): Model {
            val planModel = planForProblem(problemModel)
            RdfHelpers.skolemize(planModel)
            return planModel
        }
    }

    init {
        // Don't use #, see https://github.com/EricssonResearch/scott-eu/issues/154
        OslcHelpers.base = URI.create("http://ontology.cf.ericsson.net/ns/scott-warehouse/")
    }
}
