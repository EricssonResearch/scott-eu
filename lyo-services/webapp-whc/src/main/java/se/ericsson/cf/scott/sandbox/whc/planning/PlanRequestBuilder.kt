package se.ericsson.cf.scott.sandbox.whc.planning

import com.google.common.collect.ImmutableList
import com.google.common.collect.ImmutableSet
import eu.scott.warehouse.domains.blocksworld.Block
import eu.scott.warehouse.domains.blocksworld.On
import eu.scott.warehouse.domains.mission.Location
import eu.scott.warehouse.domains.pddl.Constant
import eu.scott.warehouse.domains.pddl.Or
import eu.scott.warehouse.domains.pddl.PddlObject
import eu.scott.warehouse.domains.pddl.PrimitiveType
import eu.scott.warehouse.domains.pddl.Problem
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFParser
import org.eclipse.lyo.oslc4j.core.annotation.OslcName
import org.eclipse.lyo.oslc4j.core.annotation.OslcNamespace
import org.eclipse.lyo.oslc4j.core.annotation.OslcResourceShape
import org.eclipse.lyo.oslc4j.core.model.AbstractResource
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import se.ericsson.cf.scott.sandbox.whc.planning.OslcRdfHelper.OSLC
import se.ericsson.cf.scott.sandbox.whc.planning.OslcRdfHelper.PDDL
import se.ericsson.cf.scott.sandbox.whc.planning.OslcRdfHelper.RDFS
import se.ericsson.cf.scott.sandbox.whc.planning.OslcRdfHelper.ns
import se.ericsson.cf.scott.sandbox.whc.planning.OslcRdfHelper.nsSh
import se.ericsson.cf.scott.sandbox.whc.planning.OslcRdfHelper.u
import java.net.URI
import java.util.*
import javax.ws.rs.core.UriBuilder
import javax.xml.namespace.QName

/**
 * REALLY useful method
 */
val IResource.link: Link
    get() {
        return Link(this.about)
    }

/**
 * I think the QName part is XML heritage
 */
fun IExtendedResource.setProperty(name: String, p: Any) {
    this.extendedProperties[QName.valueOf(name)] = p
}

fun IExtendedResource.setProperty(name: URI, p: Any) {
    setProperty(name.toString(), p)
}

class RawResource(about: URI) : AbstractResource(about) {
    constructor() : this(u(UUID.randomUUID()))
}

fun IExtendedResource.setInstanceShape(cl: Class<*>) {
    this.setProperty(ns(OSLC, "instanceShape"), nsSh(cl))
}

fun IExtendedResource.setSuperclass(cl: Class<*>) {
    this.setProperty(ns(RDFS, "subClassOf"), ns(cl))
}

fun IExtendedResource.setLabel(l: String) {
    this.setProperty(ns(RDFS, "label"), l)
}

data class InstanceWithResources<T : IResource>(val instance: T,
                                                val resources: Collection<IResource>)

class PlanRequestBuilder {
    private val base: URI = URI.create("http://ontology.cf.ericsson.net/pddl_example/")

    init {
        OslcRdfHelper.setBase(base)
    }

    fun getPlanRequestComplete(): Model {
        val domainStatic = getDomainStatic()
        val domainDynamic = getDomainDynamic()
        domainDynamic.add(domainStatic)
        return domainDynamic
    }

    fun getDomainStatic(): Model {
        val ttl = """
        @prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
        @prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> .
        @prefix xsd: <http://www.w3.org/2001/XMLSchema#> .
        @prefix oslc: <http://open-services.net/ns/core#> .
        @prefix sh: <http://www.w3.org/ns/shacl#> .
        @prefix pddl: <http://ontology.cf.ericsson.net/pddl/> .
        @prefix : <http://ontology.cf.ericsson.net/pddl_example/> .

        :adl-blocksworld
          a pddl:Domain ;
          oslc:instanceShape pddl:DomainShape ;
          rdfs:label "adl-blocksworld" ;
          pddl:type :location ,
                    :block ;
          pddl:constant :table ;
          pddl:predicate :on ,
                         :clear ;
          pddl:function :moved ,
                        :total-moved ;
          pddl:action :move .

          :location
            a rdfs:Class ;
            rdfs:subClassOf pddl:PrimitiveType ;
            oslc:instanceShape pddl:PrimitiveTypeShape ;
            rdfs:label "location" .

          :block
            a rdfs:Class ;
            rdfs:subClassOf pddl:PrimitiveType ;
            oslc:instanceShape pddl:PrimitiveTypeShape ;
            rdfs:label "block" .

          :location-or-block
            a rdfs:Class ;
            rdfs:subClassOf pddl:EitherType ;
            oslc:instanceShape pddl:EitherTypeShape ;
            rdfs:member :location ,
                        :block .

          :on
            a rdfs:Class ;
            rdfs:subClassOf pddl:Predicate ;
            oslc:instanceShape pddl:PredicateShape ;
            rdfs:label "on" ;
            pddl:parameter :on-x ,
                           :on-y .

          :on-x
            a pddl:Parameter ;
            oslc:instanceShape pddl:ParameterShape ;
            rdfs:label "x" ;
            pddl:type :location-or-block ;
            sh:order 1 .

          :on-y
            a pddl:Parameter ;
            oslc:instanceShape pddl:ParameterShape ;
            rdfs:label "y" ;
            pddl:type :location-or-block ;
            sh:order 2 .

          :clear
            a rdfs:Class ;
            rdfs:subClassOf pddl:Predicate ;
            oslc:instanceShape pddl:PredicateShape ;
            rdfs:label "clear" ;
            pddl:parameter :clear-x .

          :clear-x
            a pddl:Parameter ;
            oslc:instanceShape pddl:ParameterShape ;
            rdfs:label "x" ;
            pddl:type :location-or-block ;
            sh:order 1 .

          :moved
            a rdfs:Class ;
            rdfs:subClassOf pddl:Function ;
            oslc:instanceShape pddl:FunctionShape ;
            rdfs:label "moved" ;
            pddl:parameter :moved-m .

          :moved-m
            a pddl:Parameter ;
            oslc:instanceShape pddl:ParameterShape ;
            rdfs:label "m" ;
            pddl:type :block ;
            sh:order 1 .

          :total-moved
            a rdfs:Class ;
            rdfs:subClassOf pddl:Function ;
            oslc:instanceShape pddl:FunctionShape ;
            rdfs:label "total-moved" .

            :move
              a rdfs:Class ;
              rdfs:subClassOf pddl:Action ;
              oslc:instanceShape pddl:ActionShape ;
              rdfs:label "move" ;
              pddl:parameter :move-b ,
                             :move-x ,
                             :move-y ;
              pddl:precondition [ a pddl:And ;
                                  pddl:argument [ a pddl:Not ;
                                                  pddl:argument [ a pddl:Equality ;
                                                                  pddl:left :move-b ;
                                                                  pddl:right :move-y
                                                                ]
                                                ] ,
                                                [ a :clear ;
                                                  :clear-x :move-b
                                                ] ,
                                                [ a :on ;
                                                  :on-x :move-b ;
                                                  :on-y :move-x
                                                ] ,
                                                [ a :clear ;
                                                  :clear-x :move-y
                                                ]
                                ] ;
              pddl:effect [ a pddl:And ;
                            pddl:argument [ a :on ;
                                            :on-x :move-b ;
                                            :on-y :move-y
                                          ] ,
                                          [ a pddl:Not ;
                                            pddl:argument [ a :on ;
                                                            :on-x :move-b ;
                                                            :on-y :move-x
                                                          ]
                                          ] ,
                                          [ a :clear ;
                                            :clear-x :move-x
                                          ] ,
                                          [ a pddl:Increase ;
                                            pddl:parameter [ a :moved ;
                                                             :moved-m :move-b
                                                           ] ;
                                            pddl:argument 1
                                          ] ,
                                          [ a pddl:Increase ;
                                            pddl:parameter [ a :total-moved ] ;
                                            pddl:argument 1
                                          ] ,
                                          [ a pddl:When ;
                                            pddl:parameter [ a pddl:Not ;
                                                             pddl:argument [ a pddl:Equality ;
                                                                             pddl:left :move-y ;
                                                                             pddl:right :table
                                                                           ]
                                                           ] ;
                                            pddl:argument [ a pddl:Not ;
                                                            pddl:argument [ a :clear ;
                                                                            :clear-x :move-y
                                                                          ]
                                                          ]
                                          ]
                          ] .

            :move-b
              a pddl:Parameter ;
              oslc:instanceShape pddl:ParameterShape ;
              rdfs:label "b" ;
              pddl:type :block ;
              sh:order 1 .

            :move-x
              a pddl:Parameter ;
              oslc:instanceShape pddl:ParameterShape ;
              rdfs:label "x" ;
              pddl:type :location-or-block ;
              sh:order 2 .

            :move-y
              a pddl:Parameter ;
              oslc:instanceShape pddl:ParameterShape ;
              rdfs:label "y" ;
              pddl:type :location-or-block ;
              sh:order 3 .
        """.trimIndent()

        val model = OslcRdfHelper.modelFrom(ttl, Lang.TURTLE)
        return model
    }

    fun getDomainDynamic(): Model {

        val resources = HashSet<IResource>()

        /*
            :table
              a :location ;
              oslc:instanceShape pddl:ConstantShape ;
              rdfs:label "table" .
        */

        val table = RawResource(u("table"))
        table.addType(u("location"))
        table.setInstanceShape(Constant::class.java)
        table.setLabel("table")

        resources.add(table)

        /*
            :a
              a :block ;
              oslc:instanceShape pddl:ObjectShape ;
              rdfs:label "a" .

            :b
              a :block ;
              oslc:instanceShape pddl:ObjectShape ;
              rdfs:label "b" .

            :c
              a :block ;
              oslc:instanceShape pddl:ObjectShape ;
              rdfs:label "c" .
         */

        val blocks: MutableMap<Any, Block> = HashMap()

        for (b in ImmutableSet.of("a", "b", "c")) {
            val block = Block(u(b))
            block.label = b
            // FIXME Andrew@2018-08-14: wrong resource name, not the exact same shape
            block.setInstanceShape(PddlObject::class.java)
            blocks[b] = block
            resources.add(block)
        }

        /*

        :adl-blocksworld-problem
          a pddl:Problem ;
          oslc:instanceShape pddl:ProblemShape ;
          rdfs:label "adl-blocksworld-problem" ;
          pddl:domain :adl-blocksworld ;
          pddl:object :a ,
                      :b ,
                      :c ,
                      :table ;
          pddl:init [ a pddl:EQ ;
                      pddl:left [ a :moved ;
                                  :moved-m :a
                                ] ;
                      pddl:right 0
                    ] ,
                    [ a pddl:EQ ;
                      pddl:left [ a :moved ;
                                  :moved-m :b
                                ] ;
                      pddl:right 0
                    ] ,
                    [ a pddl:EQ ;
                      pddl:left [ a :moved ;
                                  :moved-m :c
                                ] ;
                      pddl:right 0
                    ] ,
                    [ a pddl:EQ ;
                      pddl:left [ a :total-moved ] ;
                      pddl:right 0
                    ] ,
                    [ a :on ;
                      :on-x :b ;
                      :on-y :table
                    ] ,
                    [ a :on ;
                      :on-x :a ;
                      :on-y :table
                    ] ,
                    [ a :on ;
                      :on-x :c ;
                      :on-y :a
                    ] ,
                    [ a :clear ;
                      :clear-x :b
                    ] ,
                    [ a :clear ;
                      :clear-x :c
                    ] ,
                    [ a :clear ;
                      :clear-x :table
                    ] ;
          pddl:goal [ a pddl:Or ;
                      pddl:argument [ a :on ;
                                      :on-x :b ;
                                      :on-y :c
                                    ] ,
                                    [ a :on ;
                                      :on-x :c ;
                                      :on-y :b
                                    ]
                    ] ;
          pddl:minimize [ a pddl:total-time ].

        */

        var problem = Problem(u("adl-blocksworld-problem"))
        problem.label = "adl-blocksworld-problem"
        // this is a resource defined in the "static" part of the domain
        problem.domain = Link(u("adl-blocksworld"))
        problem.pddlObject.addAll(blocks.map { entry -> entry.value.link })
        problem.pddlObject.add(table.link)

        val initialState: Collection<IResource> = buildInitState(
                ImmutableList.of(table), ImmutableList.copyOf(blocks.values))
        val goalState: InstanceWithResources<IResource> = buildGoalState(ImmutableList.of(table),
                ImmutableList.copyOf(blocks.values))

        // FIXME Andrew@2018-08-15: shall be inlined
        problem.setInit(ImmutableSet.copyOf(initialState.map { it -> it.link }))
        resources.addAll(initialState)

        problem.goal = goalState.instance.link
        resources.addAll(goalState.resources)

        val minFn = RawResource(u(UUID.randomUUID()))
        minFn.addType(ns(PDDL, "total-time"))

        problem.minimize = minFn.link

        resources.add(problem)

        return JenaModelHelper.createJenaModel(resources.toArray())
    }

    private fun buildGoalState(tables: ImmutableList<IResource>,
                               blocks: ImmutableList<Block>?): InstanceWithResources<IResource> {
//        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        /*
          pddl:goal [ a pddl:Or ;
              pddl:argument [ a :on ;
                              :on-x :b ;
                              :on-y :c
                            ] ,
                            [ a :on ;
                              :on-x :c ;
                              :on-y :b
                            ]
            ] ;
         */

        val goal = Or(u(UUID.randomUUID()))

        val bONc = on(u("b"), u("c"))
        val cONb = on(u("c"), u("b"))
        goal.setArgument(ImmutableSet.of(bONc.link, cONb.link))

        return InstanceWithResources(goal, ImmutableList.of(goal, bONc, cONb))
    }

    fun on(x: URI, y: URI): On {
        val bONc = On(u(UUID.randomUUID()))
        bONc.setProperty(u("on-x"), x)
        bONc.setProperty(u("on-y"), y)
        return bONc
    }

    fun eq(l: IResource, r: Any): IExtendedResource {
        // FIXME Andrew@2018-08-14: EQ class
        val equal = RawResource(u(UUID.randomUUID()))
        equal.addType(ns(PDDL, "EQ"))
        equal.setProperty(ns(PDDL, "left"), l.about)
        equal.setProperty(ns(PDDL, "right"), r)

        return equal
    }

    fun eq(l: InstanceWithResources<IResource>, r: Any): InstanceWithResources<IResource> {
        val eqResource = eq(l.instance, r)
        val builder = ImmutableSet.builder<IResource>()
        builder.add(eqResource)
        builder.addAll(l.resources)
        builder.addAll(filterResources(r))
        return InstanceWithResources(eqResource, builder.build())
    }

    private fun filterResources(r: Any): Collection<IResource> {
        return when (r) {
            is IResource     -> ImmutableSet.of(r)
            is Collection<*> -> ImmutableSet.copyOf(r.filterIsInstance(IResource::class.java))
            is Array<*>      -> ImmutableSet.copyOf(r.filterIsInstance(IResource::class.java))
            else             -> ImmutableSet.of()
        }
    }

    fun clear(what: URI): IExtendedResource {
        val clear = RawResource(u(UUID.randomUUID()))
        clear.addType(u("clear"))
        clear.setProperty(u("clear-x"), what)
        return clear
    }

    private fun buildInitState(tables: List<IResource>,
                               blocks: List<Block>): Collection<IResource> {
        /*
          pddl:init [ a pddl:EQ ;
              pddl:left [ a :moved ;
                          :moved-m :a
                        ] ;
              pddl:right 0
            ] ,
            [ a pddl:EQ ;
              pddl:left [ a :moved ;
                          :moved-m :b
                        ] ;
              pddl:right 0
            ] ,
            [ a pddl:EQ ;
              pddl:left [ a :moved ;
                          :moved-m :c
                        ] ;
              pddl:right 0
            ] ,
            [ a pddl:EQ ;
              pddl:left [ a :total-moved ] ;
              pddl:right 0
            ] , */
        val values: Collection<InstanceWithResources<IResource>> = blocks.map { b -> eq(moved(b), 0) }

        /*
            [ a :on ;
              :on-x :b ;
              :on-y :table
            ] ,
            [ a :on ;
              :on-x :a ;
              :on-y :table
            ] ,
            [ a :on ;
              :on-x :c ;
              :on-y :a
            ] ,
            [ a :clear ;
              :clear-x :b
            ] ,
            [ a :clear ;
              :clear-x :c
            ] ,
            [ a :clear ;
              :clear-x :table
            ] ;
         */

        val allButLastBlocks = blocks.subList(0, blocks.size - 1)
        val onBlocks = allButLastBlocks.map { b -> on(b.about, tables.first().about) }
        val bONb = on(blocks.last().about, blocks.first().about)

        val clearBlocks = blocks.subList(1, blocks.size).map { b -> clear(b.about) }
        val clearTable = clear(tables.first().about)

        val setBuilder = ImmutableSet.builder<IResource>()
        setBuilder.addAll(tables)
        setBuilder.addAll(blocks)
        setBuilder.addAll(values.flatMap { v -> v.resources })
        setBuilder.addAll(onBlocks)
        setBuilder.addAll(clearBlocks)
        setBuilder.add(bONb)
        setBuilder.add(clearTable)

        return setBuilder.build()
    }

    private fun moved(b: Block): InstanceWithResources<IResource> {
        val m = RawResource(u(UUID.randomUUID()))
        m.addType(u("moved"))
        m.setProperty(u("moved-m"), b.about)

        return InstanceWithResources(m, ImmutableSet.of(m, b))
    }


    fun buildLocation(): RawResource {
        /*
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
}


object OslcRdfHelper {

    const val RDFS = "http://www.w3.org/2000/01/rdf-schema#"
    const val PDDL = "http://ontology.cf.ericsson.net/pddl/"
    const val OSLC = "http://open-services.net/ns/core#"
    const val SH_ORDER = "http://www.w3.org/ns/shacl#order"


    private lateinit var base: URI

    fun setBase(b: URI) {
        this.base = b
    }

    /**
     * URI of the Resource
     */
    fun ns(clazz: Class<*>): URI {
        val resourceShape = clazz.getAnnotation(OslcResourceShape::class.java)
                ?: throw IllegalArgumentException(
                        "The class does not have an OslcResourceShape annotation")
        // TODO Andrew@2018-08-13: think what do when there is more than one OSLC class described by a shape
        // FIXME Andrew@2018-08-13: think
        return URI.create(resourceShape.describes.single())
    }

    /**
     * URI of the Resource Shape
     */
    fun nsSh(clazz: Class<*>): URI {
        val oslcNamespace = clazz.getAnnotation(OslcNamespace::class.java)
        val oslcName = clazz.getAnnotation(OslcName::class.java)
        if (oslcNamespace == null || oslcName == null) {
            throw IllegalArgumentException("The class does not have OSLC annotations")
        }
        return URI.create(oslcNamespace.value + oslcName.value)
    }

    /**
     * URI from an arbitrary namespace and resource/property name
     */
    fun ns(ns: String, name: String): URI {
        return URI.create(ns + name)
    }


    fun u(p: String): URI {
        return UriBuilder.fromUri(base).path(p).build()
    }

    fun u(p: UUID?): URI {
        return u("blnk_" + p.toString())
    }

    fun modelFrom(str: String, l: Lang): Model {
        val model = ModelFactory.createDefaultModel()
        RDFParser.fromString(str.trimIndent()).lang(l).parse(model.graph)
        return model
    }
}
