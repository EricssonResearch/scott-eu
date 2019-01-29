package se.ericsson.cf.scott.sandbox.whc.planning

import com.google.common.collect.ImmutableList
import com.google.common.collect.ImmutableSet
import eu.scott.warehouse.domains.blocksworld.Block
import eu.scott.warehouse.domains.pddl.Or
import eu.scott.warehouse.domains.pddl.PrimitiveType
import eu.scott.warehouse.domains.pddl.Problem
import eu.scott.warehouse.lib.OslcRdfHelper
import eu.scott.warehouse.lib.OslcRdfHelper.OSLC
import eu.scott.warehouse.lib.OslcRdfHelper.PDDL
import eu.scott.warehouse.lib.OslcRdfHelper.RDFS
import eu.scott.warehouse.lib.OslcRdfHelper.ns
import eu.scott.warehouse.lib.OslcRdfHelper.nsSh
import eu.scott.warehouse.lib.OslcRdfHelper.u
import eu.scott.warehouse.lib.link
import eu.scott.warehouse.lib.setLabel
import eu.scott.warehouse.lib.setProperty
import org.apache.jena.rdf.model.Model
import org.apache.jena.riot.Lang
import org.eclipse.lyo.oslc4j.core.model.AbstractResource
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import java.math.BigInteger
import java.net.URI
import java.util.UUID

data class InstanceWithResources<T : IResource>(val instance: T,
                                                val resources: Collection<IResource>)

data class InstanceMultiWithResources<T : IResource>(val instance: Collection<T>,
                                                val resources: Collection<IResource>)

class PlanRequestBuilder {
    // FIXME Andrew@2018-08-27: fix support for # in the URIs
    private val base = URI.create("http://ontology.cf.ericsson.net/ns/scott-warehouse/")

    init {
        OslcRdfHelper.setBase(base)
    }

    fun getPlanRequestComplete(): Model {
        val domain = genDomain()
        val problem = genProblem()
        problem.add(domain)
        return problem
    }

    fun genDomain(): Model {
        val ttl = """
@prefix :      <http://ontology.cf.ericsson.net/ns/scott-warehouse/> .
@prefix rdf:   <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
@prefix sh:    <http://www.w3.org/ns/shacl#> .
@prefix xsd:   <http://www.w3.org/2001/XMLSchema#> .
@prefix pddl:  <http://ontology.cf.ericsson.net/pddl/> .
@prefix rdfs:  <http://www.w3.org/2000/01/rdf-schema#> .
@prefix oslc:  <http://open-services.net/ns/core#> .

# DOMAIN

:scott-warehouse  a         pddl:Domain ;
        rdfs:label          "scott-warehouse" ;
        pddl:type           :Box, :Robot, :ConveyorBelt, :Shelf, :Coord;
        pddl:predicate      :robot-at, :on-belt, :on-shelf, :on-robot, :free-robot, :shelf-at, :belt-at;
        pddl:action         :pick-shelf, :move-to, :drop-belt ;
        oslc:instanceShape  pddl:DomainShape .

# TYPES

:Box    a                   rdfs:Class ;
        rdfs:label          "box" ;
        rdfs:subClassOf     pddl:PrimitiveType ;
        oslc:instanceShape  pddl:PrimitiveTypeShape .

:Robot  a                   rdfs:Class ;
        rdfs:label          "robot" ;
        rdfs:subClassOf     pddl:PrimitiveType ;
        oslc:instanceShape  pddl:PrimitiveTypeShape .

:ConveyorBelt  a           rdfs:Class ;
        rdfs:label          "conveyor-belt" ;
        rdfs:subClassOf     pddl:PrimitiveType ;
        oslc:instanceShape  pddl:PrimitiveTypeShape .

:Shelf  a           rdfs:Class ;
        rdfs:label          "shelf" ;
        rdfs:subClassOf     pddl:PrimitiveType ;
        oslc:instanceShape  pddl:PrimitiveTypeShape .

:Coord  a           rdfs:Class ;
        rdfs:label          "coord" ;
        rdfs:subClassOf     pddl:PrimitiveType ;
        oslc:instanceShape  pddl:PrimitiveTypeShape .

# PREDICATES

:robot-at  a                rdfs:Class ;
        rdfs:label          "robot-at" ;
        rdfs:subClassOf     pddl:Predicate ;
        pddl:parameter      :robot-at-rb, :robot-at-x, :robot-at-y ;
        oslc:instanceShape  pddl:PredicateShape .

:robot-at-rb  a             pddl:Parameter ;
        rdfs:label          "rb" ;
        pddl:type           :Robot ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:robot-at-x  a             pddl:Parameter ;
        rdfs:label          "x" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:robot-at-y  a             pddl:Parameter ;
        rdfs:label          "y" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            3 .

:on-belt  a                   rdfs:Class ;
        rdfs:label          "on-belt" ;
        rdfs:subClassOf     pddl:Predicate ;
        pddl:parameter      :on-belt-b, :on-belt-cb;
        oslc:instanceShape  pddl:PredicateShape .

:on-belt-b  a             pddl:Parameter ;
        rdfs:label          "b" ;
        pddl:type           :Box ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:on-belt-cb  a             pddl:Parameter ;
        rdfs:label          "cb" ;
        pddl:type           :ConveyorBelt ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:on-shelf  a                   rdfs:Class ;
        rdfs:label          "on-shelf" ;
        rdfs:subClassOf     pddl:Predicate ;
        pddl:parameter      :on-shelf-b, :on-shelf-sh ;
        oslc:instanceShape  pddl:PredicateShape .

:on-shelf-b  a             pddl:Parameter ;
        rdfs:label          "b" ;
        pddl:type           :Box ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:on-shelf-sh  a             pddl:Parameter ;
        rdfs:label          "sh" ;
        pddl:type           :Shelf ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:on-robot  a                   rdfs:Class ;
        rdfs:label          "on-robot" ;
        rdfs:subClassOf     pddl:Predicate ;
        pddl:parameter      :on-robot-b, :on-robot-rb ;
        oslc:instanceShape  pddl:PredicateShape .

:on-robot-b  a             pddl:Parameter ;
        rdfs:label          "b" ;
        pddl:type           :Box ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:on-robot-rb  a             pddl:Parameter ;
        rdfs:label          "rb" ;
        pddl:type           :Robot ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .


:free-robot  a                   rdfs:Class ;
        rdfs:label          "free-robot" ;
        rdfs:subClassOf     pddl:Predicate ;
        pddl:parameter      :free-robot-rb ;
        oslc:instanceShape  pddl:PredicateShape .

:free-robot-rb  a             pddl:Parameter ;
        rdfs:label          "rb" ;
        pddl:type           :Robot ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .


:shelf-at  a                   rdfs:Class ;
        rdfs:label          "shelf-at" ;
        rdfs:subClassOf     pddl:Predicate ;
        pddl:parameter      :shelf-at-sh, :shelf-at-x, :shelf-at-y ;
        oslc:instanceShape  pddl:PredicateShape .

:shelf-at-sh  a             pddl:Parameter ;
        rdfs:label          "sh" ;
        pddl:type           :Shelf ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:shelf-at-x  a             pddl:Parameter ;
        rdfs:label          "x" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:shelf-at-y  a             pddl:Parameter ;
        rdfs:label          "y" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            3 .

:belt-at  a                   rdfs:Class ;
        rdfs:label          "belt-at" ;
        rdfs:subClassOf     pddl:Predicate ;
        pddl:parameter      :belt-at-cb, :belt-at-x, :belt-at-y ;
        oslc:instanceShape  pddl:PredicateShape .

:belt-at-cb  a             pddl:Parameter ;
        rdfs:label          "cb" ;
        pddl:type           :ConveyorBelt ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order           1 .

:belt-at-x  a             pddl:Parameter ;
        rdfs:label          "x" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:belt-at-y  a             pddl:Parameter ;
        rdfs:label          "y" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            3 .

# ACTIONS


:move-to   a                   rdfs:Class ;
        oslc:instanceShape  pddl:ActionShape ;
        rdfs:label          "move-to" ;
        rdfs:subClassOf     pddl:Action ;
        pddl:parameter      :move-to-rb, :move-to-x1, :move-to-y1, :move-to-x2, :move-to-y2 ;
        pddl:precondition   [ a              pddl:And ;
                              pddl:argument  [ a         :robot-at ;
                                               :robot-at-rb :move-to-rb;
                                               :robot-at-x :move-to-x1;
                                               :robot-at-y :move-to-y1 ] ;
                            ] ;
        pddl:effect   [ a  pddl:And ;
                        pddl:argument  [ a         :robot-at ;
                                         :robot-at-rb :move-to-rb;
                                         :robot-at-x :move-to-x2;
                                         :robot-at-y :move-to-y2 ] ;
                      ] .

:move-to-rb  a             pddl:Parameter ;
        rdfs:label          "rb" ;
        pddl:type           :Robot ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:move-to-x1  a             pddl:Parameter ;
        rdfs:label          "x1" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:move-to-y1  a             pddl:Parameter ;
        rdfs:label          "y1" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            3 .

:move-to-x2  a             pddl:Parameter ;
        rdfs:label          "x2" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            4 .

:move-to-y2  a             pddl:Parameter ;
        rdfs:label          "y2" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            5 .

:pick-shelf   a                   rdfs:Class ;
        oslc:instanceShape  pddl:ActionShape ;
        rdfs:subClassOf     pddl:Action ;
        rdfs:label          "pick-shelf" ;
        pddl:parameter      :pick-shelf-rb, :pick-shelf-b, :pick-shelf-sh, :pick-shelf-x, :pick-shelf-y ;
        pddl:precondition   [ a              pddl:And ;
                              pddl:argument  [ a         :on-shelf ;
                                              :on-shelf-b :pick-shelf-b;
                                              :on-shelf-sh :pick-shelf-sh ] ;
                             pddl:argument  [ a         :shelf-at ;
                                              :shelf-at-sh :pick-shelf-sh;
                                              :shelf-at-x :pick-shelf-x;
                                              :shelf-at-y :pick-shelf-y ] ;
                              pddl:argument  [ a         :robot-at ;
                                               :robot-at-rb :pick-shelf-rb;
                                               :robot-at-x :pick-shelf-x;
                                               :robot-at-y :pick-shelf-y ] ;
                              pddl:argument  [ a         :free-robot ;
                                               :free-robot-rb :pick-shelf-rb ] ;
                            ] ;
        pddl:effect   [ a  pddl:And ;
                        pddl:argument  [ a  :on-robot ;
                                         :on-robot-b :pick-shelf-b;
                                         :on-robot-rb :pick-shelf-rb ] ;
                        pddl:argument  [ a pddl:Not ;
                                         pddl:argument [ a         :on-shelf ;
                                                          :on-shelf-b :pick-shelf-b;
                                                          :on-shelf-sh :pick-shelf-sh ] ];
                        pddl:argument  [ a pddl:Not ;
                                         pddl:argument [ a         :free-robot ;
                                                          :free-robot-rb :pick-shelf-rb ] ] ;
                      ] .


:pick-shelf-rb  a             pddl:Parameter ;
        rdfs:label          "rb" ;
        pddl:type           :Robot ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:pick-shelf-b  a             pddl:Parameter ;
        rdfs:label          "b" ;
        pddl:type           :Box ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:pick-shelf-sh  a             pddl:Parameter ;
        rdfs:label          "sh" ;
        pddl:type           :Shelf ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            3 .

:pick-shelf-x  a             pddl:Parameter ;
        rdfs:label          "x" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            4 .

:pick-shelf-y  a             pddl:Parameter ;
        rdfs:label          "y" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            5 .

:drop-belt   a                   rdfs:Class ;
        oslc:instanceShape  pddl:ActionShape ;
        rdfs:label          "drop-belt" ;
        rdfs:subClassOf     pddl:Action ;
        pddl:parameter      :drop-belt-rb, :drop-belt-b, :drop-belt-cb, :drop-belt-x, :drop-belt-y ;
        pddl:precondition   [ a              pddl:And ;
                              pddl:argument  [ a  :on-robot ;
                                               :on-robot-b :drop-belt-b;
                                               :on-robot-rb :drop-belt-rb ] ;
                              pddl:argument  [ a         :belt-at ;
                                               :belt-at-cb :drop-belt-cb;
                                               :belt-at-x :drop-belt-x;
                                               :belt-at-y :drop-belt-y ] ;
                              pddl:argument  [ a         :robot-at ;
                                              :robot-at-rb :pick-shelf-rb;
                                              :robot-at-x  :pick-shelf-x;
                                              :robot-at-y  :pick-shelf-y ] ;
                            ] ;
        pddl:effect   [ a  pddl:And ;
                        pddl:argument  [ a  :on-belt ;
                                         :on-belt-b :drop-belt-b;
                                         :on-belt-cb :drop-belt-cb ] ;
                        pddl:argument  [ a pddl:Not ;
                                        pddl:argument  [ a  :on-robot ;
                                                         :on-robot-b :drop-belt-b;
                                                         :on-robot-rb :drop-belt-rb ] ] ;
                        pddl:argument [ a         :free-robot ;
                                         :free-robot-rb :drop-belt-rb ] ;
                      ] .


:drop-belt-rb  a             pddl:Parameter ;
        rdfs:label          "rb" ;
        pddl:type           :Robot ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            1 .

:drop-belt-b  a             pddl:Parameter ;
        rdfs:label          "b" ;
        pddl:type           :Box ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            2 .

:drop-belt-cb  a             pddl:Parameter ;
        rdfs:label          "cb" ;
        pddl:type           :ConveyorBelt ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            3 .

:drop-belt-x  a             pddl:Parameter ;
        rdfs:label          "x" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            4 .

:drop-belt-y  a             pddl:Parameter ;
        rdfs:label          "y" ;
        pddl:type           :Coord ;
        oslc:instanceShape  pddl:ParameterShape ;
        sh:order            5 .
        """.trimIndent()

        val model = OslcRdfHelper.modelFrom(ttl, Lang.TURTLE)
        return model
    }

    fun genProblem(): Model {
        val resources = HashSet<IResource>()
        val problem = Problem(u("scott-warehouse-problem"))
        resources.add(problem)

        problem.label = "scott-warehouse-problem"
        problem.domain = Link(u("scott-warehouse"))


        // OBJECTS

        val sh1 = shelf("sh1")
        val rb1 = robot("rb1")
        val cb1 = belt("cb1")
        val b1 = box("b1")

        val x0 = coord("x0")
        val y0 = coord("y0")
        val y10 = coord("y10")
        val y20 = coord("y20")

        val pddlObjects = hashSetOf(sh1, rb1, cb1, b1, x0, y0, y10, y20);
        problem.pddlObject.addAll(pddlObjects.map { it.link })
        resources += pddlObjects


        // INIT STATE

        val robotAt: InstanceWithResources<IExtendedResource> = robotAt(rb1, x0, y0)
        val shelfAt = shelfAt(sh1, x0, y10)
        val beltAt = beltAt(cb1, x0, y20)
        val onShelf = onShelf(b1, sh1)
        val freeRobot = freeRobot(rb1)


        val initObjects = hashSetOf(robotAt, shelfAt, beltAt, onShelf, freeRobot)
        problem.setInit(initObjects.map { it.instance.link }.toHashSet())
        resources += initObjects.flatMap { it.resources }

        // GOAL STATE

        val goal = and(onBelt(b1, cb1))
        problem.goal = goal.instance.link
        resources += goal.resources


        // misc

        val minFn = this.minFn()
        problem.minimize = minFn.link
        resources += minFn

        return JenaModelHelper.createJenaModel(resources.toArray())
    }


    private fun minFn(): RawResource {
        val minFn = RawResource(u(UUID.randomUUID()))
        minFn.addType(ns(PDDL, "total-time"))
        return minFn
    }

    private fun freeRobot(robot: IExtendedResource): InstanceWithResources<IExtendedResource> {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(u("free-robot"))
        r.setProperty(u("free-robot-rb"), robot.about)
        return InstanceWithResources(r, ImmutableSet.of(r, robot))
    }

    private fun and(
            vararg predicates: InstanceWithResources<IExtendedResource>): InstanceWithResources<IExtendedResource> {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(ns(PDDL, "And"))
        r.setProperty(ns(PDDL, "argument"), predicates.map { it.instance.about })
        val resources = HashSet(predicates.flatMap { it.resources })
        resources.add(r)
        return InstanceWithResources(r, resources)
    }

    private fun robotAt(robot: IExtendedResource, x: IExtendedResource,
                        y: IExtendedResource): InstanceWithResources<IExtendedResource> {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(u("robot-at"))
        r.setProperty(u("robot-at-rb"), robot.about)
        r.setProperty(u("robot-at-x"), x.about)
        r.setProperty(u("robot-at-y"), y.about)
        return InstanceWithResources(r, ImmutableSet.of(r, robot, x, y))
    }

    private fun beltAt(belt: IExtendedResource, x: IExtendedResource,
                       y: IExtendedResource): InstanceWithResources<IExtendedResource> {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(u("belt-at"))
        r.setProperty(u("belt-at-cb"), belt.about)
        r.setProperty(u("belt-at-x"), x.about)
        r.setProperty(u("belt-at-y"), y.about)
        return InstanceWithResources(r, ImmutableSet.of(r, belt, x, y))

    }

    private fun shelfAt(shelf: IExtendedResource, x: IExtendedResource,
                        y: IExtendedResource): InstanceWithResources<IExtendedResource> {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(u("shelf-at"))
        r.setProperty(u("shelf-at-sh"), shelf.about)
        r.setProperty(u("shelf-at-x"), x.about)
        r.setProperty(u("shelf-at-y"), y.about)
        return InstanceWithResources(r, ImmutableSet.of(r, shelf, x, y))
    }


    private fun onShelf(box: IExtendedResource,
                        shelf: IExtendedResource): InstanceWithResources<IExtendedResource> {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(u("on-shelf"))
        r.setProperty(u("on-shelf-b"), box.about)
        r.setProperty(u("on-shelf-sh"), shelf.about)
        return InstanceWithResources(r, ImmutableSet.of(r, shelf, box))
    }

    private fun onBelt(box: IExtendedResource,
                       belt: IExtendedResource): InstanceWithResources<IExtendedResource> {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(u("on-belt"))
        r.setProperty(u("on-belt-cb"), belt.about)
        r.setProperty(u("on-belt-b"), box.about)
        return InstanceWithResources(r, ImmutableSet.of(r, belt, box))
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

    fun coord(id: String): IExtendedResource {
        return pddlInstance(id, "Coord")
    }

    fun pddlInstance(id: String, type: String): IExtendedResource {
        val pddlResource = RawResource(u(id))
        pddlResource.setLabel(id)
        pddlResource.addType(u(type))

        return pddlResource
    }

    private fun buildInitState(tables: List<IResource>,
                               blocks: List<Block>): InstanceMultiWithResources<IResource> {
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
        val values: Collection<InstanceWithResources<IResource>> = blocks.map { b -> eq(moved(b), BigInteger.ZERO) }
        val totalMoved = RawResource()
        totalMoved.addType(u("total-moved"))
        val totalMovedEq = eq(totalMoved, BigInteger.ZERO)

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

        val initResourcesBuilder = ImmutableSet.builder<IResource>()
        initResourcesBuilder.addAll(values.map { v -> v.instance })
        initResourcesBuilder.addAll(onBlocks)
        initResourcesBuilder.addAll(clearBlocks)
        initResourcesBuilder.add(bONb)
        initResourcesBuilder.add(clearTable)
        initResourcesBuilder.add(totalMovedEq)

        val allResourcesBuilder = ImmutableSet.builder<IResource>()
        allResourcesBuilder.addAll(tables)
        allResourcesBuilder.addAll(blocks)
        allResourcesBuilder.addAll(values.flatMap { v -> v.resources })
        allResourcesBuilder.addAll(onBlocks)
        allResourcesBuilder.addAll(clearBlocks)
        allResourcesBuilder.add(bONb)
        allResourcesBuilder.add(clearTable)
        allResourcesBuilder.add(totalMoved)
        allResourcesBuilder.add(totalMovedEq)

        val initResources = initResourcesBuilder.build()
        val allResources = allResourcesBuilder.build()

        return InstanceMultiWithResources(initResources, allResources)
    }


    private fun buildGoalState(tables: ImmutableList<IResource>,
                               blocks: ImmutableList<Block>?): InstanceWithResources<IResource> {
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

        // FIXME Andrew@2018-08-16: one of the goals is empty when serialised
        val goal = Or(u(UUID.randomUUID()))

        val bONc = on(u("b"), u("c"))
        val cONb = on(u("c"), u("b"))
        goal.setArgument(hashSetOf(bONc.link, cONb.link))

        val instanceWithResources: InstanceWithResources<IResource> = InstanceWithResources(goal,
                ImmutableList.of(goal, bONc, cONb))
        return instanceWithResources
    }

    fun on(x: URI, y: URI): IResource {
        val r = RawResource(u(UUID.randomUUID()))
        r.addType(u("on"))
        r.setProperty(u("on-x"), x)
        r.setProperty(u("on-y"), y)
        return r
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

class RawResource(about: URI) : AbstractResource(about) {
    constructor() : this(u(UUID.randomUUID()))
}
