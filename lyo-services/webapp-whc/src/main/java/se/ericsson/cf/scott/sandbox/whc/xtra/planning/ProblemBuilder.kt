package se.ericsson.cf.scott.sandbox.whc.xtra.planning

import eu.scott.warehouse.domains.pddl.Problem
import eu.scott.warehouse.lib.InstanceWithResources
import eu.scott.warehouse.lib.OslcHelpers
import eu.scott.warehouse.lib.link
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.net.URI
import java.util.LinkedList

interface Label {
    val value: String
}

data class RobotLabel(override val value: String) : Label
data class ShelfLabel(override val value: String) : Label
data class BeltLabel(override val value: String) : Label
data class BoxLabel(override val value: String) : Label

/**
 * TODO
 *
 * @since   TODO
 */
class ProblemBuilder {

    companion object {
        val log: Logger = LoggerFactory.getLogger(ProblemBuilder::class.java)
    }

    private var width: Int = -1
    private var height: Int = -1

    // TODO Andrew@2019-02-19: use typed labels
    private val robots: MutableSet<String> = HashSet()

    // TODO Andrew@2019-02-19: use typed labels
    // TODO Andrew@2019-02-19: use se.ericsson.cf.scott.sandbox.whc.xtra.planning.PlanRequestHelper.freeRobot
    private val freeRobots: MutableSet<String> = HashSet()

    private val initPositions: MutableMap<Pair<Int, Int>, Label> = HashMap()
    private val goalPositions: MutableMap<Pair<Int, Int>, Label> = HashMap()

    private val boxOnShelfInit: MutableSet<Pair<BoxLabel, ShelfLabel>> = HashSet()
    private val boxOnBeltGoal: MutableSet<Pair<BoxLabel, BeltLabel>> = HashSet()

    private val resources: MutableSet<IExtendedResource> = LinkedHashSet()

    private lateinit var problem: Problem

    fun build(baseURI: URI): ProblemRequestState {
        if (width == 0 || height == 0) {
            throw IllegalArgumentException("Width and height must be set")
        }

        val resources: MutableSet<IExtendedResource> = resources

        // TODO Andrew@2019-02-20: make it a class ctor param
        OslcHelpers.setBase(baseURI)

        problem = Problem(OslcHelpers.u("scott-warehouse-problem"))
        problem.label = "scott-warehouse-problem"
        problem.domain = Link(OslcHelpers.u("scott-warehouse"))

        resources.add(problem)

        buildObjects()
        buildInitState()
        buildGoalState()
        buildMinimisationFn()

        val model = JenaModelHelper.createJenaModel(resources.toTypedArray())
        return ProblemRequestState(model)

        TODO("not implemented")
    }

    private fun buildMinimisationFn() {
        val minFn = PlanRequestHelper.minFn()
        problem.minimize = minFn.link
        resources += minFn
    }

    private fun buildObjects() {
        // Shop floor
        for (i in 1..width) {
            val xCoord = PlanRequestHelper.coord("x$i")
            addObject(xCoord)
        }
        for (i in 1..height) {
            val yCoord = PlanRequestHelper.coord("y$i")
            addObject(yCoord)
        }

        // Stationary objects
        initPositions.forEach { coord, label ->
            when (label) {
                is ShelfLabel -> addObject(PlanRequestHelper.shelf(label.value))
                is BeltLabel  -> addObject(PlanRequestHelper.belt(label.value))
            }
        }

        // Robots
        robots.forEach { label ->
            val robot = PlanRequestHelper.robot(label)
            addObject(robot)
        }

        freeRobots.forEach { label: String ->
            val freeRobot = PlanRequestHelper.freeRobot(lookupRobot(label))
            addObject(freeRobot)
        }

        boxOnShelfInit.forEach { pair: Pair<BoxLabel, ShelfLabel> ->
            val box = PlanRequestHelper.box(pair.first.value)
            addObject(box)
        }
    }

    private fun buildInitState() {
        initPositions.forEach { coord, label ->
            val x = lookupCoordX(coord.first)
            val y = lookupCoordY(coord.second)
            val objAtXY: InstanceWithResources<IExtendedResource> = when (label) {
                is ShelfLabel -> PlanRequestHelper.shelfAt(lookupShelf(label.value), x, y)
                is BeltLabel  -> PlanRequestHelper.beltAt(lookupBelt(label.value), x, y)
                is RobotLabel -> PlanRequestHelper.robotAt(lookupRobot(label.value), x, y)
                else          -> throw IllegalStateException("Unexpected label type")
            }
            addInit(objAtXY)
        }

        boxOnShelfInit.forEach { pair: Pair<BoxLabel, ShelfLabel> ->
            val box = lookupBox(pair.first.value)
            val shelf = lookupShelf(pair.second.value)
            val onShelf = PlanRequestHelper.onShelf(box, shelf)
            addInit(onShelf)
        }
    }

    private fun buildGoalState() {
        //
        //        // GOAL STATE
        //
        //        val goal = and(onBelt(b1, cb1))
        //        problem.goal = goal.instance.link
        //        resources += goal.resources

        val goalPredicates: MutableList<InstanceWithResources<IExtendedResource>> = LinkedList()

        val onBeltPredicates = boxOnBeltGoal.map { pair: Pair<BoxLabel, BeltLabel> ->
            PlanRequestHelper.onBelt(lookupBox(pair.first.value), lookupBelt(pair.second.value))
        }
        goalPredicates.addAll(onBeltPredicates)


        goalPositions.forEach { coord, label ->
            when (label) {
                is RobotLabel -> {
                    val robot = lookupRobot(label.value)
                    val x = lookupCoordX(coord.first)
                    val y = lookupCoordY(coord.second)
                    val robotAt = PlanRequestHelper.robotAt(robot, x, y)

                    goalPredicates.add(robotAt)
                }
            }
        }


        // just a conjunction of all terms
        val andGoal = PlanRequestHelper.and(goalPredicates)

        // no need for an addGoal() function
        problem.goal = andGoal.instance.link
        resources += andGoal.resources
    }

    private fun addObject(resource: IExtendedResource) {
        problem.pddlObject.add(resource.link)
        resources += resource
    }


    private fun addObject(complexResource: InstanceWithResources<IExtendedResource>) {
        problem.pddlObject.add(complexResource.instance.link)

        resources += complexResource.instance
        resources += complexResource.resources
    }

    private fun addInit(resource: IExtendedResource) {
        problem.init.add(resource.link)
        resources += resource
    }


    private fun addInit(complexResource: InstanceWithResources<IExtendedResource>) {
        problem.init.add(complexResource.instance.link)

        resources += complexResource.instance
        resources += complexResource.resources
    }

    private fun lookupCoordX(coord: Int): IExtendedResource {
        return lookupNaive("x$coord")
    }

    private fun lookupCoordY(coord: Int): IExtendedResource {
        return lookupNaive("y$coord")
    }

    private fun lookupShelf(label: String): IExtendedResource {
        return lookupNaive(label)
    }

    private fun lookupBelt(label: String): IExtendedResource {
        return lookupNaive(label)
    }

    private fun lookupRobot(label: String): IExtendedResource {
        return lookupNaive(label)
    }

    private fun lookupBox(label: String): IExtendedResource {
        return lookupNaive(label)
    }

    private fun lookupNaive(label: String): IExtendedResource {
        return resources.single { r -> r.about == OslcHelpers.u(label) }
    }

    fun warehouseSize(_width: Int, _height: Int): ProblemBuilder {
        if (_width <= 0 || _height <= 0) {
            throw IllegalArgumentException("Width & height must be within (0, INT_MAX]")
        }
        this.width = _width
        this.height = _height
        return this
    }

    fun robotsActive(labels: Collection<String>): ProblemBuilder {
        robots.addAll(labels)
        freeRobots.addAll(labels)
        return this
    }

    fun robotsInactive(labels: Collection<String>): ProblemBuilder {
        if (freeRobots.intersect(labels).isNotEmpty()) {
            throw IllegalArgumentException("Active & inactive robots shall not overlap")
        }
        robots.addAll(labels)
        return this
    }

    fun robotAtInit(label: String, x: Int, y: Int): ProblemBuilder {
        if (initPositions.putIfAbsent(Pair(x, y), RobotLabel(label)) != null) {
            throw IllegalArgumentException("Waypoint ($x, $y) is already occupied")
        }
        return this
    }

    fun robotAtGoal(label: String, x: Int, y: Int): ProblemBuilder {
        if (goalPositions.putIfAbsent(Pair(x, y), RobotLabel(label)) != null) {
            throw IllegalArgumentException(
                "Waypoint ($x, $y) is already planned to be occupied (goal)")
        }
        return this
    }

    fun shelfAt(label: String, x: Int, y: Int): ProblemBuilder {
        if (initPositions.putIfAbsent(Pair(x, y), ShelfLabel(label)) != null) {
            throw IllegalArgumentException("Waypoint ($x, $y) is already occupied")
        }
        return this
    }

    fun beltAt(label: String, x: Int, y: Int): ProblemBuilder {
        if (initPositions.putIfAbsent(Pair(x, y), BeltLabel(label)) != null) {
            throw IllegalArgumentException("Waypoint ($x, $y) is already occupied")
        }
        return this
    }

    fun boxOnShelfInit(boxLabel: String, shelfLabel: String): ProblemBuilder {
        boxOnShelfInit.add(Pair(BoxLabel(boxLabel), ShelfLabel(shelfLabel)))
        return this
    }

    fun boxOnBeltGoal(boxLabel: String, beltLabel: String): ProblemBuilder {
        boxOnBeltGoal.add(Pair(BoxLabel(boxLabel), BeltLabel(beltLabel)))
        return this
    }

}
