package se.ericsson.cf.scott.sandbox.whc.planning

import eu.scott.warehouse.domains.pddl.PrimitiveType
import eu.scott.warehouse.lib.OslcRdfHelper.modelFrom
import org.apache.jena.rdf.model.Model
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.junit.Assert.assertTrue
import org.junit.Ignore
import org.junit.Test
import java.io.ByteArrayOutputStream
import java.net.URI

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
class PlanRequestBuilderTest {

    @Test
    fun outputTurtleForManualTesting() {
        val planRequestComplete = PlanRequestBuilder().getPlanRequestComplete()
        println(planRequestComplete.turtle)
    }

    @Test
    // TODO Andrew@2018-08-28: update the expectation in the unit test
    @Ignore("shall be updated from blocksworld to the scott-warehouse PDDL domain")
    fun locationIsIsomorphic() {
        val locationModelExpected: Model = modelFrom("""
            @prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
            @prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> .
            @prefix oslc: <http://open-services.net/ns/core#> .
            @prefix pddl: <http://ontology.cf.ericsson.net/pddl/> .
            @prefix : <http://ontology.cf.ericsson.net/pddl_example/> .

            :location
              a rdfs:Class ;
              rdfs:subClassOf pddl:PrimitiveType ;
              oslc:instanceShape pddl:PrimitiveTypeShape ;
              rdfs:label "location" .
        """, Lang.TURTLE)

        val location = PlanRequestBuilder().buildLocation()
        val locationModelActual = JenaModelHelper.createJenaModel(arrayOf(location))

        assertIsomorphic("Location object is incorrect", locationModelExpected, locationModelActual)
    }

    @Test
    fun resourceURI() {
        val primitiveType = PrimitiveType(URI.create("urn:test1"))
        val model = JenaModelHelper.createJenaModel(arrayOf(primitiveType))
        println(model.turtle)
    }

    private fun assertIsomorphic(s: String, locationModelExpected: Model,
                                 locationModelActual: Model?) {
        val expected = locationModelExpected.turtle
        val actual = locationModelActual?.turtle ?: "<null>"
        assertTrue("$s:\n$actual\n***\n$expected",
                locationModelExpected.isIsomorphicWith(locationModelActual))
    }


}

val Model.turtle: String
    get() {
        val outputStream = ByteArrayOutputStream()
        RDFDataMgr.write(outputStream, this, RDFFormat.TURTLE_PRETTY)
        return outputStream.toString()
    }
