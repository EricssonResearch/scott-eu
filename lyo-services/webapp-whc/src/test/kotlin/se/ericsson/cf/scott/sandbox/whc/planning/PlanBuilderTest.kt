package se.ericsson.cf.scott.sandbox.whc.planning

import eu.scott.warehouse.domains.pddl.PrimitiveType
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.apache.jena.riot.RDFParser
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.junit.Assert.*
import org.junit.Test
import java.io.ByteArrayOutputStream
import java.net.URI

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
class PlanBuilderTest {

    @Test
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

        val location = PlanBuilder().buildLocation()
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

    private fun modelFrom(str: String, l: Lang): Model {
        val model = ModelFactory.createDefaultModel()
        RDFParser.fromString(str.trimIndent()).lang(l).parse(model.graph)
        return model
    }
}

val Model.turtle: String
    get() {
        val outputStream = ByteArrayOutputStream()
        RDFDataMgr.write(outputStream, this, RDFFormat.TURTLE_PRETTY)
        return outputStream.toString()
    }
