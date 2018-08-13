package se.ericsson.cf.scott.sandbox.whc.planning

import eu.scott.warehouse.domains.pddl.Domain
import eu.scott.warehouse.domains.pddl.PrimitiveType
import org.eclipse.lyo.oslc4j.core.annotation.OslcName
import org.eclipse.lyo.oslc4j.core.annotation.OslcNamespace
import org.eclipse.lyo.oslc4j.core.annotation.OslcResourceShape
import org.eclipse.lyo.oslc4j.core.model.AbstractResource
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.core.model.Link
import java.net.URI
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

class RawResource(about: URI?) : AbstractResource(about) {

}

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
class PlanBuilder {
    private val base: URI = URI.create("http://ontology.cf.ericsson.net/pddl_example/")
    private val RDFS = "http://www.w3.org/2000/01/rdf-schema#"
    private val PDDL = "http://ontology.cf.ericsson.net/pddl/"
    private val OSLC = "http://open-services.net/ns/core#"

    fun build(): Array<Any> {

        val location = buildLocation()

        /*
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
         */

        val adl = Domain(u("adl-blocksworld"))

        adl.label = "adl-blocksworld"
        // FIXME must be a set
//        adl.type = Sets.of(location, block)
        adl.type = location.link

        return arrayOf()
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


    /**
     * URI of the Resource
     */
    private fun ns(clazz: Class<*>): URI {
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
    private fun nsSh(clazz: Class<*>): URI {
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
    private fun ns(ns: String, name: String): URI {
        return URI.create(ns + name)
    }

    

    fun u(p: String): URI? {
        return UriBuilder.fromUri(base).path(p).build()
    }
}
