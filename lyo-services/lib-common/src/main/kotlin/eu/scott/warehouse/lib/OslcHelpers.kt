package eu.scott.warehouse.lib

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
import java.net.URI
import java.util.UUID
import javax.ws.rs.core.UriBuilder
import javax.xml.namespace.QName

/**
 * TODO
 *
 * @since   TODO
 */


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

fun IExtendedResource.setProperty(name: URI, p: Any) = setProperty(name.toString(), p)

fun IExtendedResource.setInstanceShape(cl: Class<*>) = this.setProperty(OslcRdfHelper.ns(OslcRdfHelper.OSLC, "instanceShape"), OslcRdfHelper.nsSh(cl))

fun IExtendedResource.setSuperclass(cl: Class<*>) = this.setProperty(OslcRdfHelper.ns(OslcRdfHelper.RDFS, "subClassOf"), OslcRdfHelper.ns(cl))

fun IExtendedResource.setLabel(l: String) = this.setProperty(OslcRdfHelper.ns(OslcRdfHelper.RDFS, "label"), l)

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
