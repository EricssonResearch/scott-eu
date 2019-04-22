package eu.scott.warehouse.lib

import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.oslc4j.core.annotation.OslcName
import org.eclipse.lyo.oslc4j.core.annotation.OslcNamespace
import org.eclipse.lyo.oslc4j.core.annotation.OslcResourceShape
import org.eclipse.lyo.oslc4j.core.model.AbstractResource
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.net.URI
import javax.xml.namespace.QName

/**
 * REALLY useful method
 */
val IResource.link: Link
    get() {
        return Link(this.about)
    }

val IResource.toTurtleString: String
    get() {
        val model = RdfHelpers.modelFromResources(this)
        return RdfHelpers.modelToString(model)
    }

/**
 * I think the QName part is XML heritage
 */
fun IExtendedResource.setProperty(name: String, p: Any) {
    this.extendedProperties[QName.valueOf(name)] = p
}

fun IExtendedResource.setProperty(name: URI, p: Any) = setProperty(name.toString(), p)

fun IExtendedResource.setInstanceShape(cl: Class<*>) = this.setProperty(OslcHelpers.ns(OslcHelpers.OSLC, "instanceShape"), OslcHelpers.nsSh(cl))

fun IExtendedResource.setSuperclass(cl: Class<*>) = this.setProperty(OslcHelpers.ns(OslcHelpers.RDFS, "subClassOf"), OslcHelpers.ns(cl))

fun IExtendedResource.setLabel(l: String) = this.setProperty(OslcHelpers.ns(OslcHelpers.RDFS, "label"), l)

class RawResource(about: URI) : AbstractResource(about)


object OslcHelpers {

    val log: Logger = LoggerFactory.getLogger(OslcHelpers::class.java)

    const val RDFS = "http://www.w3.org/2000/01/rdf-schema#"
    const val PDDL = "http://ontology.cf.ericsson.net/pddl/"
    const val OSLC = "http://open-services.net/ns/core#"
    const val SH_ORDER = "http://www.w3.org/ns/shacl#order"

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

    // TODO Andrew@2018-02-23: move to JMH
    // TODO Andrew@2018-02-23: create a stateful JMH that would keep resources hashed by URI
    @Throws(LyoJenaModelException::class)
    @JvmStatic
    private fun <R : IResource> nav(m: Model, l: Link, rClass: Class<R>): R {
        val rs = JenaModelHelper.unmarshal(m, rClass)
        for (r in rs) {
            if (l.value == r.about) {
                return r
            }
        }
        throw IllegalArgumentException("Link cannot be followed in this model")
    }

    // TODO Andrew@2019-02-21: make sure we catch all exceptions here
    @SafeVarargs
    @JvmStatic
    fun navTry(m: Model, l: Link, vararg rClass: Class<out IResource>): IResource {
        for (aClass in rClass) {
            try {
                return nav(m, l, aClass)
            } catch (e: IllegalArgumentException) {
                log.warn("Fix RDFS reasoning in JMH!!!")
            }

        }
        // give up
        throw IllegalArgumentException("Link ${l.value} cannot be followed in this model")
    }

    @SafeVarargs
    @JvmStatic
    fun navTry(m: Model, l: Link, vararg rClass: Class<out IExtendedResource>): IExtendedResource{
        for (aClass in rClass) {
            try {
                return nav(m, l, aClass)
            } catch (e: IllegalArgumentException) {
                log.warn("Fix RDFS reasoning in JMH!!!")
            }

        }
        // give up
        throw IllegalArgumentException("Link cannot be followed in this model")
    }


    @JvmStatic
    fun hexHashCodeFor(aResource: IResource): String {
        return Integer.toHexString(aResource.hashCode())
    }
}
