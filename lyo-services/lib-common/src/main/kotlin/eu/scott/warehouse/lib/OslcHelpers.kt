/*
 * Copyright (c) 2019 Ericsson Research and others
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package eu.scott.warehouse.lib

import org.apache.jena.rdf.model.Model
import org.apache.jena.vocabulary.RDF
import org.eclipse.lyo.oslc4j.core.annotation.OslcName
import org.eclipse.lyo.oslc4j.core.annotation.OslcNamespace
import org.eclipse.lyo.oslc4j.core.annotation.OslcResourceShape
import org.eclipse.lyo.oslc4j.core.model.AbstractResource
import org.eclipse.lyo.oslc4j.core.model.IExtendedResource
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
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

val IResource.toTurtle: String
    get() {
        val model = RdfHelpers.modelFromResources(this)
        return RdfHelpers.modelToString(model)
    }

val IResource.toModel: Model
    get() {
        return JenaModelHelper.createJenaModel(arrayOf(this))
    }

val Model.toTurtle: String?
    get() {
        return RdfHelpers.modelToString(this)
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

    @SafeVarargs
    @JvmStatic
    fun navTry(m: Model, l: Link, vararg rClass: Class<out IExtendedResource>): IExtendedResource {
        if(l.value == null) {
            throw IllegalArgumentException("Link $l has no URI value")
        }
        if(rClass.size > 1) {
            log.debug("Fix RDFS reasoning in JMH!!!")
        }
        for (aClass in rClass) {
            try {
                val resource = nav(m, l, aClass)
                log.debug("Found an instance of ${aClass.simpleName}")
                return resource
            } catch (e: IllegalArgumentException) {}
        }

        // give up
        if(log.isTraceEnabled) {
            val builder = StringBuilder()
            builder.appendln("Resource with URI ${l.value} was not found in the Model:")
            val resources = m.listSubjectsWithProperty(
                org.apache.jena.vocabulary.RDF.type)
            resources.forEach {
                val property = it.getProperty(org.apache.jena.vocabulary.RDF.type).`object`
                builder.appendln("\t${it.nameSpace}:${it.localName} a $property")
            }
            log.trace(builder.toString())
        }

        throw IllegalArgumentException("Link ${l.value} cannot be followed in this model")
    }

    @JvmStatic
    fun <T : IExtendedResource> findSubclasses(rClass: Class<T>,
                                               m: Model): Set<T> {
        val objects = HashSet<T>()
        for (r in m.listResourcesWithProperty(RDF.type)) {
            try {
                val lyoOjbect: T = JenaModelHelper.unmarshal(r, rClass)
                objects.add(lyoOjbect)
            } catch (e: Throwable) {
                log.trace("${r.localName} is not an instance of ${rClass.simpleName}")
            }
        }

        if(objects.isNotEmpty()) {
            return objects
        }

        throw IllegalArgumentException("There are no subclasses of ${rClass.simpleName} in this model")
    }


    @JvmStatic
    fun <T : IExtendedResource> follow(l: Link, rClass: Class<T>, m: Model): T {
        val resource = m.getResource(l.value.toString())
        val obj = JenaModelHelper.unmarshal(resource, rClass)
        return obj
    }

    @JvmStatic
    fun hexHashCodeFor(aResource: IResource): String {
        return Integer.toHexString(aResource.hashCode())
    }
}
