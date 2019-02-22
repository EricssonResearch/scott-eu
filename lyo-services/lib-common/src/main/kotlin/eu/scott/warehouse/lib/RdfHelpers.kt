package eu.scott.warehouse.lib

import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.apache.jena.riot.RDFParser
import org.apache.jena.util.ResourceUtils
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.io.StringWriter
import java.lang.reflect.InvocationTargetException
import javax.xml.datatype.DatatypeConfigurationException

/**
 * TODO
 *
 * @since   TODO
 */
object RdfHelpers {
    
    val log: Logger = LoggerFactory.getLogger(RdfHelpers::class.java)

    @JvmStatic
    @JvmOverloads
    fun modelToString(model: Model, rdfFormat: RDFFormat = RDFFormat.TURTLE_PRETTY): String {
        val writer = StringWriter(1000)
        RDFDataMgr.write(writer, model, rdfFormat)
        return writer.toString()
    }

    @JvmStatic
    fun modelFromResources(vararg objects: IResource): Model {
        try {
            return JenaModelHelper.createJenaModel(objects)
        } catch (e: DatatypeConfigurationException) {
            log.error("Shape definition error, check your annotations", e)
            throw IllegalStateException(e)
        } catch (e: IllegalAccessException) {
            throw IllegalStateException(e)
        } catch (e: InvocationTargetException) {
            throw IllegalStateException(e)
        } catch (e: OslcCoreApplicationException) {
            throw IllegalStateException(e)
        }
    }

    @JvmStatic
    fun modelFromIndentedString(str: String, l: Lang): Model {
        val model = ModelFactory.createDefaultModel()
        RDFParser.fromString(str.trimIndent()).lang(l).parse(model.graph)
        return model
    }

    /**
     * Skolemizes the model in-situ.
     */
    @JvmStatic
    fun skolemize(m: Model) {
        val resIterator = m.listSubjects()
        while (resIterator.hasNext()) {
            val resource = resIterator.nextResource()
            if (resource != null && resource.isAnon) {
                val skolemURI = "urn:skolem:" + resource.id.blankNodeId.labelString
                ResourceUtils.renameResource(resource, skolemURI)
            }
        }
    }





}
