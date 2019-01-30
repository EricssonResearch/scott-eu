package eu.scott.warehouse.lib

import eu.scott.warehouse.domains.trs.TrsXConstants
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.io.ByteArrayInputStream
import java.io.StringWriter
import java.lang.reflect.InvocationTargetException
import javax.xml.datatype.DatatypeConfigurationException

/**
 * TODO
 *
 * @version $version-stub$
 * @since   0.0.1
 */
object MqttHelper {
    val log: Logger = LoggerFactory.getLogger(MqttHelper.javaClass)
    var QoS = 1

    fun msgFromResources(rdfFormat: RDFFormat,
                         vararg resources: IResource): MqttMessage {
        val model = jenaModelFrom(*resources)
        return msgFromModel(rdfFormat, model)
    }

    fun msgFromModel(rdfFormat: RDFFormat,
                     model: Model): MqttMessage {
        val rdfString = serialiseModel(model, rdfFormat)
        val mqttMessage = MqttMessage(rdfString.toByteArray())
        mqttMessage.qos = QoS
        return mqttMessage
    }

    fun serialiseModel(model: Model, rdfFormat: RDFFormat): String {
        val writer = StringWriter(1000)
        RDFDataMgr.write(writer, model, rdfFormat)
        return writer.toString()
    }

    fun jenaModelFrom(vararg objects: IResource): Model {
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

    fun extractModelFromMessage(message: MqttMessage): Model {
        val inputStream = ByteArrayInputStream(message.payload)
        val model = ModelFactory.createDefaultModel()
        RDFDataMgr.read(model, inputStream, TrsXConstants.rdfLang)
        return model
    }
}
