package eu.scott.warehouse.lib

import eu.scott.warehouse.domains.trs.TrsXConstants
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.io.ByteArrayInputStream


object MqttHelper {
    val log: Logger = LoggerFactory.getLogger(MqttHelper.javaClass)
    var QoS = 1

    fun msgFromResources(rdfFormat: RDFFormat,
                         vararg resources: IResource): MqttMessage {
        val model = RdfHelpers.modelFromResources(*resources)
        return msgFromModel(rdfFormat, model)
    }

    fun msgFromModel(rdfFormat: RDFFormat,
                     model: Model): MqttMessage {
        val rdfString = RdfHelpers.modelToString(model, rdfFormat)
        val mqttMessage = MqttMessage(rdfString.toByteArray())
        mqttMessage.qos = QoS
        return mqttMessage
    }


    fun extractModelFromMessage(message: MqttMessage): Model {
        val inputStream = ByteArrayInputStream(message.payload)
        val model = ModelFactory.createDefaultModel()
        RDFDataMgr.read(model, inputStream, TrsXConstants.rdfLang)
        return model
    }
}
