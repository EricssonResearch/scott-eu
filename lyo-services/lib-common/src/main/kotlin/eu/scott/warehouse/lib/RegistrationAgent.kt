package eu.scott.warehouse.lib

import eu.scott.warehouse.domains.trs.TrsXConstants
import org.apache.jena.rdf.model.Model

data class LastWillMessage(val topic: String, val message: Model) {
    val messageBytes: ByteArray
        get() = MqttTrsServices.msgFromModel(message).payload
}
