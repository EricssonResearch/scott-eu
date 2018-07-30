package eu.scott.warehouse

import eu.scott.warehouse.domains.trs.TrsXConstants
import org.apache.jena.rdf.model.Model

data class LastWillMessage(val topic: String, val message: Model) {
    val messageBytes: ByteArray
        get() = MqttHelper.msgFromModel(TrsXConstants.rdfFormat, message).payload
}

/**
 * TBD
 *
 * @version $version-stub$
 * @since   FIXME
 */
interface RegistrationAgent {
    val lastWill: LastWillMessage?
    fun register(gateway: TrsMqttGateway)
    fun unregister(gateway: TrsMqttGateway)
}

// R for request, A for ack
interface AckRegistrationAgent<R, A> : RegistrationAgent {
}

abstract class SimpleAckRegistrationAgent<R, A>() :
        AckRegistrationAgent<R, A> {
}
