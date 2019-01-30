package eu.scott.warehouse.lib

import eu.scott.warehouse.domains.trs.TrsXConstants
import org.apache.jena.rdf.model.Model

data class LastWillMessage(val topic: String, val message: Model) {
    val messageBytes: ByteArray
        get() = MqttHelper.msgFromModel(TrsXConstants.rdfFormat, message).payload
}

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
interface RegistrationAgent {
    val lastWill: LastWillMessage?
    fun register(gateway: TrsMqttGateway)
    fun unregister(gateway: TrsMqttGateway)
}

// R for request, A for ack
interface AckRegistrationAgent<R, A> : RegistrationAgent {
}

// TODO Andrew@2018-08-03: extract the latch logic from ExecutorAckRegistrationAgent
/**
 * The simplest registration that sends the un-registration message to a special topic
 */
abstract class SimpleAckRegistrationAgent<R, A> : AckRegistrationAgent<R, A>


/**
 * The registration that allows to notify multiple nodes with an LWT message by setting up its own topic.
 */
abstract class TopicAckRegistrationAgent<R, A> : AckRegistrationAgent<R, A>
