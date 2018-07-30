package se.ericsson.cf.scott.sandbox.whc.trs

import eu.scott.warehouse.MqttHelper
import eu.scott.warehouse.MqttTopics
import eu.scott.warehouse.domains.trs.TrsServerAck
import eu.scott.warehouse.domains.trs.TrsServerAnnouncement
import eu.scott.warehouse.domains.trs.TrsXConstants
import org.apache.jena.riot.RDFFormat
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.net.URI

/**
 * TBD
 *
 * @version $version-stub$
 * @since   FIXME
 */
class TwinRegistrationListener(val twins: MutableMap<String, URI>, val mqttClient: MqttClient,
                               val mqttTopic: String): IMqttMessageListener {
    private val log: Logger = LoggerFactory.getLogger(javaClass)
    override fun messageArrived(topic: String?, message: MqttMessage?) {
        if (message != null) {
            val model = MqttHelper.extractModelFromMessage(message)
            val annmt = JenaModelHelper.unmarshalSingle(
                    model,
                    TrsServerAnnouncement::class.java
            )
            if (!annmt.isLeaving) {
                log.info("New TRS server announcement from {}: {}", annmt.twinId, annmt.trsUri)
                registerInternal(annmt)
                val trsServerAck = TrsServerAck(annmt.twinId, mqttTopic)
                log.debug("Sending an ACK")
                sendAck(trsServerAck)
            } else {
                if(twins.containsKey(annmt.twinId)) {
                    twins.remove(annmt.twinId)
                    log.info("Twin {} was unregistered", annmt.twinId)
                } else {
                    log.warn("Twin {} un-registration is noop - twin was not registered before",
                            annmt.twinId)
                }
            }
        } else {
            log.warn("An empty message has arrived")
        }
    }

    private fun sendAck(trsServerAck: TrsServerAck) {
        val mqttMessage = MqttHelper.msgFromResources(TrsXConstants.rdfFormat, trsServerAck)
        mqttClient.publish(MqttTopics.REGISTRATION_ACK, mqttMessage)
    }

    private fun registerInternal(annmt: TrsServerAnnouncement) {
        twins[annmt.twinId] = annmt.trsUri.value
        log.info("There are {} twins registered", twins.size)
    }
}
