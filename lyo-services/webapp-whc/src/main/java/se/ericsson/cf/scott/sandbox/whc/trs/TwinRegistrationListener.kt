package se.ericsson.cf.scott.sandbox.whc.trs

import eu.scott.warehouse.lib.MqttHelper
import eu.scott.warehouse.lib.MqttTopics
import eu.scott.warehouse.domains.trs.TrsServerAck
import eu.scott.warehouse.domains.trs.TrsServerAnnouncement
import eu.scott.warehouse.domains.trs.TrsXConstants
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.net.URI
import java.util.concurrent.ConcurrentHashMap

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
class TwinRegistrationListener(val mqttClient: MqttClient,
                               val mqttTopic: String): IMqttMessageListener {
    private val log: Logger = LoggerFactory.getLogger(javaClass)
    val executors: MutableMap<String, URI> = ConcurrentHashMap()
    val twins: MutableMap<String, URI> = ConcurrentHashMap()

    override fun messageArrived(topic: String?, message: MqttMessage?) {
        if (message != null) {
            val model = MqttHelper.extractModelFromMessage(message)
            val annmt = JenaModelHelper.unmarshalSingle(
                    model,
                    TrsServerAnnouncement::class.java
            )
            if (!annmt.isLeaving) {
                log.info("New TRS server announcement from {}: {}", annmt.adaptorId, annmt.trsUri)
                registerInternal(annmt)
                val trsServerAck = TrsServerAck(annmt.adaptorId, mqttTopic)
                log.debug("ACKing {}", annmt.adaptorId)
                sendAck(trsServerAck)
            } else {
                if (TrsXConstants.TYPE_TWIN.equals(annmt.kind)) {
                    unregister(annmt, twins)
                    log.info("There are {} twins registered", twins.size)
                } else if (TrsXConstants.TYPE_EXECUTOR.equals(annmt.kind)) {
                    unregister(annmt, executors)
                    log.info("There are {} executors registered", executors.size)
                } else {
                    log.warn("Cannot unregister; adaptor kind cannot be determined: {}", annmt)
                }
            }
        } else {
            log.warn("An empty message has arrived")
        }
    }

    private fun unregister(annmt: TrsServerAnnouncement, mutableMap: MutableMap<String, URI>) {
        if (mutableMap.containsKey(annmt.adaptorId)) {
            mutableMap.remove(annmt.adaptorId)
            log.info("Adaptor {} was unregistered", annmt.adaptorId)
        } else {
            log.warn("Adaptor {} un-registration is noop - adaptor was not registered before",
                    annmt.adaptorId)
        }
    }

    private fun sendAck(trsServerAck: TrsServerAck) {
        val mqttMessage = MqttHelper.msgFromResources(TrsXConstants.rdfFormat, trsServerAck)
        mqttClient.publish(MqttTopics.REGISTRATION_ACK, mqttMessage)
    }

    private fun registerInternal(annmt: TrsServerAnnouncement) {
        log.trace(dumpAnnouncement(annmt))
        if (TrsXConstants.TYPE_TWIN.equals(annmt.kind)) {
            twins[annmt.adaptorId] = annmt.trsUri.value
            log.info("There are {} twins registered", twins.size)
        } else if (TrsXConstants.TYPE_EXECUTOR.equals(annmt.kind)) {
            executors[annmt.adaptorId] = annmt.trsUri.value
            log.info("There are {} executors registered", executors.size)
        } else {
            log.warn("Announcement kind cannot be determined: {}", annmt)
        }
    }

    private fun dumpAnnouncement(annmt: TrsServerAnnouncement): String? {
        return annmt.toString()
    }
}
