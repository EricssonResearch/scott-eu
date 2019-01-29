package se.ericsson.cf.scott.sandbox.twin.trs

import eu.scott.warehouse.lib.ChangeEventMqttMessageListener
import eu.scott.warehouse.lib.LastWillMessage
import eu.scott.warehouse.lib.MqttHelper
import eu.scott.warehouse.lib.MqttTopics
import eu.scott.warehouse.lib.SimpleAckRegistrationAgent
import eu.scott.warehouse.lib.TrsMqttGateway
import eu.scott.warehouse.lib.PlanChangeEventListener
import eu.scott.warehouse.domains.trs.TrsServerAck
import eu.scott.warehouse.domains.trs.TrsServerAnnouncement
import eu.scott.warehouse.domains.trs.TrsXConstants
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttException
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.twin.TwinAdaptorHelper
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import javax.ws.rs.core.UriBuilder

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
class TwinAckRegistrationAgent(val whcTopic: String) :
        SimpleAckRegistrationAgent<TrsServerAnnouncement, TrsServerAck>() {
    private val log = LoggerFactory.getLogger(javaClass)!!
    private lateinit var trsTopic: String

    override fun register(gateway: TrsMqttGateway) {
        val latch = CountDownLatch(1)
        try {
            gateway.subscribe(MqttTopics.REGISTRATION_ACK,
                    IMqttMessageListener() { topic, message ->
                        completeRegistration(message, latch, gateway)
                    })
        } catch (e: MqttException) {
            log.error("Something went wrong with the REG_ACK subscription", e)
        }

        try {
            while (latch.count > 0) {
                log.trace("Posting the registration message")
                gateway.publish(MqttTopics.REGISTRATION_ANNOUNCE,
                        getTwinRegistrationMessage(gateway.clientId))
                latch.await(5, TimeUnit.SECONDS)
                if (latch.count > 0) {
                    log.warn("Failed to register the twin with the WHC, attempting again")
                }
            }
        } catch (e: MqttException) {
            log.error("Failed to publish the twin registration message")
            log.debug("Failed to publish the twin registration message", e)
        } catch (e: InterruptedException) {
            log.error("The program was interrupted before the latch reached zero")
        }
    }

    override fun unregister(gateway: TrsMqttGateway) {
        gateway.publish(MqttTopics.REGISTRATION_ANNOUNCE, lastWill.message)
    }

    override val lastWill: LastWillMessage
        get() {
            val trsUri = UriBuilder.fromUri(OSLC4JUtils.getServletURI()).path("trs").build()
            val announcement = TrsServerAnnouncement(
                TwinAdaptorHelper.getTwinUUID(),
                    TrsXConstants.TYPE_TWIN, trsUri, MqttTopics.REGISTRATION_ANNOUNCE, true)
            val model = MqttHelper.jenaModelFrom(announcement)
            return LastWillMessage(whcTopic, model)
        }


    private fun getTwinRegistrationMessage(id: String): MqttMessage {
        val trsUri = UriBuilder.fromUri(OSLC4JUtils.getServletURI()).path("trs").build()
        val announcement = TrsServerAnnouncement(
            TwinAdaptorHelper.getTwinUUID(),
                TrsXConstants.TYPE_TWIN, trsUri, MqttTopics.REGISTRATION_ANNOUNCE, false)
        return MqttHelper.msgFromResources(TrsXConstants.rdfFormat, announcement)
    }


    private fun completeRegistration(message: MqttMessage, latch: CountDownLatch,
                                     gateway: TrsMqttGateway) {
        val id = gateway.clientId
        val model = MqttHelper.extractModelFromMessage(message)
        try {
            val serverAck = JenaModelHelper.unmarshalSingle(model, TrsServerAck::class.java)
            if (id == serverAck.adaptorId) {
                log.debug("The WHC registration of the {} has been confirmed", id)
                trsTopic = serverAck.trsTopic
                log.info("Using the '{}' topic to monitor new plans", trsTopic)
                subscribeToPlans(trsTopic, gateway)
                latch.countDown()
            }
        } catch (e: LyoJenaModelException) {
            log.error("Error unmarshalling TrsServerAck from the server response", e)
        }
    }

    private fun subscribeToPlans(trsTopic: String, gateway: TrsMqttGateway) {
        gateway.subscribe(trsTopic, ChangeEventMqttMessageListener(
            PlanChangeEventListener(gateway.executorService)))
    }


}
