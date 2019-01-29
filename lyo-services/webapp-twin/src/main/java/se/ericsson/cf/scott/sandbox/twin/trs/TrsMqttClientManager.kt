package se.ericsson.cf.scott.sandbox.twin.trs

import eu.scott.warehouse.lib.LoggingMqttCallback
import eu.scott.warehouse.lib.PlanChangeEventListener
import eu.scott.warehouse.lib.ChangeEventMqttMessageListener
import eu.scott.warehouse.lib.MqttHelper
import eu.scott.warehouse.lib.MqttTopics
import eu.scott.warehouse.domains.trs.*
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttException
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.twin.TwinAdaptorHelper
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import javax.ws.rs.core.UriBuilder

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
class TrsMqttClientManager() {
    private val log = LoggerFactory.getLogger(javaClass)
    private val executorService: ScheduledExecutorService = Executors.newScheduledThreadPool(32)
    private lateinit var mqttClient: MqttClient
    private lateinit var trsTopic: String

    constructor(mqttClient: MqttClient) : this() {
        this.mqttClient = mqttClient
    }

    constructor(mqttBroker: String): this() {
        this.mqttClient = mqttBrokerConnect(mqttBroker, getTwinUUID())
    }

    fun connectAndSubscribeToPlans() {
        try {
            registerWithWHC(mqttClient)
        } catch (e: MqttException) {
            log.error("Failed to connect to the MQTT broker")
        }
    }

    fun unregisterTwinAndDisconnect() {
        try {
            // we need to send LWT by hand due to a clean disconnect
            mqttClient.publish(MqttTopics.REGISTRATION_ANNOUNCE,
                    getTwinUnregistrationMessage().payload, 2, false)
            mqttClient.disconnect()
            log.debug("Disconnected from the MQTT broker")
        } catch (e: MqttException) {
            log.error("Failed to disconnect from the MQTT broker")
        }
    }

    private fun registerWithWHC(mqttClient: MqttClient) {
        val latch = CountDownLatch(1)
        try {
            mqttClient.subscribe(
                MqttTopics.REGISTRATION_ACK) { topic, message ->
                completeRegistration(message, latch)
            }
        } catch (e: MqttException) {
            log.error("Something went wrong with the REG_ACK subscription", e)
        }

        try {
            while (latch.count > 0) {
                log.trace("Posting the registration message")
                mqttClient.publish(MqttTopics.REGISTRATION_ANNOUNCE, getTwinRegistrationMessage())
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

    private fun completeRegistration(message: MqttMessage, latch: CountDownLatch) {
        val model = MqttHelper.extractModelFromMessage(message)
        try {
            val serverAck = JenaModelHelper.unmarshalSingle(model, TrsServerAck::class.java)
            if (getTwinUUID() == serverAck.adaptorId) {
                log.debug("The WHC registration of the {} has been confirmed", getTwinUUID())
                trsTopic = serverAck.trsTopic
                log.info("Using the '{}' topic to monitor new plans", trsTopic)
                subscribeToPlans(trsTopic)
                latch.countDown()
            }
        } catch (e: LyoJenaModelException) {
            log.error("Error unmarshalling TrsServerAck from the server response", e)
        }

    }

    @Throws(MqttException::class)
    private fun mqttBrokerConnect(mqttBroker: String, mqttClientId: String): MqttClient {
        val mqttClient = MqttClient(mqttBroker, mqttClientId)
        val mqttConnectOptions = MqttConnectOptions()
        mqttConnectOptions.isAutomaticReconnect = true
        mqttConnectOptions.setWill(MqttTopics.REGISTRATION_ANNOUNCE,
                getTwinUnregistrationMessage().payload, 2, false)
        mqttClient.setCallback(LoggingMqttCallback())
        // TODO Andrew@2018-03-13: set highest QoS
        mqttClient.connect(mqttConnectOptions)
        return mqttClient
    }

    private fun subscribeToPlans(trsTopic: String) {
        mqttClient.subscribe(trsTopic, ChangeEventMqttMessageListener(
            PlanChangeEventListener(executorService)))
        // TODO Andrew@2018-07-29: shall I ACK this too to make WHC registration deterministic?
//        mqttClient.subscribe(trsTopic, ChangeEventMqttMessageListener(
//                ChangeEventListener { changeEvent, trsResourceModel ->
//                    log.info("Change Event on resource ${changeEvent.changed} received")
//                }, executorService))
    }
    // HELPERS

    private fun getTwinUUID() = TwinAdaptorHelper.getTwinUUID()

    private fun getTwinRegistrationMessage(isLeaving: Boolean = false): MqttMessage {
        val trsUri = UriBuilder.fromUri(OSLC4JUtils.getServletURI()).path("trs").build()
        val announcement = TrsServerAnnouncement(getTwinUUID(), TrsXConstants.TYPE_TWIN, trsUri, MqttTopics.REGISTRATION_ANNOUNCE,
                isLeaving)
        return MqttHelper.msgFromResources(TrsXConstants.rdfFormat, announcement)
    }

    private fun getTwinUnregistrationMessage(): MqttMessage {
        return getTwinRegistrationMessage(true)
    }
}
