/*
 * Copyright (c) 2019 Ericsson Research and others
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package se.ericsson.cf.scott.sandbox.twin.xtra.trs

import eu.scott.warehouse.domains.trs.TrsServerAck
import eu.scott.warehouse.domains.trs.TrsServerAnnouncement
import eu.scott.warehouse.domains.trs.TrsXConstants
import eu.scott.warehouse.lib.MqttTrsServices
import eu.scott.warehouse.lib.MqttTopics
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils
import org.eclipse.lyo.oslc4j.core.exception.LyoModelException
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttException
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory
import se.ericsson.cf.scott.sandbox.twin.xtra.TwinAdaptorHelper
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import javax.ws.rs.core.UriBuilder
import kotlin.math.roundToLong


class TrsMqttPlanManager() {
    private val log = LoggerFactory.getLogger(javaClass)
    private lateinit var mqttClient: MqttClient
    private lateinit var trsTopic: String

    constructor(mqttClient: MqttClient) : this() {
        this.mqttClient = mqttClient
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
            mqttClient.subscribe(MqttTopics.REGISTRATION_ACK) { _, message ->
                completeRegistration(message, latch)
            }
        } catch (e: MqttException) {
            log.error("Something went wrong with the REG_ACK subscription", e)
        }

        try {
            var n = 1
            while (latch.count > 0) {
                // back off exponentially but no more than 30s
                val backoffDelay = (Math.exp(n+2.0)*2).coerceAtMost(30000.0)
                log.trace("Posting the registration message")
                mqttClient.publish(MqttTopics.REGISTRATION_ANNOUNCE, getTwinRegistrationMessage())
                latch.await(backoffDelay.roundToLong(), TimeUnit.MILLISECONDS)
                if (latch.count > 0) {
                    log.warn("Failed to register the twin with the WHC, attempting again")
                }

                // give up after 10 attempts
                if(n <= 10) {
                    n += 1
                } else {
                    log.error("Give up on registration with the WHC")
                    break
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
        val model = MqttTrsServices.extractModelFromMessage(message)
        try {
            val serverAck = JenaModelHelper.unmarshalSingle(model, TrsServerAck::class.java)
            if (getTwinUUID() == serverAck.adaptorId) {
                log.debug("The WHC registration of the {} has been confirmed", getTwinUUID())
                trsTopic = serverAck.trsTopic
                latch.countDown()
            }
        } catch (e: LyoModelException) {
            log.error("Error unmarshalling TrsServerAck from the server response", e)
        }

    }

    // HELPERS

    private fun getTwinUUID() = TwinAdaptorHelper.getTwinUUID()

    private fun getTwinRegistrationMessage(isLeaving: Boolean = false): MqttMessage {
        val trsUri = UriBuilder.fromUri(OSLC4JUtils.getServletURI()).path("trs").build()
        val announcement = TrsServerAnnouncement(getTwinUUID(), TrsXConstants.TYPE_TWIN, trsUri, MqttTopics.REGISTRATION_ANNOUNCE,
                isLeaving)
        return MqttTrsServices.msgFromResources(announcement)
    }

    private fun getTwinUnregistrationMessage(): MqttMessage {
        return getTwinRegistrationMessage(true)
    }
}
