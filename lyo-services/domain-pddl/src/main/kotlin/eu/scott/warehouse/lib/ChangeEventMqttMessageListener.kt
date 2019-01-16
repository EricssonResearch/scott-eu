/*
 * Copyright (c) 2017-2018 Xufei Ning and others.
 *
 * All rights reserved. This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v1.0 and Eclipse Distribution License v. 1.0 which
 * accompanies this distribution.
 *
 * The Eclipse Public License is available at http://www.eclipse.org/legal/epl-v10.html and the
 * Eclipse Distribution License is available at http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *
 * Xufei Ning          -  Initial implementation
 * Andrew Berezovskyi  -  Lyo contribution updates
 */
package eu.scott.warehouse.lib

import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.rdf.model.Resource
import org.apache.jena.rdf.model.ResourceFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.eclipse.lyo.core.trs.ChangeEvent
import org.eclipse.lyo.core.trs.Creation
import org.eclipse.lyo.core.trs.Deletion
import org.eclipse.lyo.core.trs.Modification
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.oslc4j.provider.jena.LyoJenaModelException
import org.eclipse.lyo.oslc4j.trs.client.handlers.ChangeEventListener
import org.eclipse.lyo.oslc4j.trs.client.mqtt.ChangeEventMessage
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory
import java.io.ByteArrayInputStream
import java.net.URI
import java.nio.charset.StandardCharsets

/**
 * TBD
 *
 * @version $version-stub$
 * @since   FIXME
 */
class ChangeEventMqttMessageListener(private val changeEventListener: ChangeEventListener) :
        IMqttMessageListener {
    private val log = LoggerFactory.getLogger(javaClass)

    override fun messageArrived(s: String, mqttMessage: MqttMessage) {
        val payload = String(mqttMessage.payload)
        log.trace("Change Event received: $payload")
        val handleChangeEvent = Runnable {
            log.info("Full ChangeEvent received")
            val eventMessage = unmarshalChangeEvent(payload)
            val changeEvent = eventMessage.changeEvent
            log.debug("Processing a fat ping for {}", changeEvent)
            val trsResourceModel = eventMessage.mqttMessageModel
            notifyListener(changeEvent, trsResourceModel)
        }
        // FIXME Andrew@2018-07-28: run in the BG thread once tested to work
        handleChangeEvent.run()
//            executorService.submit(handleChangeEvent)
    }

    // TODO Andrew@2018-07-28: make non nullable
    private fun notifyListener(changeEvent: ChangeEvent?, trsResourceModel: Model?) {
        log.trace("Notifying changeEventListener about {}", changeEvent)
        changeEventListener.handleChangeEvent(changeEvent, trsResourceModel)
    }

    @Throws(LyoJenaModelException::class)
    private fun unmarshalChangeEvent(payload: String): ChangeEventMessage {
        log.debug("MQTT payload: {}", payload)
        var changeEvent: ChangeEvent
        val payloadModel = ModelFactory.createDefaultModel()
        val inputStream = ByteArrayInputStream(payload.toByteArray(StandardCharsets.UTF_8))
        RDFDataMgr.read(payloadModel, inputStream, Lang.JSONLD)
        try {1
            changeEvent = JenaModelHelper.unmarshalSingle(payloadModel, Modification::class.java)
        } catch (e: LyoJenaModelException) {
            try {
                changeEvent = JenaModelHelper.unmarshalSingle(payloadModel, Creation::class.java)
            } catch (e1: LyoJenaModelException) {
                try {
                    changeEvent = JenaModelHelper.unmarshalSingle(payloadModel,
                            Deletion::class.java)
                } catch (e2: LyoJenaModelException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                } catch (e2: IllegalArgumentException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                }

            } catch (e1: IllegalArgumentException) {
                try {
                    changeEvent = JenaModelHelper.unmarshalSingle(payloadModel,
                            Deletion::class.java)
                } catch (e2: LyoJenaModelException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                } catch (e2: IllegalArgumentException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                }

            }

        } catch (e: IllegalArgumentException) {
            try {
                changeEvent = JenaModelHelper.unmarshalSingle(payloadModel, Creation::class.java)
            } catch (e1: LyoJenaModelException) {
                try {
                    changeEvent = JenaModelHelper.unmarshalSingle(payloadModel,
                            Deletion::class.java)
                } catch (e2: LyoJenaModelException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                } catch (e2: IllegalArgumentException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                }

            } catch (e1: IllegalArgumentException) {
                try {
                    changeEvent = JenaModelHelper.unmarshalSingle(payloadModel,
                            Deletion::class.java)
                } catch (e2: LyoJenaModelException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                } catch (e2: IllegalArgumentException) {
                    log.error("Can't unmarshal the payload", e, e1, e2)
                    throw e2
                }

            }

        }

        val tResourceUri = changeEvent.changed
        return ChangeEventMessage(payloadModel, changeEvent,
                payloadModel.containsResource(r(tResourceUri)))
    }

    /**
     * Dummy Jena Resource for a URI. Can be to do raw ops on a model.
     */
    private fun r(tResourceUri: URI): Resource {
        return ResourceFactory.createResource(tResourceUri.toString())
    }
}
