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

package eu.scott.warehouse.lib

import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.eclipse.lyo.core.trs.ChangeEvent
import org.eclipse.lyo.oslc4j.core.model.IResource
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.io.ByteArrayInputStream
import javax.swing.JMenu


object MqttTrsServices {
    val log: Logger = LoggerFactory.getLogger(MqttTrsServices.javaClass)
    var QoS = 1
    val defaultLang: Lang = Lang.TURTLE
    val defaultFormat: RDFFormat = RDFFormat.TURTLE_PRETTY

    @JvmStatic
    fun msgFromResources(rdfFormat: RDFFormat, vararg resources: IResource): MqttMessage {
        val model = RdfHelpers.modelFromResources(*resources)
        return msgFromModel(model, rdfFormat)
    }

    @JvmStatic
    fun msgFromResources(vararg resources: IResource): MqttMessage {
        val model = RdfHelpers.modelFromResources(*resources)
        return msgFromModel(model, defaultFormat)
    }

    @JvmStatic
    fun msgFromModel(model: Model, rdfFormat: RDFFormat = defaultFormat): MqttMessage {
        val rdfString = RdfHelpers.modelToString(model, rdfFormat)
        val mqttMessage = MqttMessage(rdfString.toByteArray())
        mqttMessage.qos = QoS
        return mqttMessage
    }

    @JvmStatic
    fun msgFromEventWithModel(event: ChangeEvent, model: Model,
                              rdfFormat: RDFFormat = defaultFormat): MqttMessage {
        val eventModel = JenaModelHelper.createJenaModel(arrayOf(event))
        eventModel.add(model)
        val rdfString = RdfHelpers.modelToString(eventModel, rdfFormat)
        val mqttMessage = MqttMessage(rdfString.toByteArray())
        mqttMessage.qos = QoS
        return mqttMessage
    }

    @JvmStatic
    fun extractModelFromMessage(message: MqttMessage): Model {
        val inputStream = ByteArrayInputStream(message.payload)
        val model = ModelFactory.createDefaultModel()
        RDFDataMgr.read(model, inputStream, defaultLang)
        return model
    }

    @JvmStatic
    fun trsMqttTopic(twinKind: String, twinId: String): String {
        return String.format("scott/trs/twins/%s.%s", twinKind, twinId)
    }

}
