/*
 * Copyright (c) 2019  Ericsson Research and others
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
package eu.scott.warehouse.lib.trs

import eu.scott.warehouse.lib.MqttTrsServices
import eu.scott.warehouse.lib.RdfHelpers
import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.core.trs.ChangeEvent
import org.eclipse.lyo.core.trs.Creation
import org.eclipse.lyo.core.trs.Deletion
import org.eclipse.lyo.core.trs.Modification
import org.eclipse.paho.client.mqttv3.MqttClient
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.net.URI

class ConcurrentMqttAppender(private val mqttClient: MqttClient,
                             private val orderGenerator: IConcurrentOrderGenerator) :
    ITrsLogAppender {

    companion object {
        private val log: Logger = LoggerFactory.getLogger(ConcurrentMqttAppender::class.java)
    }

    // TODO Andrew@2019-07-22: refactor
    fun forwardEvent(event: ChangeEvent, model: Model, topic: String) {
        val message = MqttTrsServices.msgFromEventWithModel(event, model)
        mqttClient.publish(topic, message)
    }

    override fun appendCreationEvent(changed: URI, model: Model, twinKind: String,
                                     twinId: String): Creation {
        val topic = MqttTrsServices.trsMqttTopic(twinKind, twinId)
        val order = orderGenerator.nextOrder(twinKind, twinId)

        val creation = Creation(RdfHelpers.randomUuidUrn(), changed, order)
        val message = MqttTrsServices.msgFromEventWithModel(creation, model)

        log.debug("Publishing a Creation CE for $twinId to $topic ($changed)")
        mqttClient.publish(topic, message)

        return creation
    }

    override fun appendModificationEvent(changed: URI, twinKind: String, twinId: String,
                                         model: Model): Modification {
        TODO("not implemented")
    }

    override fun appendDeletionEvent(changed: URI, twinKind: String, twinId: String,
                                     model: Model): Deletion {
        TODO("not implemented")
    }

}
