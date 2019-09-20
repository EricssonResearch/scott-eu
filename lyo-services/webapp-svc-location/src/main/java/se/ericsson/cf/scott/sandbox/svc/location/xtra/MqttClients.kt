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
package se.ericsson.cf.scott.sandbox.svc.location.xtra

import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.util.UUID

object MqttClients {
    private val log: Logger = LoggerFactory.getLogger(MqttClients::class.java)

    private fun clientUUID(): String {
        val id = UUID.randomUUID().toString()
        return "scott+$id"
    }

    @JvmOverloads
    fun connect(broker: String, clientID: String = clientUUID()): MqttClient {
        log.debug("Initialising an MQTT client (id=$clientID)")
        val options = MqttConnectOptions()

        val memoryPersistence = MemoryPersistence()
        val mqttClient = MqttClient(broker, clientID, memoryPersistence)

        options.isAutomaticReconnect = true
        options.connectionTimeout = 3
        options.keepAliveInterval = 5

        mqttClient.connect(options)

        return mqttClient
    }
}
