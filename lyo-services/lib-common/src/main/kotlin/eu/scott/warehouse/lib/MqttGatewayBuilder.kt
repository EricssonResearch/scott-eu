package eu.scott.warehouse.lib

import com.google.common.base.Strings
import java.util.UUID
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttException
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory

class MqttGatewayBuilder {
    val log = LoggerFactory.getLogger(javaClass)
    private var clientID = UUID.randomUUID().toString()
    private var callback: MqttCallbackExtended = LoggingMqttCallback()
    private val lwtTopic: String? = null
    private val lwtMessage: MqttMessage? = null
    private var brokerURL = "tcp://localhost:1883"

    fun withBroker(URL: String): MqttGatewayBuilder {
        this.brokerURL = URL
        return this
    }

    fun withId(id: String): MqttGatewayBuilder {
        if (Strings.isNullOrEmpty(id)) {
            throw IllegalArgumentException("ID must be unique and not null")
        }
        this.clientID = id
        return this
    }

    fun withCallback(callback: MqttCallbackExtended): MqttGatewayBuilder {
        this.callback = callback
        return this
    }

    @Throws(MqttException::class)
    fun build(): RdfMqttGateway {
        if (Strings.isNullOrEmpty(brokerURL) || Strings.isNullOrEmpty(clientID)) {
            throw IllegalStateException(
                    "MqttGateway cannot be built without broker URL and client ID")
        }

        val options = MqttConnectOptions()
        options.isAutomaticReconnect = true
        //        options.setCleanSession(true);
        // these are too lax but the client otherwise tries to reconnect like crazy
        // TODO Andrew@2018-09-04: contribute a PR to https://github.com/eclipse/paho.mqtt.java/issues/374
        options.connectionTimeout = 3
        options.keepAliveInterval = 5
        if (lwtMessage != null) {
            options.setWill(lwtTopic!!, lwtMessage.payload, 1, false)
        }
        log.debug("Initialising an MQTT client (id=$clientID)")
        return RdfMqttGateway(brokerURL, clientID, options, callback)
    }
}
