package eu.scott.warehouse

import com.google.common.base.Strings
import eu.scott.warehouse.domains.trs.LoggingMqttCallback
import java.util.UUID
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttException
import org.eclipse.paho.client.mqttv3.MqttMessage

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
class MqttClientBuilder {
    private var clientID = UUID.randomUUID().toString()
    private var callback: MqttCallbackExtended = LoggingMqttCallback()
    private val lwtTopic: String? = null
    private val lwtMessage: MqttMessage? = null
    private var brokerURL = "tcp://localhost:1883"
    private var registrationAgent: RegistrationAgent? = null

    fun withBroker(URL: String): MqttClientBuilder {
        this.brokerURL = URL
        return this
    }

    fun withId(id: String): MqttClientBuilder {
        if (Strings.isNullOrEmpty(id)) {
            throw IllegalArgumentException("ID must be unique and not null")
        }
        this.clientID = id
        return this
    }

    fun withCallback(callback: MqttCallbackExtended): MqttClientBuilder {
        this.callback = callback
        return this
    }

    fun withRegistration(registrationAgent: RegistrationAgent): MqttClientBuilder {
        this.registrationAgent = registrationAgent
        return this
    }

    @Throws(MqttException::class)
    fun build(): TrsMqttGateway {
        if (Strings.isNullOrEmpty(brokerURL) || Strings.isNullOrEmpty(clientID)) {
            throw IllegalStateException(
                    "MqttGateway cannot be built without broker URL and client ID")
        }

        val options = MqttConnectOptions()
        options.isAutomaticReconnect = true
        //        options.setCleanSession(true);
        options.connectionTimeout = 1
        options.keepAliveInterval = 5
        if (lwtMessage != null) {
            options.setWill(lwtTopic!!, lwtMessage.payload, 1, false)
        }
        return TrsMqttGateway(brokerURL, clientID, options, callback, registrationAgent!!)
    }
}
