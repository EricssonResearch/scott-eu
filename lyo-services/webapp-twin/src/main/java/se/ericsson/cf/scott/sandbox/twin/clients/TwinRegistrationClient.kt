package se.ericsson.cf.scott.sandbox.twin.clients

import eu.scott.warehouse.domains.twins.RegistrationMessage
import eu.scott.warehouse.lib.link
import org.apache.wink.client.ClientResponse
import org.eclipse.lyo.client.oslc.OslcClient
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider
import org.slf4j.LoggerFactory
import java.net.URI

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
class TwinRegistrationClient(private val client: OslcClient, private val registrationCFUri: String) {
    val log = LoggerFactory.getLogger(javaClass)

    fun registerTwin(serviceProvider: ServiceProvider): ClientResponse? {
        val m = RegistrationMessage()

        m.about = URI.create("http://twin/registrationMessage_${System.currentTimeMillis()}")
        m.serviceProvider = serviceProvider.link
        m.isDeregister = false
//        m.trsUri = trsURI(serviceProvider)
//        m.trsMqttTopic = trsMqttTopic(m.trsUri)
        m.label = serviceProvider.description

        return client.createResource(registrationCFUri, m, OslcMediaType.TEXT_TURTLE)
    }

    @Deprecated("Should pass the newly created ServiceProvider instead")
    fun registerTwin(id: String) {
        val registrationMessage = RegistrationMessage()
        registrationMessage.trsUri = trsURI(id)
        registrationMessage.trsMqttTopic = trsMqttTopic(id)
        val createResource = client.createResource(registrationCFUri, registrationMessage,
            OslcMediaType.TEXT_TURTLE)
    }

    private fun trsMqttTopic(trsURI: URI?): String {
        TODO("TRS Server MQTT topic can't be returned")
    }

    private fun trsURI(id: ServiceProvider): URI? {
        TODO("TRS Server MQTT topic can't be returned")
    }

    @Deprecated("Remove dummy impl")
    fun trsMqttTopic(id: String) = "scott.warehouse.todo"

    @Deprecated("Remove dummy impl")
    private fun trsURI(id: String): URI? {
        return URI.create("urn:x:todo:"+id)
    }
}
