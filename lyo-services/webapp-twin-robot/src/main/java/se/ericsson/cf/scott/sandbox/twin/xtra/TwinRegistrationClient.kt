package se.ericsson.cf.scott.sandbox.twin.xtra

import eu.scott.warehouse.domains.twins.RegistrationMessage
import eu.scott.warehouse.lib.link
import org.eclipse.lyo.oslc4j.client.OslcClient
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider
import org.slf4j.LoggerFactory
import java.net.URI
import javax.ws.rs.core.Response

object TwinRegistrationHelper {
    fun stripIdentifierUri(
        uriString: String) = uriString.split('/').last()

}

class TwinRegistrationClient(private val client: OslcClient, private val registrationCFUri: String) {
    val log = LoggerFactory.getLogger(javaClass)

    fun registerTwin(serviceProvider: ServiceProvider): Response? {
        val m = RegistrationMessage()

        m.about = URI.create("urn:twin:registrationMessage_${System.currentTimeMillis()}")
        m.serviceProvider = serviceProvider.link
        m.isDeregister = false
//        m.trsUri = trsURI(serviceProvider)
//        m.trsMqttTopic = trsMqttTopic(m.trsUri)
        m.label = serviceProvider.description
        m.twinId = TwinRegistrationHelper.stripIdentifierUri(serviceProvider.identifier)

        return client.createResource(registrationCFUri, m, OslcMediaType.TEXT_TURTLE)
    }


    @Deprecated("Remove dummy impl")
    fun trsMqttTopic(id: String) = "scott.warehouse.todo"

    @Deprecated("Remove dummy impl")
    private fun trsURI(id: String): URI? {
        return URI.create("urn:x:todo:"+id)
    }
}
