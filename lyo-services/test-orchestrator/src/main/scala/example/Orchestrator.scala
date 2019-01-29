package example

import java.net.URI

import com.typesafe.scalalogging.LazyLogging
import eu.scott.warehouse.domains.twins.DeviceRegistrationMessage
import org.eclipse.lyo.client.oslc.OslcClient
import org.eclipse.lyo.oslc4j.core.model.OslcMediaType


class RegistrationMessageFactory {
  def create(kind: String): DeviceRegistrationMessage = {
    val message = new DeviceRegistrationMessage(URI.create(s"http://example.com/$kind"))
    message.setTwinType(kind)
    return message
  }

  def create(kind: String, id: Int): DeviceRegistrationMessage = {
    // TODO Andrew@2018-09-03: remove other reg msgs, from twins & mission DSs
    val message = new DeviceRegistrationMessage(URI.create(s"http://example.com/$kind/$id"))
    // TODO Andrew@2018-09-03: rename to "kind", as in SPInfo
    message.setTwinType(kind)
    message.setTwinId(id.toString)
    return message
  }
}


/**
  * TODO
  *
  * @version $version-stub$
  * @since   TODO
  */
object Orchestrator extends App with LazyLogging {
  private val registrationCF = "http://sandbox-twin:8081/services/deviceRegistrationMessages/register"
  private val r = scala.util.Random
  private val messageFactory = new RegistrationMessageFactory()

  println("Registering the twins...")

  private val registrationMessages: Seq[DeviceRegistrationMessage] =
    for (i <- 1 to 10) yield messageFactory
      .create("robot")

  private val client = new OslcClient()

  for (r <- registrationMessages) {
    val clientResponse = client
      .createResource(registrationCF,
        r, OslcMediaType.TEXT_TURTLE)
    clientResponse.getStatusCode match {
      case it if 200 to 299 contains it => {
        val message: DeviceRegistrationMessage = clientResponse
          .getEntity(classOf[DeviceRegistrationMessage])
        logger.info(s"Response for twin ${r.getTwinId}: $message")
      }
      case it if 400 to 599 contains it =>
        logger.warn(s"Response for twin ${r.getTwinId}: " +
                    s"ERROR ${clientResponse.getStatusCode} ${clientResponse.getStatusType}")
    }
  }
}

