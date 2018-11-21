package eu.scottproject.wp10.gw.backend
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.module.kotlin.readValue
import com.fasterxml.jackson.module.kotlin.registerKotlinModule
import eu.scottproject.wp10.gw.api.GatewayDiscoveryService
import org.zeromq.ZContext
import org.zeromq.ZMQ
import java.util.*

/**
 * TBD
 *
 * @version $version-stub$
 * @since   FIXME
 */

const val PULL_PORT = 5557
const val CONNECT_ADDR = "tcp://127.0.0.1:$PULL_PORT"

fun main(args: Array<String>) {
    println("Registering Gateway Discovery Services")
    val serviceLoader = ServiceLoader.load(GatewayDiscoveryService::class.java)
    serviceLoader.forEach {
        println("Found service: ${it.javaClass.canonicalName}")
    }
    val providers = serviceLoader.mapNotNull { it.marshallingProviders }.flatMap { it.entries }

    if (providers.isEmpty()) {
        println("No JARs have been placed on the classpath with a ServiceLoader definition for\n" +
                "\tsvc-sample/src/main/resources/META-INF/services/eu.scottproject.wp10.gw.api.GatewayDiscoveryService")
        System.exit(-1);
    }
    // Select the first service that can (un)marshall messages of type "sample"
    val simpleProvider = providers.first { it.key == "sample" }.value

    println("Pulling from $CONNECT_ADDR")
    val mapper = ObjectMapper().registerKotlinModule()
    ZContext().use { context ->
        // Socket to talk to clients
        val socket = context.createSocket(ZMQ.DEALER)
        // only one node binds, the rest connect
        socket.connect(CONNECT_ADDR)

        while (!Thread.currentThread().isInterrupted) {
            // Block until a message is received
            val reply = socket.recv(0)
            val rcvMessage = String(reply, ZMQ.CHARSET)

            // Uncomment the line below if you want to see raw JSON
//            println("Received: '$rcvMessage'")
            // show how Jackson can automatically map the JSON to a Kotlin class
            val msg = mapper.readValue<WorkerMessage>(reply)
            println("Worker number: ${msg.num}")

            simpleProvider.fromBytes(reply).ifPresent {
                println("Unmarshalling by the loaded service: $it")
            }

            // ZMQ.DEALER mode allows us to use the socket bidirectionally w/o blocking
            // but has limits on buffers (1000 msg approx)
            val response = "{\"id\": 1, \"status\": \"SUCCESS\"}";
            socket.send(response.toByteArray(ZMQ.CHARSET), 0)
        }
    }

}
