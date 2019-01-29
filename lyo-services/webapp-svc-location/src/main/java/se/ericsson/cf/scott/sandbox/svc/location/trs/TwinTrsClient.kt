package se.ericsson.cf.scott.sandbox.svc.location.trs

import org.eclipse.lyo.oslc4j.trs.client.handlers.TrsProviderHandler
import org.slf4j.LoggerFactory
import java.net.URI

data class TrsEndpoint(val baseURI: URI)

object TrsHelper {
    fun trsServerHandlerFor(e: TrsEndpoint): TrsProviderHandler {
        return TrsProviderHandler(e.baseURI.toASCIIString(), null, null, null, null, null,
            null, null)
    }
}

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
class TwinTrsClient {
    val log = LoggerFactory.getLogger(javaClass)!!

    init {
        log.info("Starting TRS client")
        log.warn("TRS Client not implemented")
    }
}
