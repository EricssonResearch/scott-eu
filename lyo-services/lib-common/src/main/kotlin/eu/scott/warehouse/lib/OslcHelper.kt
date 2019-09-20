package eu.scott.warehouse.lib

import java.net.URI
import java.util.UUID
import javax.ws.rs.core.UriBuilder

class OslcHelper(private val base: URI) {

    fun u(p: String): URI {
        return UriBuilder.fromUri(base)
            .path(p).build()
    }

    fun u(p: UUID?): URI {
        return u("blnk_" + p.toString())
    }

    fun createRaw(): RawResource = RawResource(
        u(UUID.randomUUID()))

}
