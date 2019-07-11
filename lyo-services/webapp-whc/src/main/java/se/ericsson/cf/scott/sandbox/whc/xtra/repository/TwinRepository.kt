package se.ericsson.cf.scott.sandbox.whc.xtra.repository

import com.google.common.collect.ImmutableSet
import eu.scott.warehouse.domains.twins.RegistrationMessage
import java.net.URI
import java.util.Collections
import java.util.concurrent.ConcurrentHashMap
import javax.ws.rs.core.UriBuilder

data class TwinInfo(val id: String, val label: String, val cfUri: String)


class TwinRepository {
    private val _twins: MutableSet<TwinInfo> = Collections.newSetFromMap(ConcurrentHashMap())
    val twins: Set<TwinInfo>
        get() = ImmutableSet.copyOf(_twins)

    fun registerTwin(message: RegistrationMessage) {
        // TODO Andrew@2019-04-21: send in the RegistrationMessage
        val cfUri: URI = UriBuilder.fromUri(message.serviceProvider.value)
            .path("/planExecutionRequests/create")
            .build()
        _twins += TwinInfo(message.twinId, message.label, cfUri.toString())
    }
}
