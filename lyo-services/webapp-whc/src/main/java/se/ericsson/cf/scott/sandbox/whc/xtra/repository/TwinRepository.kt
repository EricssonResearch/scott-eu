package se.ericsson.cf.scott.sandbox.whc.xtra.repository

import com.google.common.collect.ImmutableSet
import eu.scott.warehouse.domains.twins.RegistrationMessage
import java.util.Collections
import java.util.concurrent.ConcurrentHashMap

data class TwinInfo(val label: String, val id: String)


class TwinRepository {
    private val _twins: MutableSet<TwinInfo> = Collections.newSetFromMap(ConcurrentHashMap())
    val twins: Set<TwinInfo>
        get() = ImmutableSet.copyOf(_twins)

    fun registerTwin(message: RegistrationMessage) {
        // FIXME Andrew@2019-04-20: extract an ID
        _twins += TwinInfo(message.label, message.label)
    }
}
