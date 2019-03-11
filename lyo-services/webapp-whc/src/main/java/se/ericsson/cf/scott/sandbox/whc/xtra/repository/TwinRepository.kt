package se.ericsson.cf.scott.sandbox.whc.xtra.repository

import com.google.common.collect.ImmutableSet
import eu.scott.warehouse.domains.twins.RegistrationMessage
import java.util.Collections
import java.util.concurrent.ConcurrentHashMap

data class TwinInfo(val label: String)

/**
 * TODO
 *
 */
class TwinRepository {
    private val _twins: MutableSet<TwinInfo> = Collections.newSetFromMap(ConcurrentHashMap())
    val twins: Set<TwinInfo>
        get() = ImmutableSet.copyOf(_twins)

    fun registerTwin(message: RegistrationMessage) {
        _twins += TwinInfo(message.label)
    }
}
