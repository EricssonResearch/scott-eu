package se.ericsson.cf.scott.sandbox.twin.xtra.trs

import eu.scott.warehouse.lib.RdfHelpers
import eu.scott.warehouse.lib.toTurtle
import org.eclipse.lyo.core.trs.Creation
import org.eclipse.lyo.core.trs.Deletion
import org.eclipse.lyo.core.trs.Modification
import org.eclipse.lyo.store.Store
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import redis.clients.jedis.Jedis
import java.net.URI

/**
 * The idea is the following: we can fix the page size for the TRS log and use Redis to mint us new
 * 'order' values. We then put
 */
class ConcurrentRedisAppender(private val jedis: Jedis, private val store: Store) :
    ITrsLogAppender {
    val log: Logger = LoggerFactory.getLogger(ConcurrentRedisAppender::class.java)

    override fun appendCreationEvent(changed: URI, twinKind: String, twinId: String): Creation {
        log.trace("Appending a Creation Event for {} (Twin {}:{})", changed, twinKind, twinId)
        val order = nextOrderId(twinKind, twinId)
        log.debug("Minted order={} (Twin {}:{})", order, twinKind, twinId)
        val creation = Creation(RdfHelpers.randomUuidUrn(), changed, order)
        // TODO Andrew@2019-07-16: what to do with the order if the write fails?
        store.appendResource(ITrsLogAppender.logGraphFor(twinKind, twinId, order), creation)
        log.info("Added a Creation Event to the TRS log ({})", creation.about)
        log.trace(creation.toTurtle)
        return creation
    }

    // TODO Andrew@2019-07-16: remove cast after https://github.com/eclipse/lyo.core/issues/104 is fixed
    @Synchronized //just in case, Jedis is not thread-safe unless we use pools &c.
    private fun nextOrderId(twinKind: String, twinId: String): Int =
        jedis.incr("order:$twinKind:$twinId").toInt()

    override fun appendModificationEvent(changed: URI, twinKind: String,
                                         twinId: String): Modification {
        TODO("not implemented")
    }

    override fun appendDeletionEvent(changed: URI, twinKind: String, twinId: String): Deletion {
        TODO("not implemented")
    }
}
