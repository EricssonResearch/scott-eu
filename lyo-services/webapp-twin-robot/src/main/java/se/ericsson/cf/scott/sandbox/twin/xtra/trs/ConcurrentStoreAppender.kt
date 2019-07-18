package se.ericsson.cf.scott.sandbox.twin.xtra.trs

import eu.scott.warehouse.lib.RdfHelpers
import eu.scott.warehouse.lib.toTurtle
import org.apache.jena.rdf.model.Model
import org.eclipse.lyo.core.trs.Creation
import org.eclipse.lyo.core.trs.Deletion
import org.eclipse.lyo.core.trs.Modification
import org.eclipse.lyo.store.Store
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.net.URI

/**
 * The idea is the following: we can fix the page size for the TRS log and use Redis to mint us new
 * 'order' values. We then put
 */
class ConcurrentStoreAppender(private val store: Store, private val orderGenerator: IConcurrentOrderGenerator) :
    ITrsLogAppender {
    val log: Logger = LoggerFactory.getLogger(ConcurrentStoreAppender::class.java)

    override fun appendCreationEvent(changed: URI,
                                     model: Model,
                                     twinKind: String, twinId: String): Creation {
        log.trace("Appending a Creation Event for {} (Twin {}:{})", changed, twinKind, twinId)
        val order = orderGenerator.nextOrder(twinKind, twinId)
        log.debug("Minted order={} (Twin {}:{})", order, twinKind, twinId)
        val creation = Creation(RdfHelpers.randomUuidUrn(), changed, order)
        // TODO Andrew@2019-07-16: what to do with the order if the write fails?
        store.appendResource(ITrsLogAppender.logGraphFor(twinKind, twinId, order), creation)
        log.info("Added a Creation Event to the TRS log ({})", creation.about)
        log.trace(creation.toTurtle)
        return creation
    }

    override fun appendModificationEvent(changed: URI, twinKind: String,
                                         twinId: String, model: Model): Modification {
        TODO("not implemented")
    }

    override fun appendDeletionEvent(changed: URI, twinKind: String, twinId: String, model: Model): Deletion {
        TODO("not implemented")
    }
}
