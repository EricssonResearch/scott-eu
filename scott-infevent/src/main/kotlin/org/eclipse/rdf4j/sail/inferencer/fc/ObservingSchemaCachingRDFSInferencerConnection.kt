package org.eclipse.rdf4j.sail.inferencer.fc

import org.eclipse.rdf4j.model.IRI
import org.eclipse.rdf4j.model.Resource
import org.eclipse.rdf4j.model.URI
import org.eclipse.rdf4j.model.Value
import org.eclipse.rdf4j.model.vocabulary.RDF
import org.eclipse.rdf4j.model.vocabulary.RDFS
import org.eclipse.rdf4j.sail.inferencer.InferencerConnection
import org.slf4j.Logger
import org.slf4j.LoggerFactory

class ObservingSchemaCachingRDFSInferencerConnection(sail: SchemaCachingRDFSInferencer?,
                                                     connection: InferencerConnection?) :
    SchemaCachingRDFSInferencerConnection(sail, connection) {
    companion object {
        val log: Logger = LoggerFactory.getLogger(ObservingSchemaCachingRDFSInferencerConnection::class.java)
    }

    override fun addInferredStatement(subj: Resource?, pred: IRI?, obj: Value?,
                                      vararg contexts: Resource?): Boolean {
        if (nonRejectedInference(subj, pred, obj)) {
            log.info("Inference added <${subj}, ${pred}, ${obj}>")
        }
        return super.addInferredStatement(subj, pred, obj, *contexts)
    }

    private fun nonRejectedInference(subj: Resource?, pred: IRI?, obj: Value?): Boolean {
        if(pred == RDF.TYPE && obj == RDFS.RESOURCE) {
            return false
        }
        if(pred == RDFS.SUBCLASSOF && obj == RDFS.RESOURCE) {
            return false
        }
        return true
    }

    override fun removeInferredStatement(subj: Resource?, pred: IRI?, obj: Value?,
                                         vararg contexts: Resource?): Boolean {
        log.info("Inference retracted <${subj}, ${pred}, ${obj}>")
        return super.removeInferredStatement(subj, pred, obj, *contexts)
    }
}
