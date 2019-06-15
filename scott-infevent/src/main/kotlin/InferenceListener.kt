import org.apache.jena.rdf.listeners.ChangedListener
import org.apache.jena.rdf.model.InfModel
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.reasoner.rulesys.RDFSRuleReasoner
import org.apache.jena.reasoner.rulesys.RDFSRuleReasonerFactory
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.apache.jena.util.FileUtils
import org.apache.jena.vocabulary.ReasonerVocabulary
import org.eclipse.rdf4j.repository.sail.SailRepository
import org.eclipse.rdf4j.repository.sail.SailRepositoryConnection
import org.eclipse.rdf4j.sail.SailChangedEvent
import org.eclipse.rdf4j.sail.SailConnectionListener
import org.eclipse.rdf4j.sail.inferencer.fc.ObservingSchemaCachingRDFSInferencerConnection
import org.eclipse.rdf4j.sail.inferencer.fc.SchemaCachingRDFSInferencer
import org.eclipse.rdf4j.sail.inferencer.fc.SchemaCachingRDFSInferencerConnection
import org.eclipse.rdf4j.sail.memory.MemoryStore
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.io.StringReader
import java.io.StringWriter

fun main() {
    System.setProperty(org.slf4j.impl.SimpleLogger.DEFAULT_LOG_LEVEL_KEY, "DEBUG");

//    jenaNotifications()
    rdf4jNotifications()
}

fun rdf4jNotifications() {
    val log: Logger = LoggerFactory.getLogger("rdf4j")

    val inferencer: SchemaCachingRDFSInferencer = object : SchemaCachingRDFSInferencer(MemoryStore()) {
        override fun getConnection(): SchemaCachingRDFSInferencerConnection {
            return ObservingSchemaCachingRDFSInferencerConnection(this, super.getConnection())
        }
    }
    val repository = SailRepository(inferencer)
    repository.init()
    val factory = repository.valueFactory

    repository.connection.use { conn: SailRepositoryConnection ->
        conn.add(getSchemaReader(), "http://example.com/ns/#", org.eclipse.rdf4j.rio.RDFFormat.TURTLE)
    }

    log.info("Initialised domain schema")

    inferencer.addSailChangedListener { event: SailChangedEvent ->
        log.info("Statements added: {} / removed: {}", event.statementsAdded(), event.statementsRemoved())
    }

    val book = factory.createIRI("http://example.com/books/1")
    val bookType = factory.createIRI("http://example.com/ns/#Book")

    repository.connection.use { conn: SailRepositoryConnection ->
        conn.add(book, org.eclipse.rdf4j.model.vocabulary.RDF.TYPE, bookType)
    }
}

private fun jenaNotifications() {
    val log: Logger = LoggerFactory.getLogger("jena")
    val model = ModelFactory.createDefaultModel()
    val reasoner: RDFSRuleReasoner = RDFSRuleReasonerFactory.theInstance().create(null) as RDFSRuleReasoner

    reasoner.setParameter(ReasonerVocabulary.PROPsetRDFSLevel, ReasonerVocabulary.RDFS_SIMPLE)
    reasoner.setParameter(ReasonerVocabulary.PROPderivationLogging, true)
    reasoner.setParameter(ReasonerVocabulary.PROPtraceOn, true)

    val infModel: InfModel = ModelFactory.createInfModel(reasoner, model)
    infModel.setDerivationLogging(true)
    infModel.read(getSchemaReader(), "http://example.com/ns/#", FileUtils.langTurtle)

    val rawChangedListener = ChangedListener()
    val deductionsChangedListener = ChangedListener()
    infModel.rawModel.register(rawChangedListener)
    infModel.deductionsModel.register(deductionsChangedListener)

    log.info("Changes before creation: raw={}, deductions={}", rawChangedListener.hasChanged(),
        deductionsChangedListener.hasChanged())
    infModel.createResource("http://example.com/books/1")
        .addProperty(org.apache.jena.vocabulary.RDF.type, model.createResource("http://example.com/ns/#Book"))
    log.info("Changes after creation : raw={}, deductions={}", rawChangedListener.hasChanged(),
        deductionsChangedListener.hasChanged())

    infModel.prepare()
    log.info("Changes after prepare  : raw={}, deductions={}", rawChangedListener.hasChanged(),
        deductionsChangedListener.hasChanged())

    log.info(modelToString(infModel))
}

private fun getSchemaReader(): StringReader {
    return StringReader("""
            @prefix rdf:   <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
            @prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>.
            @prefix : <http://example.com/ns/#>.
            :Publication a rdfs:Class .
            :Book a rdfs:Class;
                rdfs:subClassOf :Publication .
        """.trimIndent())
}

fun modelToString(model: Model, rdfFormat: RDFFormat = RDFFormat.TURTLE_PRETTY): String {
    val writer = StringWriter()
    RDFDataMgr.write(writer, model, rdfFormat)
    return writer.toString()
}
