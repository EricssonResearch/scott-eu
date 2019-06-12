import org.apache.jena.graph.Graph
import org.apache.jena.graph.Triple
import org.apache.jena.rdf.listeners.ChangedListener
import org.apache.jena.rdf.model.InfModel
import org.apache.jena.rdf.model.Model
import org.apache.jena.rdf.model.ModelFactory
import org.apache.jena.reasoner.Finder
import org.apache.jena.reasoner.Reasoner
import org.apache.jena.reasoner.rulesys.FBRuleInfGraph
import org.apache.jena.reasoner.rulesys.RDFSRuleReasoner
import org.apache.jena.reasoner.rulesys.RDFSRuleReasonerFactory
import org.apache.jena.reasoner.rulesys.RulePreprocessHook
import org.apache.jena.riot.RDFDataMgr
import org.apache.jena.riot.RDFFormat
import org.apache.jena.util.FileUtils
import org.apache.jena.vocabulary.RDF
import org.apache.jena.vocabulary.ReasonerVocabulary
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import java.io.StringReader
import java.io.StringWriter

fun main() {
    System.setProperty(org.slf4j.impl.SimpleLogger.DEFAULT_LOG_LEVEL_KEY, "DEBUG");

    val log: Logger = LoggerFactory.getLogger("main")
    val model = ModelFactory.createDefaultModel()
    val reasoner: RDFSRuleReasoner = RDFSRuleReasonerFactory.theInstance().create(null) as RDFSRuleReasoner

    reasoner.setParameter(ReasonerVocabulary.PROPsetRDFSLevel, ReasonerVocabulary.RDFS_SIMPLE)
    reasoner.setParameter(ReasonerVocabulary.PROPderivationLogging, true)
    reasoner.setParameter(ReasonerVocabulary.PROPtraceOn, true)

    val infModel: InfModel = ModelFactory.createInfModel(reasoner, model)
    infModel.setDerivationLogging(true)
    infModel.read(StringReader("""
            @prefix rdf:   <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
            @prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>.
            @prefix : <http://example.com/ns/#>.
            :Publication a rdfs:Class .
            :Book a rdfs:Class;
                rdfs:subClassOf :Publication .
        """.trimIndent()), "http://example.com/ns/#", FileUtils.langTurtle)

    val rawChangedListener = ChangedListener()
    val deductionsChangedListener = ChangedListener()
    infModel.rawModel.register(rawChangedListener)
    infModel.deductionsModel.register(deductionsChangedListener)

    log.info("Changes before creation: raw={}, deductions={}", rawChangedListener.hasChanged(),
        deductionsChangedListener.hasChanged())
    infModel.createResource("http://example.com/books/1")
        .addProperty(RDF.type, model.createResource("http://example.com/ns/#Book"))
    log.info("Changes after creation : raw={}, deductions={}", rawChangedListener.hasChanged(),
        deductionsChangedListener.hasChanged())

    infModel.prepare()
    log.info("Changes after prepare  : raw={}, deductions={}", rawChangedListener.hasChanged(),
        deductionsChangedListener.hasChanged())

    log.info(modelToString(infModel))
}

fun modelToString(model: Model, rdfFormat: RDFFormat = RDFFormat.TURTLE_PRETTY): String {
    val writer = StringWriter()
    RDFDataMgr.write(writer, model, rdfFormat)
    return writer.toString()
}
