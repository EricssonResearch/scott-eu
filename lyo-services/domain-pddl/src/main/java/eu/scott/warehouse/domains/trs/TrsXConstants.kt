package eu.scott.warehouse.domains.trs

import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFFormat

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
object TrsXConstants {
    const val NS: String = "http://ontology.cf.ericsson.net/ns/trs-x#"
    fun type(name: String) = NS + name

    const val TYPE_TWIN = "twin"
    const val TYPE_EXECUTOR = "plan_executor"

    public val rdfLang: Lang = Lang.JSONLD
    public val rdfFormat: RDFFormat = RDFFormat.JSONLD_PRETTY
}
