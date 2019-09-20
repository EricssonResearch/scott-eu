package eu.scott.warehouse.domains.trs

import org.apache.jena.riot.Lang
import org.apache.jena.riot.RDFFormat


object TrsXConstants {
    const val NS: String = "http://ontology.cf.ericsson.net/ns/trs-x#"
    fun type(name: String) = NS + name

    const val TYPE_TWIN = "twin"

}
