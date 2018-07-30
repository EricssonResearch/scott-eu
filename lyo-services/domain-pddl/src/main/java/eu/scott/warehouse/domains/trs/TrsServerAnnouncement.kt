package eu.scott.warehouse.domains.trs

import eu.scott.warehouse.domains.pddl.PrimitiveType
import org.eclipse.lyo.oslc4j.core.annotation.*
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.core.model.Occurs
import org.eclipse.lyo.oslc4j.core.model.ValueType
import java.net.URI

/**
 * TBD
 *
 * @version $version-stub$
 * @since   0.0.1
 */

// TODO I would love just to have @OslcNamespace mandatory and have the rest inferred
@OslcNamespace(TrsXConstants.NS)
@OslcName("TrsServerAnnouncement")
@OslcResourceShape(describes = [(TrsXConstants.NS + "TrsServerAnnouncement")])
class TrsServerAnnouncement() : PrimitiveType() {
    var trsUri: Link = Link()
        @OslcName("trs_uri")
        @OslcPropertyDefinition(TrsXConstants.NS + "trs_uri")
        @OslcOccurs(Occurs.ExactlyOne)
        @OslcValueType(ValueType.Resource)
//        @OslcReadOnly(false)
        get

    var twinId: String = ""
        @OslcName("twin_id")
        @OslcPropertyDefinition(TrsXConstants.NS + "twin_id")
        @OslcOccurs(Occurs.ExactlyOne)
        @OslcValueType(ValueType.String)
//        @OslcReadOnly(false)
        get

    var isLeaving: Boolean = false
        // TODO can we make only name mandatory? and infer the NS from @OslcNamespace
        @OslcName("is_leaving")
        @OslcPropertyDefinition(TrsXConstants.NS + "is_leaving")
//        @OslcOccurs(Occurs.ExactlyOne)
        @OslcValueType(ValueType.Boolean)
//        @OslcReadOnly(false)
        get

    @JvmOverloads
    constructor(trsUri: URI, twinId: String, isLeaving: Boolean = false) : this() {
        this.trsUri = Link(trsUri)
        this.twinId = twinId
        this.isLeaving = isLeaving
    }
}
