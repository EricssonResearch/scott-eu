package eu.scott.warehouse.domains.trs

import eu.scott.warehouse.domains.pddl.PrimitiveType
import org.eclipse.lyo.oslc4j.core.annotation.*
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.core.model.Occurs
import org.eclipse.lyo.oslc4j.core.model.ValueType
import java.net.URI

/**
 * TODO
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
        get

    var adaptorId: String = ""
        @OslcName("adaptor_id") @OslcPropertyDefinition(TrsXConstants.NS + "adaptor_id")
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

    var kind: String = ""
        @OslcName("kind")
        @OslcPropertyDefinition(TrsXConstants.NS + "kind")
//        @OslcOccurs(Occurs.ExactlyOne)
        @OslcValueType(ValueType.String)
        get

    var lwtTopic: String = ""
        @OslcName("lwt_topic") @OslcPropertyDefinition(TrsXConstants.NS + "lwt_topic") @OslcOccurs(
                Occurs.ZeroOrOne) @OslcValueType(ValueType.String) get

    @JvmOverloads constructor(twinId: String, kind: String = TrsXConstants.TYPE_TWIN, trsUri: URI,
                              lwtTopic: String, isLeaving: Boolean = false) : this() {
        this.trsUri = Link(trsUri)
        this.kind = kind
        this.adaptorId = twinId
        this.isLeaving = isLeaving
        this.lwtTopic = lwtTopic
    }

    override fun toString(): String {
        return "TrsServerAnnouncement(id=${adaptorId}, kind=${kind}, trsURI=${trsUri}, lwtTopic=${lwtTopic}, isLeaving=${isLeaving})"
    }


}
