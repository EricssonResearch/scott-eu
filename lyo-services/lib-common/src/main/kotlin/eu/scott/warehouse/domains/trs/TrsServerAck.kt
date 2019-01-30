package eu.scott.warehouse.domains.trs

import eu.scott.warehouse.domains.pddl.PrimitiveType
import org.eclipse.lyo.oslc4j.core.annotation.*
import org.eclipse.lyo.oslc4j.core.model.Occurs
import org.eclipse.lyo.oslc4j.core.model.ValueType

/**
 * Acknowledging the 
 *
 * @version $version-stub$
 * @since   0.0.1
 */

@OslcNamespace(TrsXConstants.NS)
@OslcName("TrsServerAck")
@OslcResourceShape(describes = [(TrsXConstants.NS + "TrsServerAck")])
class TrsServerAck() : PrimitiveType() {

    var adaptorId: String = ""
        @OslcName("adaptor_id")
        @OslcPropertyDefinition(TrsXConstants.NS + "adaptor_id")
        @OslcOccurs(Occurs.ExactlyOne)
        @OslcValueType(ValueType.String)
        get

    var trsTopic: String = ""
        @OslcName("trs_topic")
        @OslcPropertyDefinition(TrsXConstants.NS + "trs_topic")
        @OslcOccurs(Occurs.ExactlyOne)
        @OslcValueType(ValueType.String)
        get


    constructor(twinId: String, trsTopic: String) : this() {
        this.adaptorId = twinId
        this.trsTopic = trsTopic
    }
}
