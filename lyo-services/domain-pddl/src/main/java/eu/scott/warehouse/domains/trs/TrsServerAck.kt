package eu.scott.warehouse.domains.trs

import eu.scott.warehouse.domains.blocksworld.IBlock
import eu.scott.warehouse.domains.pddl.PddlDomainConstants
import eu.scott.warehouse.domains.pddl.PrimitiveType
import org.eclipse.lyo.oslc4j.core.annotation.*
import org.eclipse.lyo.oslc4j.core.model.Link
import org.eclipse.lyo.oslc4j.core.model.Occurs
import org.eclipse.lyo.oslc4j.core.model.ValueType
import java.net.URI

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

    var twinId: String = ""
        @OslcName("twin_id")
        @OslcPropertyDefinition(TrsXConstants.NS + "twin_id")
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
        this.twinId = twinId
        this.trsTopic = trsTopic
    }
}
