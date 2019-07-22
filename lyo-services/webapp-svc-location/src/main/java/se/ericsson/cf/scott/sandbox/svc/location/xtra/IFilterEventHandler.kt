package se.ericsson.cf.scott.sandbox.svc.location.xtra

import org.eclipse.lyo.core.trs.ChangeEvent
import org.eclipse.lyo.trs.client.model.ChangeEventMessageTR

interface IFilterEventHandler {
    fun canHandle(topic:String, event: ChangeEvent): Boolean
    fun handle(topic: String, fullEvent: ChangeEventMessageTR)
}
