/*
 * Copyright (c) 2019  Ericsson Research and others
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package se.ericsson.cf.scott.sandbox.svc.location.xtra

import eu.scott.warehouse.domains.scott.ActionExecutionReport
import eu.scott.warehouse.domains.scott.MoveToWp
import eu.scott.warehouse.lib.LyoModels
import eu.scott.warehouse.lib.toTurtle
import eu.scott.warehouse.lib.trs.ConcurrentMqttAppender
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper
import org.eclipse.lyo.trs.client.handlers.IPushProviderHandler
import org.eclipse.lyo.trs.client.model.ChangeEventMessageTR
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import javax.xml.namespace.QName


class LocationVicinityFilterHandler(private val appender: ConcurrentMqttAppender) : IPushProviderHandler {
    companion object {
        private val log: Logger = LoggerFactory.getLogger(LocationVicinityFilterHandler::class.java)
    }

    override fun handlePush(message: ChangeEventMessageTR, topic: String) {
        val resourceModel = message.trackedResourceModel
        log.trace("Processing a CE from $topic:\n{}", message.trackedResourceModel.toTurtle)
        val hasResource = LyoModels.hasResource(resourceModel, ActionExecutionReport::class.java)
        if (hasResource) {
            log.trace("An action execution report found")
            val report = JenaModelHelper.unmarshalSingle(resourceModel, ActionExecutionReport::class.java)
            if (report.action is MoveToWp) {
                val from = report.action.extendedProperties[QName.valueOf(
                    "http://ontology.cf.ericsson.net/ns/scott-warehouse/move-to-wp_from")]
                val to = report.action.extendedProperties[QName.valueOf(
                    "http://ontology.cf.ericsson.net/ns/scott-warehouse/" + "move-to-wp_to")]
                val robot = report.action.extendedProperties[QName.valueOf(
                    "http://ontology.cf.ericsson.net/ns/scott-warehouse/" + "move-to-wp_rb")]
                log.debug("Forwarding a change event for the MoveToWp[$robot: $from -> $to] action report")
                appender.forwardEvent(message.changeEvent, message.trackedResourceModel, "scott/trs/controller/all")
            }
        } /*else if (LyoModels.hasResource(message.trackedResourceModel, LocationReport::class.java)) {
            // FIXME Andrew@2019-07-19: model LocationReport
            log.debug("CE with an location update report found")
        }*/
    }
}
