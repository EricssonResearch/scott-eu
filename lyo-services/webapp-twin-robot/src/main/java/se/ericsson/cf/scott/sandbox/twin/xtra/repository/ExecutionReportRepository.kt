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

package se.ericsson.cf.scott.sandbox.twin.xtra.repository

import eu.scott.warehouse.domains.scott.ActionExecutionReport
import org.eclipse.lyo.store.Store
import se.ericsson.cf.scott.sandbox.twin.TwinResourcesFactory
import java.net.URI

interface IExecutionRepository {
    fun add(report: ActionExecutionReport, twinKind: String, twinId: String)
    fun getById(id: String, twinKind: String, twinId: String): ActionExecutionReport
}

class ExecutionReportRepository(private val store: Store) : IExecutionRepository {
    override fun add(report: ActionExecutionReport, twinKind: String, twinId: String) {
        val graphUri = graphUriFor(twinKind, twinId)
        val success = store.appendResource(graphUri, report)
    }

    override fun getById(id: String, twinKind: String, twinId: String): ActionExecutionReport {
        val graphUri = graphUriFor(twinKind, twinId)
        val reportUri = TwinResourcesFactory.constructURIForActionExecutionReport(twinKind, twinId,
            id)
        return store.getResource(graphUri, reportUri, ActionExecutionReport::class.java)
    }

    private fun graphUriFor(twinKind: String, twinId: String): URI =
        URI.create("http://twins.local/$twinKind/$twinId")
}

