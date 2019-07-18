/*
 * Copyright 2019 Ericsson Research and others.
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

package se.ericsson.cf.scott.sandbox.twin.xtra.plans

import eu.scott.warehouse.domains.scott.ActionExecutionReport
import eu.scott.warehouse.domains.scott.DropBelt
import eu.scott.warehouse.domains.scott.ExecutableAction
import eu.scott.warehouse.domains.scott.MoveToWp
import eu.scott.warehouse.domains.scott.PickShelf
import eu.scott.warehouse.lib.toModel
import se.ericsson.cf.scott.sandbox.twin.TwinResourcesFactory
import se.ericsson.cf.scott.sandbox.twin.xtra.repository.ExecutionReportRepository
import se.ericsson.cf.scott.sandbox.twin.xtra.trs.ITrsLogAppender
import java.util.Date
import java.util.UUID

class TrsActionStatusHandler(private val logAppender: ITrsLogAppender,
                             private val reportRepository: ExecutionReportRepository) :
    IActionStatusHandler {
    override fun actionFailed(action: ExecutableAction, info: ExecutionInfo) {
        TODO("not implemented")
    }

    override fun actionCompleted(action: ExecutableAction, info: ExecutionInfo) {
        when (action) {
            is PickShelf -> pickShelfCompleted(action, info)
            is MoveToWp  -> moveToWpCompleted(action, info)
            is DropBelt  -> dropBeltCompleted(action, info)
        }
    }

    private fun dropBeltCompleted(action: DropBelt, info: ExecutionInfo) {
        appendCompletionEvent(action, info)
    }

    private fun moveToWpCompleted(action: MoveToWp, info: ExecutionInfo) {
        appendCompletionEvent(action, info)
    }

    private fun pickShelfCompleted(action: PickShelf, info: ExecutionInfo) {
        appendCompletionEvent(action, info)
    }

    private fun appendCompletionEvent(action: ExecutableAction, info: ExecutionInfo) {
        val executionReport = generateReport(info, action, success = true)
        reportRepository.add(executionReport, info.twinKind, info.twinId)
        logAppender.appendCreationEvent(executionReport.about, executionReport.toModel, info.twinKind,
            info.twinId)
    }

    private fun generateReport(info: ExecutionInfo, action: ExecutableAction,
                               success: Boolean): ActionExecutionReport {
        val reportId = UUID.randomUUID()
            .toString()
        val executionReport = TwinResourcesFactory.createActionExecutionReport(info.twinKind,
            info.twinId, reportId)
        executionReport.action = action
        executionReport.isExecutionSuccess = success
        executionReport.executionBegin = info.executionBegin
        executionReport.executionEnd = Date()
        return executionReport
    }
}

