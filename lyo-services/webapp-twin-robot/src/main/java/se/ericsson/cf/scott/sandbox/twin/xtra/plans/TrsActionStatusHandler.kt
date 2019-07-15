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

import eu.scott.warehouse.domains.scott.DropBelt
import eu.scott.warehouse.domains.scott.ExecutableAction
import eu.scott.warehouse.domains.scott.IExecutableAction
import eu.scott.warehouse.domains.scott.MoveToWp
import eu.scott.warehouse.domains.scott.PickShelf
import se.ericsson.cf.scott.sandbox.twin.xtra.trs.ITrsLogAppender

class TrsActionStatusHandler(private val logAppender: ITrsLogAppender) : IActionStatusHandler {
    override fun actionFailed(action: IExecutableAction) {
        TODO("not implemented")
    }

    override fun actionCompleted(action: IExecutableAction) {
        when (action) {
            is PickShelf -> pickShelfCompleted(action)
            is MoveToWp  -> moveToWpCompleted(action)
            is DropBelt  -> dropBeltCompleted(action)
        }
    }

    private fun dropBeltCompleted(action: DropBelt) {
        appendCompletionEvent(action)
    }

    private fun moveToWpCompleted(action: MoveToWp) {
        appendCompletionEvent(action)
    }

    private fun pickShelfCompleted(action: PickShelf) {
        appendCompletionEvent(action)
    }

    private fun appendCompletionEvent(action: ExecutableAction) {
        logAppender.appendModificationEvent(action.about)
    }

}
