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

package se.ericsson.cf.scott.sandbox.twin.xtra.trs

import org.eclipse.lyo.core.trs.Creation
import org.eclipse.lyo.core.trs.Deletion
import org.eclipse.lyo.core.trs.Modification
import java.net.URI

interface ITrsLogAppender {
    fun appendCreationEvent(changed: URI, twinKind: String, twinId: String): Creation
    fun appendModificationEvent(changed: URI, twinKind: String, twinId: String): Modification
    fun appendDeletionEvent(changed: URI, twinKind: String, twinId: String): Deletion

    companion object {
        val pageSize = 200
        fun logGraphFor(twinKind: String, twinId: String, orderId: Int): URI {
            val page = pageNo(orderId)
            return URI.create("http://twins.svc/ng/trs/log/$twinKind/$twinId/$page")
        }

        private fun pageNo(orderId: Int) = orderId / pageSize + 1
    }
}
