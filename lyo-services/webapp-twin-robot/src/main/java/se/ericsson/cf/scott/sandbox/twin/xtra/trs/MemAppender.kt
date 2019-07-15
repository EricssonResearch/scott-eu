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

package se.ericsson.cf.scott.sandbox.twin.xtra.trs

import org.eclipse.lyo.core.trs.Creation
import org.eclipse.lyo.core.trs.Deletion
import org.eclipse.lyo.core.trs.Modification
import java.net.URI

class MemAppender : ITrsLogAppender {
    override fun appendCreationEvent(changed: URI): Creation {
        TODO("not implemented")
    }

    override fun appendModificationEvent(changed: URI): Modification {
        TODO("not implemented")
    }

    override fun appendDeletionEvent(changed: URI): Deletion {
        TODO("not implemented")
    }
}
