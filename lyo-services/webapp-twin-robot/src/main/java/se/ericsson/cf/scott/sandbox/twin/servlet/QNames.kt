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
package se.ericsson.cf.scott.sandbox.twin.servlet

import java.net.URI
import javax.xml.namespace.QName
import org.eclipse.lyo.oslc4j.core.model.PrefixDefinition

object QNames {


    @JvmStatic
    val trsPrefix = PrefixDefinition("trs", URI.create("http://open-services.net/ns/core/trs#"))

    @JvmStatic
    val trsxPrefix = PrefixDefinition("trsx", URI.create("http://md.kth.se/ns/trsx#"))

    @JvmStatic
    fun trs(property: String): QName = ns(property, trsPrefix)

    @JvmStatic
    fun trsx(property: String): QName = ns(property, trsxPrefix)

    private fun ns(property: String, definition: PrefixDefinition): QName =
        QName(definition.prefixBase.toString(), property, definition.prefix)
}
