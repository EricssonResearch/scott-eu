/*
 * Copyright (c) 2019 Ericsson Research and others
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

import eu.scott.warehouse.lib.toTurtle
import org.apache.kafka.clients.producer.KafkaProducer
import org.apache.kafka.clients.producer.ProducerRecord
import org.eclipse.lyo.core.trs.ChangeEvent
import java.util.Properties

class TrsEventKafkaPublisher(private val server: String, private val topic: String) {
    private val producer: KafkaProducer<String, String>
    init {
        val props = Properties()
        props.put("bootstrap.servers", server)
        props.put("zookeeper.connect", "zookeeper:2181")
        props.put("acks", "all")
        props.put("key.serializer", "org.apache.kafka.common.serialization.StringSerializer")
        props.put("value.serializer", "org.apache.kafka.common.serialization.StringSerializer")

        producer = KafkaProducer(props)
    }

    fun publish(event: ChangeEvent) {
        producer.send(ProducerRecord(topic, event.about.toString(), event.toTurtle))
    }
}
