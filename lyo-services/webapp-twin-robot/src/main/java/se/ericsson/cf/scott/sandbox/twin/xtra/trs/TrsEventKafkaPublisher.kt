package se.ericsson.cf.scott.sandbox.twin.xtra.trs

import eu.scott.warehouse.lib.toTurtleString
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
        producer.send(ProducerRecord(topic, event.about.toString(), event.toTurtleString))
    }
}
