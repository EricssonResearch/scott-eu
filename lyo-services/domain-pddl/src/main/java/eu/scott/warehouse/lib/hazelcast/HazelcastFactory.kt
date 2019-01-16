package eu.scott.warehouse.lib.hazelcast

import com.hazelcast.core.HazelcastInstance
import com.hazelcast.config.ClasspathXmlConfig
import com.hazelcast.instance.DefaultNodeContext
import com.hazelcast.instance.HazelcastInstanceFactory
import org.slf4j.LoggerFactory

@Deprecated("Phasing Hazelcast out of the SCOTT sandbox")
object HazelcastFactory {
    val log = LoggerFactory.getLogger(javaClass)
    @JvmOverloads
    fun instanceFromDefaultXmlConfig(name: String = ""): HazelcastInstance {
        fixSwarmMemberAddressProvider()

        val config = ClasspathXmlConfig("hazelcast.xml")

        log.info("Starting the Hazelcast config {}", config)

        return HazelcastInstanceFactory.newHazelcastInstance(config, name, DefaultNodeContext())
    }


    private fun fixSwarmMemberAddressProvider() {
        // See https://github.com/bitsofinfo/hazelcast-docker-swarm-discovery-spi/blob/46a1224b379cf136d5314a79aa1502209c6d1dc2/src/main/java/org/bitsofinfo/hazelcast/discovery/docker/swarm/SwarmMemberAddressProvider.java#L58
        // the variable is not read from the ENV but from the JVM system props
//        System.setProperty("hazelcastPeerPort", "5701")
//        System.setProperty("swarmMgrUri", "http://192.168.65.3:2377")
    }
}

