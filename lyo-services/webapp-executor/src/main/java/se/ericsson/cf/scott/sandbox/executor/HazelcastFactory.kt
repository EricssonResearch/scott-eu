package se.ericsson.cf.scott.sandbox.executor

import com.hazelcast.config.Config
import com.hazelcast.core.Hazelcast
import com.hazelcast.core.HazelcastInstance
import java.util.*
import com.hazelcast.config.ClasspathXmlConfig
import com.hazelcast.instance.DefaultNodeContext
import com.hazelcast.instance.HazelcastInstanceFactory
import org.slf4j.LoggerFactory
import java.io.Serializable


/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
class HazelcastFactory {
    val log = LoggerFactory.getLogger(javaClass)
    var hazelcastInstance: HazelcastInstance?
        private set

    init {
//        val config = Config()
//        config.groupConfig.name = "executors"
//        config.networkConfig.join.multicastConfig.isEnabled = true;
//        config.networkConfig.join.multicastConfig.multicastGroup = "224.0.0.1";
//        config.networkConfig.join.multicastConfig.multicastGroup = "224.0.0.1";
//
//        config.networkConfig.

        fixSwarmMemberAddressProvider()

        val config = ClasspathXmlConfig("hazelcast.xml")

        log.info("Starting the Hazelcast config {}", config)

        hazelcastInstance = HazelcastInstanceFactory.newHazelcastInstance(config, "executor",
                DefaultNodeContext())


    }

    private fun fixSwarmMemberAddressProvider() {
        // See https://github.com/bitsofinfo/hazelcast-docker-swarm-discovery-spi/blob/46a1224b379cf136d5314a79aa1502209c6d1dc2/src/main/java/org/bitsofinfo/hazelcast/discovery/docker/swarm/SwarmMemberAddressProvider.java#L58
        // the variable is not read from the ENV but from the JVM system props
//        System.setProperty("hazelcastPeerPort", "5701")
//        System.setProperty("swarmMgrUri", "http://192.168.65.3:2377")
    }
}

data class HCData(val url: String, val uuid: String, val headers: Array<String>) : Serializable {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is HCData) return false

        if (url != other.url) return false
        if (uuid != other.uuid) return false
        if (!Arrays.equals(headers, other.headers)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = url.hashCode()
        result = 31 * result + uuid.hashCode()
        result = 31 * result + Arrays.hashCode(headers)
        return result
    }
}
