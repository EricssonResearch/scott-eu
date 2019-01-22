package eu.scott.warehouse.lib.hazelcast

/*
@Deprecated("Phasing Hazelcast out of the SCOTT sandbox")
data class HCData(val url: String, val uuid: String, val headers: Array<String>) :
    Serializable {
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
*/
