package eu.scott.warehouse

/**
 * Created on 2018-07-27
 *
 * @author  Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since   0.0.1
 */

object MqttTopics {
    const val REGISTRATION_ANNOUNCE = "_scott/whc/registration/announce"
    const val REGISTRATION_ACK = "_scott/whc/registration/ack"
    const val WHC_PLANS = "_scott/whc/plans"
    const val TWIN_SIM_REG_ANNOUNCE = "twins/registration/announce"
    const val TWIN_SIM_REG_ACK = "twins/registration/handshake"

}
