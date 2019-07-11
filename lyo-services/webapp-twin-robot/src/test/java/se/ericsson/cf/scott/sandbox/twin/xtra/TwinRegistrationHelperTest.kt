package se.ericsson.cf.scott.sandbox.twin.xtra

import org.assertj.core.api.Assertions.*
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

/**
 * TODO
 *
 * @since TODO
 */
internal class TwinRegistrationHelperTest {

    @Test
    fun stripIdentifierUri() {
        val identifierUri = TwinRegistrationHelper.stripIdentifierUri("/robots/rb-1")
        assertThat(identifierUri).isEqualTo("rb-1")
    }
}
