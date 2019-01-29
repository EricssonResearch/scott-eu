package se.ericsson.cf.scott.sandbox.executor;

import org.junit.Test;

import static org.junit.Assert.*;
import static org.assertj.core.api.Assertions.*;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class PlanExecutorManagerTest {

    @Test
    public void pFQDN() {
        final String testKey = PlanExecutorManager.pFQDN("test");

        assertThat(testKey).isEqualTo("se.ericsson.cf.scott.sandbox.executor.test");
    }
}
