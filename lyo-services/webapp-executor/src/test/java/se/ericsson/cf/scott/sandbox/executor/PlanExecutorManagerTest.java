package se.ericsson.cf.scott.sandbox.executor;

import org.junit.Test;

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
        final String testKey = ExecutorServiceHelper.pFQDN("test");

        assertThat(testKey).isEqualTo("se.ericsson.cf.scott.sandbox.executor.test");
    }
}
