package se.ericsson.cf.scott.sandbox.executor;

import org.junit.Test;
import se.ericsson.cf.scott.sandbox.executor.xtra.ExecutorServiceHelper;

import static org.assertj.core.api.Assertions.*;

public class PlanExecutorManagerTest {

    @Test
    public void pFQDN() {
        final String testKey = ExecutorServiceHelper.pFQDN("test");

        assertThat(testKey).isEqualTo("se.ericsson.cf.scott.sandbox.executor.test");
    }
}
