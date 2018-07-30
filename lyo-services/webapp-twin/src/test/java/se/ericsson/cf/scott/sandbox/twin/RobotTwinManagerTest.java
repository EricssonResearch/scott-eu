package se.ericsson.cf.scott.sandbox.twin;

import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twin.trs.MqttManager;

import static org.assertj.core.api.Assertions.*;

/**
 *
 * @version $version-stub$
 * @since 0.0.1
 */
public class RobotTwinManagerTest {
    private final static Logger log = LoggerFactory.getLogger(RobotTwinManagerTest.class);

    @Before
    public void setUp() throws Exception {
        // TODO Andrew@2018-07-27: mock the ServletContextEvent and invoke ServletListener
        // https://stackoverflow.com/a/22611270/464590
        OSLC4JUtils.setPublicURI("http://localhost/twin/");
        OSLC4JUtils.setServletPath("services/");
        log.info("Public  URI: {}", OSLC4JUtils.getPublicURI());
        log.info("Servlet URI: {}", OSLC4JUtils.getServletURI());
    }

    @Test
    @Ignore("TODO update after refactoring")
    public void twinRegistrationMessage() {
//        final MqttMessage message = MqttManager.getTwinRegistrationMessage();
//        final String actual = message.toString();
//        log.info(actual);
//        assertThat(actual).isNotBlank();
    }
}
