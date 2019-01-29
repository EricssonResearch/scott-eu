package se.ericsson.cf.scott.sandbox.twin.ros;

// TODO Andrew@2019-01-22: clean up
/*
import java.net.URI;
import org.jetbrains.annotations.NotNull;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

*/
/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 *//*

public class RosManager {
    private final static Logger log = LoggerFactory.getLogger(RosManager.class);

    public static void runRosNode() {
        log.warn("ROS node code is commented out");
        try {
            final NodeMainExecutor executor = DefaultNodeMainExecutor.newDefault();
            final URI masterUri = getMasterUri();
            log.warn("ROS functionality is off");
//            executor.execute(new RobotClientNode(), NodeConfiguration.newPublic("localhost",
//                                                                                masterUri));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @NotNull
    private static URI getMasterUri() {
//        return URI.create("http://localhost:11311");
        final String rosMasterUri = System.getenv("ROS_MASTER_URI");
        return URI.create(rosMasterUri);
    }
}
*/
