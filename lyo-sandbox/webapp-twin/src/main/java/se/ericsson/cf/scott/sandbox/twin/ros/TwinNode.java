package se.ericsson.cf.scott.sandbox.twin.ros;

import org.apache.jena.sparql.function.library.leviathan.log;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created on 2018-05-09
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class TwinNode implements NodeMain {
    private final static Logger log = LoggerFactory.getLogger(TwinNode.class);
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("RobotTwin");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        log.info("Robot Twin ROS node started");
    }

    @Override
    public void onShutdown(final Node node) {
        log.warn("Robot Twin ROS node is shutting down");
    }

    @Override
    public void onShutdownComplete(final Node node) {
        log.info("Robot Twin ROS node has shut down");
    }

    @Override
    public void onError(final Node node, final Throwable throwable) {
        log.warn("Robot Twin ROS node '{}' has encountered an error: {}", node, throwable);
    }
}
