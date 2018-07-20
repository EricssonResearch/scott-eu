package se.ericsson.cf.scott.sandbox.twin.ros;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import turtlebot2i_warehouse.TaskRequest;
import turtlebot2i_warehouse.TaskRequestRequest;
import turtlebot2i_warehouse.TaskRequestResponse;

public class RobotClientNode extends AbstractNodeMain {

    private final static Logger log           = LoggerFactory.getLogger(RobotClientNode.class);
    private final static String TASK_SVC_NAME = "TaskRequestService";

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("/turtlebot2i_twin");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
            log.info("RobotClientNode started");

	    /*
	     *  EVENTS
	     */

	    // Bumper
	    Subscriber<kobuki_msgs.BumperEvent> bumperEventSubscriber = connectedNode.newSubscriber("/turtlebot2i/events/bumper", kobuki_msgs.BumperEvent._TYPE);
	    bumperEventSubscriber.addMessageListener(new MessageListener<kobuki_msgs.BumperEvent>() {
		    @Override
		    public void onNewMessage(kobuki_msgs.BumperEvent message) {
			log.info("Bumper Event: \"" + message.getState() + "\"");
		    }
		});

	    // Cliff
	    Subscriber<kobuki_msgs.CliffEvent> cliffEventSubscriber = connectedNode.newSubscriber("/turtlebot2i/events/cliff", kobuki_msgs.CliffEvent._TYPE);
	    cliffEventSubscriber.addMessageListener(new MessageListener<kobuki_msgs.CliffEvent>() {
		    @Override
		    public void onNewMessage(kobuki_msgs.CliffEvent message) {
			log.info("Cliff Event: \"" + message.getState() + "\"");
		    }
		});

	    // Wheel Drop
	    Subscriber<kobuki_msgs.WheelDropEvent> wDropEventSubscriber = connectedNode.newSubscriber("/turtlebot2i/events/wheel_drop", kobuki_msgs.WheelDropEvent._TYPE);
	    wDropEventSubscriber.addMessageListener(new MessageListener<kobuki_msgs.WheelDropEvent>() {
		    @Override
		    public void onNewMessage(kobuki_msgs.WheelDropEvent message) {
			log.info("Wheel Drop Event: \"" + message.getState() + "\"");
		    }
		});

	    /*
	     *  SENSORS
	     */

	    // Odometry
	    Subscriber<nav_msgs.Odometry> odomSubscriber = connectedNode.newSubscriber("/turtlebot2i/odom", nav_msgs.Odometry._TYPE);
	    odomSubscriber.addMessageListener(new MessageListener<nav_msgs.Odometry>() {
		    @Override
		    public void onNewMessage(nav_msgs.Odometry message) {
			System.out.println("Odometry: \"" + message.getPose().getPose() + "\"");
		    }
		});

	    // Dock IR
	    /*	    Subscriber<kobuki_msgs.DockInfraRed> dockIRSubscriber = connectedNode.newSubscriber("/turtlebot2i/sensors/dock_ir", kobuki_msgs.DockInfraRed._TYPE);
	    dockIRSubscriber.addMessageListener(new MessageListener<kobuki_msgs.DockInfraRed>() {
		    @Override
		    public void onNewMessage(kobuki_msgs.DockInfraRed message) {
			log.info("Dock IR Sensor: \"" + message.getData() + "\"");
		    }
		    });*/

	    /*
	     *  ACTIONS
	     */

	    // Velocity
	    Subscriber<geometry_msgs.Twist> velSubscriber = connectedNode.newSubscriber("/turtlebot2i/commands/velocity", geometry_msgs.Twist._TYPE);
	    velSubscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
		    @Override
		    public void onNewMessage(geometry_msgs.Twist message) {
			log.info("Velocity: \"" + message + "\"");
		    }
		});

	    // Motor Power
	    Subscriber<kobuki_msgs.MotorPower> motorPowerSubscriber = connectedNode.newSubscriber("/turtlebot2i/commands/motor_power", kobuki_msgs.MotorPower._TYPE);
	    motorPowerSubscriber.addMessageListener(new MessageListener<kobuki_msgs.MotorPower>() {
		    @Override
		    public void onNewMessage(kobuki_msgs.MotorPower message) {
			log.info("Motor: \"" + message.getState() + "\"");
		    }
		});

	    
	    
	    /*serviceClient = connectedNode.newServiceClient(TASK_SVC_NAME, TaskRequest._TYPE);
            final TaskRequestRequest request = serviceClient.newMessage();
            request.setRobot("RobotA");
            request.setStamp(new Time());

            serviceClient.call(request, new ServiceResponseListener<TaskRequestResponse>() {
                @Override
                public void onSuccess(TaskRequestResponse response) {
                    log.info(
                            "Status:\n\tPose: {}\n\tTask: {}\n\tObject: {}\n\tStamp: {}",
                            response.getWaypoint(),
                            response.getTask(),
                            response.getObject(),
                            response.getStamp());
			    }
	    

                @Override
                public void onFailure(RemoteException e) {
                    throw new RosRuntimeException(e);
                }
		}); */
	    /*} catch (ServiceNotFoundException e) {
            log.error("ROS exception", e);
	    }*/
    }
}
