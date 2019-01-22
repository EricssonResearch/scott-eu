package se.ericsson.cf.scott.sandbox.twin.ros;

// TODO Andrew@2019-01-22: clean up
/*import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.Node;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageListener;
import org.ros.concurrent.CancellableLoop;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import turtlebot2i_warehouse.TaskRequest;
import turtlebot2i_warehouse.TaskRequestRequest;
import turtlebot2i_warehouse.TaskRequestResponse;

public class RobotClientNode extends AbstractNodeMain {

    private final static Logger log =
	LoggerFactory.getLogger(RobotClientNode.class);
    private final static String TASK_SVC_NAME = "TaskRequestService";

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("/turtlebot2i_twin");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
            log.info("RobotClientNode started");

	    *//*
	     * Subscribers
	     *//*

	     //  EVENTS

	    // Bumper
	    final Subscriber<kobuki_msgs.BumperEvent> bumperEventSubscriber =
		connectedNode.newSubscriber("/turtlebot2i/events/bumper",
					    kobuki_msgs.BumperEvent._TYPE);
	    bumperEventSubscriber.addMessageListener(new MessageListener<kobuki_msgs.BumperEvent>() {
		    private int count = 0;
		    @Override
		    public void onNewMessage(kobuki_msgs.BumperEvent message) {
			count++;
			if(count%5 == 0){
			    log.info("Bumper Event: \"" + message.getState() + "\"");
			}
		    }
		});

	    // Cliff
	    final Subscriber<kobuki_msgs.CliffEvent> cliffEventSubscriber =
		connectedNode.newSubscriber("/turtlebot2i/events/cliff",
					    kobuki_msgs.CliffEvent._TYPE);
	    cliffEventSubscriber.addMessageListener(new MessageListener<kobuki_msgs.CliffEvent>() {
		    private int count = 0;
		    @Override
		    public void onNewMessage(kobuki_msgs.CliffEvent message) {
			count++;
			if(count%5 == 0){
			    log.info("Cliff Event: \"" + message.getState() + "\"");
			}
		    }
		});

	    // Wheel Drop
	    final Subscriber<kobuki_msgs.WheelDropEvent> wDropEventSubscriber =
		connectedNode.newSubscriber("/turtlebot2i/events/wheel_drop",
					    kobuki_msgs.WheelDropEvent._TYPE);
	    wDropEventSubscriber.addMessageListener(new MessageListener<kobuki_msgs.WheelDropEvent>() {
		    private int count = 0;
		    @Override
		    public void onNewMessage(kobuki_msgs.WheelDropEvent message) {
			count++;
			if(count%5 == 0){
			    log.info("Wheel Drop Event: \"" + message.getState() + "\"");
			}
		    }
		});

	    
	    //  SENSORS

	    // Odometry
	    final Subscriber<nav_msgs.Odometry> odomSubscriber =
		connectedNode.newSubscriber("/turtlebot2i/odom",
					    nav_msgs.Odometry._TYPE);
	    odomSubscriber.addMessageListener(new MessageListener<nav_msgs.Odometry>() {
		    private int count = 0;
		    @Override
		    public void onNewMessage(nav_msgs.Odometry message) {
			//			count++;
			//if(count%5 == 0) {
			    log.info("Odometry: \"" + message.getPose().getPose() + "\"");
			    //}
		    }
		});

	    // Dock IR
	    final Subscriber<kobuki_msgs.DockInfraRed> dockIRSubscriber =
		connectedNode.newSubscriber("/turtlebot2i/sensors/dock_ir",
					    kobuki_msgs.DockInfraRed._TYPE);
	    dockIRSubscriber.addMessageListener(new MessageListener<kobuki_msgs.DockInfraRed>() {
		    private int count = 0;
		    @Override
		    public void onNewMessage(kobuki_msgs.DockInfraRed message) {
			count++;
			if(count%5 == 0){
			    log.info("Dock IR Sensor: \"" + message + "\"");
			}
		    }
		});

	    
	    //  ACTIONS

	    // Velocity
	    *//*	    final Subscriber<geometry_msgs.Twist> velSubscriber =
		connectedNode.newSubscriber("/turtlebot2i/commands/velocity",
					    geometry_msgs.Twist._TYPE);
	    velSubscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
		    @Override
		    public void onNewMessage(geometry_msgs.Twist message) {
			log.info("Velocity: \"" + message + "\"");
		    }
		});

	    // Motor Power
	    final Subscriber<kobuki_msgs.MotorPower> motorPowerSubscriber =
		connectedNode.newSubscriber("/turtlebot2i/commands/motor_power",
					    kobuki_msgs.MotorPower._TYPE);
	    motorPowerSubscriber.addMessageListener(new MessageListener<kobuki_msgs.MotorPower>() {
		    @Override
		    public void onNewMessage(kobuki_msgs.MotorPower message) {
			log.info("Motor: \"" + message.getState() + "\"");
		    }
		    });*//*

	    *//*
	     * Publishers
	     *//*

	    // velocity
	    	    final Publisher<geometry_msgs.Twist> velPub =
		connectedNode.newPublisher("/turtlebot2i/commands/velocity",
					   geometry_msgs.Twist._TYPE);
	    
	    connectedNode.executeCancellableLoop(new CancellableLoop() {
		    private int sequenceNumber;

		    @Override
		    protected void setup() {
			sequenceNumber = 0;
		    }

		    @Override
		    protected void loop() throws InterruptedException {
			geometry_msgs.Twist twt = velPub.newMessage();
			twt.getLinear().setX(2);
			velPub.publish(twt);
			sequenceNumber++;
			Thread.sleep(5000);
			twt.getLinear().setX(0);
			twt.getAngular().setZ(1);
			velPub.publish(twt);
			Thread.sleep(3000);
			twt.getLinear().setX(0);
			twt.getAngular().setZ(0);
			velPub.publish(twt);
		    }
		});
	    *//*serviceClient = connectedNode.newServiceClient(TASK_SVC_NAME, TaskRequest._TYPE);
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
		}); *//*
	    *//*} catch (ServiceNotFoundException e) {
            log.error("ROS exception", e);
	    }*//*
    }

    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }
}*/
