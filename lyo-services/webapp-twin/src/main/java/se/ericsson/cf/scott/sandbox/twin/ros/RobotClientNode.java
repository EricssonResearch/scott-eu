package se.ericsson.cf.scott.sandbox.twin.ros;

// FIXME Andrew@2018-07-18: bring the ROS part back and allow it be turned on/off
//import org.ros.exception.RemoteException;
//import org.ros.exception.RosRuntimeException;
//import org.ros.exception.ServiceNotFoundException;
//import org.ros.message.Time;
//import org.ros.namespace.GraphName;
//import org.ros.node.AbstractNodeMain;
//import org.ros.node.ConnectedNode;
//import org.ros.node.service.ServiceClient;
//import org.ros.node.service.ServiceResponseListener;
//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;
//import turtlebot2i_warehouse.TaskRequest;
//import turtlebot2i_warehouse.TaskRequestRequest;
//import turtlebot2i_warehouse.TaskRequestResponse;
//
//public class RobotClientNode extends AbstractNodeMain {
//
//    private final static Logger log           = LoggerFactory.getLogger(RobotClientNode.class);
//    private final static String TASK_SVC_NAME = "TaskRequestService";
//
//    @Override
//    public GraphName getDefaultNodeName() {
//        return GraphName.of("rosjava_tutorial_services/client");
//    }
//
//    @Override
//    public void onStart(final ConnectedNode connectedNode) {
//        ServiceClient<TaskRequestRequest, TaskRequestResponse> serviceClient;
//        try {
//            log.info("RobotClientNode started");
//            serviceClient = connectedNode.newServiceClient(TASK_SVC_NAME, TaskRequest._TYPE);
//
//            final TaskRequestRequest request = serviceClient.newMessage();
//            request.setRobot("RobotA");
//            request.setStamp(new Time());
//
//            serviceClient.call(request, new ServiceResponseListener<TaskRequestResponse>() {
//                @Override
//                public void onSuccess(TaskRequestResponse response) {
//                    log.info(
//                            "Status:\n\tPose: {}\n\tTask: {}\n\tObject: {}\n\tStamp: {}",
//                            response.getWaypoint(),
//                            response.getTask(),
//                            response.getObject(),
//                            response.getStamp());
//                }
//
//                @Override
//                public void onFailure(RemoteException e) {
//                    throw new RosRuntimeException(e);
//                }
//            });
//        } catch (ServiceNotFoundException e) {
//            log.error("ROS exception", e);
//        }
//    }
//}
