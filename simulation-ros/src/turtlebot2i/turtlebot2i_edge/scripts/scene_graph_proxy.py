#!/usr/bin/env python

import rospy
from turtlebot2i_scene_graph.srv import GenerateSceneGraph as GenerateSceneGraphOut
from turtlebot2i_edge.srv import GenerateSceneGraph as GenerateSceneGraphIn
from turtlebot2i_edge.srv import GenerateSceneGraphResponse, Stamp


class ModelPerformance:
    def __init__(self, m_ap, execution_latency):
        self.m_ap = m_ap
        self.execution_latency = execution_latency


def on_edge_proxy(stamp, generate_scene_graph, model_performance, request):
    try:
        # communication latency
        time_start = rospy.Time.now()
        response = stamp(bytes=request.data, max_duration=1)
        time_end = response.header.stamp
        communication_latency = time_end - time_start
        if response.stamped != len(request.data):
            return None

        # scene graph
        response = generate_scene_graph()
        scene_graph = response.scene_graph
    except rospy.ROSException, rospy.ServiceException:
        return None

    return GenerateSceneGraphResponse(
        communication_latency=communication_latency,
        execution_latency=model_performance.execution_latency,
        model_m_ap=model_performance.m_ap,
        scene_graph=scene_graph
    )


def on_robot_proxy(generate_scene_graph, model_performance, request):
    communication_latency = request.header.stamp - rospy.Time.now()     # not 0 because of inter-process communication
    response = generate_scene_graph()

    return GenerateSceneGraphResponse(
        communication_latency=communication_latency,
        execution_latency=model_performance.execution_latency,
        model_m_ap=model_performance.m_ap,
        scene_graph=response.scene_graph
    )


def main():
    rospy.init_node('scene_graph_proxy')

    rospy.loginfo('Getting parameters...')
    m_ap = rospy.get_param('~model/m_ap')
    on_edge = rospy.get_param('~on_edge')
    execution_latency = rospy.Duration(
        rospy.get_param('~model/execution_latency/secs'),
        rospy.get_param('~model/execution_latency/nsecs')
    )
    model_performance = ModelPerformance(m_ap, execution_latency)
    rospy.loginfo('Model mAP: %f' % m_ap)
    rospy.loginfo('On edge: %s' % str(on_edge))
    rospy.loginfo('Model execution latency: %f s' % execution_latency.to_sec())

    rospy.loginfo('Waiting for ROS service to generate scene graph...')
    generate_scene_graph = rospy.ServiceProxy('generate_scene_graph', GenerateSceneGraphOut)
    generate_scene_graph.wait_for_service()

    if on_edge:
        rospy.loginfo('Waiting for ROS service to stamp...')
        stamp = rospy.ServiceProxy('edge/stamp', Stamp)
        stamp.wait_for_service()

        def proxy(request):
            return on_edge_proxy(stamp, generate_scene_graph, model_performance, request)
    else:
        def proxy(request):
            return on_robot_proxy(generate_scene_graph, model_performance, request)

    rospy.Service(
        name='generate_scene_graph_proxy',
        service_class=GenerateSceneGraphIn,
        handler=proxy,
        buff_size=2**20     # 1 MB, enough for camera images
    )
    rospy.loginfo('ROS service to generate scene graph with proxy ready')

    rospy.spin()


if __name__ == '__main__':
    main()
