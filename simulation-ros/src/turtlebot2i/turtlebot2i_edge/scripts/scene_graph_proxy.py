#!/usr/bin/env python

import rospy
from turtlebot2i_scene_graph.srv import GenerateSceneGraph as GenerateSceneGraphOut
from turtlebot2i_edge.srv import GenerateSceneGraph as GenerateSceneGraphIn
from turtlebot2i_edge.srv import GenerateSceneGraphResponse


def proxy(generate_scene_graph, request):
    time_start = rospy.Time.now()
    response = generate_scene_graph()
    time_end = rospy.Time.now()

    communication_latency = time_start - request.header.stamp
    execution_latency = time_end - time_start

    return GenerateSceneGraphResponse(
        communication_latency=communication_latency,
        execution_latency=execution_latency,
        scene_graph=response.scene_graph
    )


def main():
    rospy.init_node('scene_graph_proxy')

    rospy.loginfo('Waiting for ROS service to generate scene graph...')
    generate_scene_graph = rospy.ServiceProxy('generate_scene_graph', GenerateSceneGraphOut)
    generate_scene_graph.wait_for_service()

    rospy.Service(
        name='generate_scene_graph_proxy',
        service_class=GenerateSceneGraphIn,
        handler=lambda request: proxy(generate_scene_graph, request),
        buff_size=2**20     # 1 MB, enough for camera images
    )
    rospy.loginfo('ROS service to generate scene graph with proxy ready')

    rospy.spin()


if __name__ == '__main__':
    main()
