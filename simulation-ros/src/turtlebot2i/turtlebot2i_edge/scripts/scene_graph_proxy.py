#!/usr/bin/env python

import rospy
import time
import numpy as np
from pympler.asizeof import asizeof
from turtlebot2i_scene_graph.srv import GenerateSceneGraph
from turtlebot2i_edge.srv import GenerateSceneGraphProxy, GenerateSceneGraphProxyResponse, Stamp


def get_scene_graph(generate_scene_graph, execution_latency):
    time_start = rospy.Time.now()
    response = generate_scene_graph()
    time_elapsed = rospy.Time.now() - time_start

    execution_latency += rospy.Duration.from_sec(np.random.normal(scale=0.1*execution_latency.to_sec()))
    time_sleep = execution_latency - time_elapsed
    time_sleep = time_sleep.to_sec()
    if time_sleep > 0:
        time.sleep(time_sleep)

    return response, execution_latency


def on_edge_proxy(generate_scene_graph, upload, download, execution_latency, request):
    max_latency = request.max_latency

    n_bytes = asizeof(request)
    response = upload(to_stamp=n_bytes, max_duration=max_latency)
    if response.stamped != n_bytes:   # timeout
        return None
    upload_latency = rospy.Time.now() - request.header.stamp

    response, execution_latency = get_scene_graph(generate_scene_graph, execution_latency)
    scene_graph = response.scene_graph

    if upload_latency + execution_latency > max_latency:
        return None
    max_latency -= upload_latency + execution_latency

    n_bytes = asizeof(response)
    time_start = rospy.Time.now()
    response = download(to_stamp=n_bytes, max_duration=max_latency)
    if response.stamped != n_bytes:   # timeout
        return None
    download_latency = rospy.Time.now() - time_start

    return GenerateSceneGraphProxyResponse(
        communication_latency=upload_latency + download_latency,
        execution_latency=execution_latency,
        scene_graph=scene_graph
    )


def on_robot_proxy(generate_scene_graph, execution_latency, request):
    communication_latency = rospy.Time.now() - request.header.stamp     # inter-process communication

    response, execution_latency = get_scene_graph(generate_scene_graph, execution_latency)
    scene_graph = response.scene_graph

    return GenerateSceneGraphProxyResponse(
        communication_latency=communication_latency,
        execution_latency=execution_latency,
        scene_graph=scene_graph
    )


def main():
    rospy.init_node('scene_graph_proxy')

    rospy.loginfo('Getting parameters...')
    on_edge = rospy.get_param('~on_edge')
    execution_latency = rospy.get_param('~model/execution_latency')
    rospy.loginfo('On edge: %s' % str(on_edge))
    rospy.loginfo('Model execution latency: %f s' % execution_latency)
    execution_latency = rospy.Duration.from_sec(execution_latency)

    rospy.loginfo('Waiting for ROS service to generate scene graph...')
    generate_scene_graph = rospy.ServiceProxy('generate_scene_graph', GenerateSceneGraph)
    generate_scene_graph.wait_for_service()

    if on_edge:
        rospy.loginfo('Waiting for ROS service to upload...')
        upload = rospy.ServiceProxy('upload', Stamp)
        upload.wait_for_service()

        rospy.loginfo('Waiting for ROS service to download...')
        download = rospy.ServiceProxy('download', Stamp)
        download.wait_for_service()

        def proxy(request):
            return on_edge_proxy(generate_scene_graph, upload, download, execution_latency, request)
    else:
        def proxy(request):
            return on_robot_proxy(generate_scene_graph, execution_latency, request)

    rospy.Service(
        name='generate_scene_graph_proxy',
        service_class=GenerateSceneGraphProxy,
        handler=proxy,
        buff_size=2**20     # 1 MB, enough for camera images
    )
    rospy.loginfo('ROS service to generate scene graph with proxy ready')

    rospy.spin()


if __name__ == '__main__':
    main()
