#!/usr/bin/env python

import rospy
import time
from pympler.asizeof import asizeof
from turtlebot2i_scene_graph.srv import GenerateSceneGraph as GenerateSceneGraphOut
from turtlebot2i_edge.srv import GenerateSceneGraph as GenerateSceneGraphIn
from turtlebot2i_edge.srv import GenerateSceneGraphResponse, Stamp


class ModelPerformance:
    def __init__(self, m_ap, execution_latency):
        self.m_ap = m_ap
        self.execution_latency = rospy.Duration.from_sec(execution_latency)


def get_scene_graph(generate_scene_graph, model_performance, sleep=True):
    time_start = rospy.Time.now()
    response = generate_scene_graph()
    time_elapsed = rospy.Time.now() - time_start

    time_sleep = model_performance.execution_latency - time_elapsed
    time_sleep = time_sleep.to_sec()
    if time_sleep > 0 and sleep:
        time.sleep(time_sleep)

    return response


def on_edge_proxy(generate_scene_graph, upload, download, model_performance, request):
    n_bytes = asizeof(request)
    bytes_ = b'\x00' * n_bytes
    response = upload(bytes=bytes_, max_duration=rospy.Time.from_sec(2))
    if response.stamped != n_bytes:   # timeout
        return None
    upload_latency = rospy.Time.now() - request.header.stamp

    response = get_scene_graph(generate_scene_graph, model_performance, sleep=False)
    scene_graph = response.scene_graph

    n_bytes = asizeof(response)
    bytes_ = b'\x00' * n_bytes
    time_start = rospy.Time.now()
    response = download(bytes=bytes_, max_duration=rospy.Time.from_sec(2))
    if response.stamped != n_bytes:   # timeout
        return None
    download_latency = rospy.Time.now() - time_start

    return GenerateSceneGraphResponse(
        communication_latency=upload_latency + download_latency,
        execution_latency=model_performance.execution_latency,
        model_m_ap=model_performance.m_ap,
        scene_graph=scene_graph
    )


def on_robot_proxy(generate_scene_graph, model_performance, request):
    communication_latency = rospy.Time.now() - request.header.stamp     # inter-process communication

    response = get_scene_graph(generate_scene_graph, model_performance, sleep=False)
    scene_graph = response.scene_graph

    return GenerateSceneGraphResponse(
        communication_latency=communication_latency,
        execution_latency=model_performance.execution_latency,
        model_m_ap=model_performance.m_ap,
        scene_graph=scene_graph
    )


def main():
    rospy.init_node('scene_graph_proxy')

    rospy.loginfo('Getting parameters...')
    m_ap = rospy.get_param('~model/m_ap')
    on_edge = rospy.get_param('~on_edge')
    execution_latency = rospy.get_param('~model/execution_latency')
    rospy.loginfo('Model mAP: %f' % m_ap)
    rospy.loginfo('On edge: %s' % str(on_edge))
    rospy.loginfo('Model execution latency: %f s' % execution_latency)
    model_performance = ModelPerformance(m_ap, execution_latency)

    rospy.loginfo('Waiting for ROS service to generate scene graph...')
    generate_scene_graph = rospy.ServiceProxy('generate_scene_graph', GenerateSceneGraphOut)
    generate_scene_graph.wait_for_service()

    if on_edge:
        rospy.loginfo('Waiting for ROS service to upload...')
        upload = rospy.ServiceProxy('upload', Stamp)
        upload.wait_for_service()

        rospy.loginfo('Waiting for ROS service to download...')
        download = rospy.ServiceProxy('download', Stamp)
        download.wait_for_service()

        def proxy(request):
            return on_edge_proxy(generate_scene_graph, upload, download, model_performance, request)
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
