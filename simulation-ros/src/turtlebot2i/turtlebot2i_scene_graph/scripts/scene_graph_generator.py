#!/usr/bin/env python

import sys
import argparse
import rospy
from std_msgs.msg import Header
from turtlebot2i_scene_graph import VrepObjectExtractor, SceneGraphGenerator
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_scene_graph.srv import GenerateSceneGraph, GenerateSceneGraphResponse


def make_message(scene_graph):
    scene_graph_msg = SceneGraph()
    scene_graph_msg.header = Header()
    scene_graph_msg.header.stamp = rospy.Time.now()
    scene_graph_msg.sg_data = scene_graph.source
    return scene_graph_msg


def generate_scene_graph(generator):
    scene_graph = generator.generate_scene_graph()
    scene_graph = make_message(scene_graph)
    rospy.loginfo('Scene graph generated')
    return GenerateSceneGraphResponse(scene_graph)


def publish_scene_graph(generator, publisher):
    scene_graph = generator.generate_scene_graph()
    scene_graph = make_message(scene_graph)
    publisher.publish(scene_graph)


def main():
    rospy.init_node('scene_graph_generator')

    rospy.loginfo('Parsing command line arguments...')
    argv = rospy.myargv(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', type=str, choices=['service', 'topic'])
    args = parser.parse_args(argv[1:])
    rospy.loginfo('Mode: %s' % args.mode)

    rospy.loginfo('Getting V-REP parameters...')
    host = rospy.get_param('~vrep/host')
    port = rospy.get_param('~vrep/port')
    walls = rospy.get_param('/vrep/walls')
    robots = rospy.get_param('/vrep/robots')
    static_objects = rospy.get_param('/vrep/static_objects')
    dynamic_objects = rospy.get_param('/vrep/dynamic_objects')
    robot = rospy.get_param('~robot/name')
    if '_' in robot:
        robot = robot.split('_')[0] + '#' + robot.split('_')[1]
    rospy.loginfo('V-REP remote API server: %s:%d' % (host, port))
    rospy.loginfo('Robot: %s' % robot)

    rospy.loginfo('Connecting to V-REP, can take a while...')
    extractor = VrepObjectExtractor()
    extractor.start_connection(host, port)
    extractor.load_objects(walls, robots, static_objects, dynamic_objects)
    rospy.loginfo('Initialization completed')

    generator = SceneGraphGenerator(robot, extractor)

    if args.mode == 'service':
        rospy.Service(
            name='generate_scene_graph',
            service_class=GenerateSceneGraph,
            handler=lambda request: generate_scene_graph(generator),
            buff_size=2 ** 20   # 1 MB, enough for camera images
        )
        rospy.loginfo('ROS service to generate scene graph ready')
        rospy.spin()

    elif args.mode == 'topic':
        rospy.loginfo('Publishing scene graph...')
        publisher = rospy.Publisher('scene_graph', SceneGraph, queue_size=1)
        rate = rospy.Rate(30)   # 30 FPS
        try:
            while not rospy.is_shutdown():
                publish_scene_graph(generator, publisher)
                rate.sleep()
        except rospy.ROSException:
            pass

    else:
        raise ValueError


if __name__ == '__main__':
    main()
