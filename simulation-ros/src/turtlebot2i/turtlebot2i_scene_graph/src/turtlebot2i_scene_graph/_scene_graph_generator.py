import rospy
import re
from graphviz import Digraph
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from turtlebot2i_scene_graph._utils import get_distance_bbox, get_velocity, get_direction, get_type, get_orientation
from turtlebot2i_scene_graph._utils import get_size, get_overlap_bbox
from turtlebot2i_scene_graph.msg import SceneGraph
from turtlebot2i_scene_graph.srv import GenerateSceneGraph, GenerateSceneGraphResponse


def _prepare_message(scene_graph):
    scene_graph_msg = SceneGraph()
    scene_graph_msg.header = Header()
    scene_graph_msg.header.stamp = rospy.Time.now()
    scene_graph_msg.sg_data = scene_graph.source
    return scene_graph_msg


class SceneGraphGenerator:
    def __init__(self, robot, extractor, mode='service'):
        self.robot = robot
        self.extractor = extractor

        if mode == 'service':
            rospy.loginfo('Starting ROS service...')
            rospy.Service('generate_scene_graph', GenerateSceneGraph, self._generate_scene_graph_service, buff_size=2**20)
        elif mode == 'topic':
            rospy.loginfo('Starting publishing...')
            self._scene_graph_pub = rospy.Publisher('scene_graph', SceneGraph, queue_size=1)
            rospy.Subscriber('camera/rgb/raw_image', Image, self._generate_scene_graph_topic)
        else:
            rospy.logwarn('Invalid mode, neither service nor topic')

    def generate_scene_graph(self):
        robot = next(r for r in self.extractor.robots if r.name == self.robot)
        detected_objects = self.extractor.detect_objects(robot)

        scene_graph = Digraph(comment='warehouse', format='svg')
        scene_graph.node_attr['shape'] = 'record'

        robot_velocity = get_velocity(robot)
        robot_label = '{%s|%s|velocity: %.2f}' % (robot.name, robot.vision_sensor.name, robot_velocity)

        scene_graph.node('robot', label=robot_label)
        scene_graph.node('warehouse', label='warehouse')
        scene_graph.node('floor', label='{floor|size: 25*25}')
        scene_graph.edge('warehouse', 'floor')

        for o in detected_objects:
            object_direction = get_direction(robot, o)
            object_distance = get_distance_bbox(robot, o)
            object_velocity = get_velocity(o)
            object_type = get_type(o)
            object_orientation = get_orientation(robot, o)
            object_size_x, object_size_y = get_size(o)

            node_label = \
                '{' \
                '%s|type: %s' \
                '|distance: %.2f' \
                '|orientation: %.2f' \
                '|direction: %.2f' \
                '|velocity: %.2f' \
                '|size_x: %.2f' \
                '|size_y: %.2f' \
                '}' % (
                    o.name,
                    object_type,
                    object_distance,
                    object_orientation,
                    object_direction,
                    object_velocity,
                    abs(object_size_x),
                    abs(object_size_y)
                )

            scene_graph.node(o.name, label=node_label)
            if re.match(r'wall*', o.name):
                scene_graph.edge('warehouse', o.name, label='on')
            elif re.match(r'product*', o.name):
                for obj_support in detected_objects:
                    # if get_support_bbox(obj, obj_support):
                    if get_overlap_bbox(o, obj_support):
                        scene_graph.edge(obj_support.name, o.name, label='on')
                        break
                    else:
                        scene_graph.edge('floor', o.name, label='on')
                        break
            else:
                scene_graph.edge('floor', o.name, label='on')

        return scene_graph

    def _generate_scene_graph_service(self, images):
        scene_graph = self.generate_scene_graph()
        scene_graph_msg = _prepare_message(scene_graph)
        return GenerateSceneGraphResponse(scene_graph_msg)

    def _generate_scene_graph_topic(self, image_rgb, image_depth):
        scene_graph = self.generate_scene_graph()
        self._scene_graph_pub.publish(scene_graph)