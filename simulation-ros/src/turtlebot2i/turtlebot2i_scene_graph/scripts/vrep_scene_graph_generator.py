#!/usr/bin/env python

from __future__ import print_function

import argparse
import time
from turtlebot2i_scene_graph import VrepObjectExtractor, SceneGraphGenerator


WALLS = [
    '80cmHighWall1000cm',
    '80cmHighWall1000cm0',
    '80cmHighWall1500cm',
    '80cmHighWall1500cm0',
    '80cmHighWall1500cm1',
    '80cmHighWall2000cm',
    '80cmHighWall2000cm0',
    '80cmHighWall500cm',
    '80cmHighWall500cm0',
    '80cmHighWall500cm1',
    '80cmHighWall500cm2',
    '80cmHighWall500cm3',
    '80cmHighWall500cm4',
    '80cmHighWall500cm5',
    '80cmHighWall200cm',
    '80cmHighWall200cm0',
    '80cmHighWall200cm1',
    '80cmHighWall200cm2',
    '80cmHighWall200cm3',
    '80cmHighWall750cm',
    '80cmHighWall750cm0',
    '80cmHighWall100cm',
    '80cmHighWall100cm0',
    '80cmHighWall100cm1',
    '80cmHighWall100cm2'
]

ROBOTS = [
    'turtlebot2i',
    'turtlebot2i#0'
]

STATIC_OBJECTS = [
    'stairs',
    'slidingDoor',
    'dockstation_body',
    'ConcreteBox',
    'ConcreteBox#0',
    'ConcreteBox#1',
    'ConcreteBox#2',
    'ConcreteBox#3',
    'ConcreteBox#4',
    'ConcreteBox#5',
    'ConcreteBox#6',
    'ConcreteBox#7',
    'ConcreteBox#8',
    'ConcreteBox#9',
    '80cmHighPillar100cm',
    '80cmHighPillar100cm0',
    'ShelfBody',
    'ShelfBody#0',
    'ShelfBody#1',
    'ShelfBodySimple',
    'ShelfBodySimple#0',
    'ShelfBodySimple#1',
    'ConveyorBeltBody',
    'ConveyorBeltBody#0',
    'ConveyorBeltBody#1',
    'ConveyorBeltBody#2',
    'ConveyorBeltBody#3',
    'ConveyorBeltBody#4',
    'ConveyorBeltBody#5',
    'ConveyorBeltBody#6',
    'ConveyorBeltBody#7',
    'ConveyorBeltBody#8',
    'ConveyorBeltBody#9'
]

DYNAMIC_OBJECTS = [
    'Walking_Bill#0',
    'Walking_Bill#1',
    'Walking_Bill#2',
    'Walking_Bill#3',
    'Walking_Bill#4',
    'Bill',
    'Bill#5',
    'Bill#0'
]


def main():
    print('Parsing command line arguments...')
    parser = argparse.ArgumentParser(
        description='Generate scene graph extracting it from V-REP.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--vrep-host', type=str, default='localhost',
                        help='hostname or IP address of V-REP remote API server')
    parser.add_argument('--vrep-port', type=int, default='19997',
                        help='port of V-REP remote API server')
    parser.add_argument('--robot', type=str, default='turtlebot2i',
                        help='Name of the robot to compute the scene graph for')
    parser.add_argument('--output-path', type=str, default='scene_graph',
                        help='Path where to save the rendered scene graphs')
    args = parser.parse_args()

    print('V-REP remote API server: %s:%d' % (args.vrep_host, args.vrep_port))
    print('Robot: %s' % args.robot)

    print('Connecting to V-REP, can take a while...')
    extractor = VrepObjectExtractor(args.vrep_host, args.vrep_port, WALLS, ROBOTS, STATIC_OBJECTS, DYNAMIC_OBJECTS)
    print('Initialization completed')

    scene_graph_generator = SceneGraphGenerator(args.robot, extractor)

    for _ in range(10):
        time_cost = time.time()
        scene_graph = scene_graph_generator.generate_scene_graph()
        time_cost = time.time() - time_cost
        fps = 1.0 / time_cost
        print('FPS: %f' % fps)
    scene_graph.render('%s/scene_graph' % args.output_path, format='svg', view=True)


if __name__ == '__main__':
    main()
