#!/usr/bin/env python

from __future__ import print_function

import argparse
import time
import yaml
from turtlebot2i_scene_graph import VrepObjectExtractor, SceneGraphGenerator


def load_objects(filepath):
    with open(filepath) as f:
        content = yaml.load(f, Loader=yaml.FullLoader)
        content = content['vrep']
    return content['walls'], content['robots'], content['static_objects'], content['dynamic_objects']


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
    extractor = VrepObjectExtractor()
    extractor.start_connection(args.vrep_host, args.vrep_port)
    extractor.load_objects(*load_objects('config/vrep.yaml'))
    print('Initialization completed')

    scene_graph_generator = SceneGraphGenerator(args.robot, extractor)

    n_runs = 10
    print('Generating scene graph %d times' % n_runs)
    for i in range(n_runs):
        time_cost = time.time()
        scene_graph = scene_graph_generator.generate_scene_graph()
        time_cost = time.time() - time_cost
        fps = 1.0 / time_cost
        print('FPS: %f' % fps)
        if i == n_runs - 1:
            scene_graph.render('%s/scene_graph' % args.output_path, format='svg', view=True)


if __name__ == '__main__':
    main()
