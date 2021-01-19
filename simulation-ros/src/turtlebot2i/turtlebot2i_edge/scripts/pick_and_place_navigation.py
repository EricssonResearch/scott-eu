#!/usr/bin/env python

import rospy
from turtlebot2i_edge import PickAndPlaceNavigator


def main():
    rospy.init_node('pick_and_place_navigation')

    rospy.loginfo('Getting pick-and-place goals...')
    pick_goals = rospy.get_param('~goals/pick')
    place_goals = rospy.get_param('~goals/place')
    rospy.loginfo('Goals where to pick products: %s' % str(pick_goals))
    rospy.loginfo('Goals where to place products: %s' % str(place_goals))

    rospy.loginfo('Starting to navigate...')
    pick_and_place_navigator = PickAndPlaceNavigator(pick_goals, place_goals)
    pick_and_place_navigator.start()

    rospy.on_shutdown(lambda: pick_and_place_navigator.stop())
    rospy.spin()


if __name__ == '__main__':
    main()
