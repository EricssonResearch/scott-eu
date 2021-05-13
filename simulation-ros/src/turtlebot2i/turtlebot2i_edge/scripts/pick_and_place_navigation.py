#!/usr/bin/env python

import rospy
from turtlebot2i_edge import PickAndPlaceNavigator


def main():
    rospy.init_node('pick_and_place_navigation')

    rospy.loginfo('Getting pick-and-place goals...')
    pick_goals = rospy.get_param('/pick_and_place/goals/pick')
    place_goals = rospy.get_param('/pick_and_place/goals/place')
    rospy.loginfo('Goals where to pick products: %s' % str(pick_goals))
    rospy.loginfo('Goals where to place products: %s' % str(place_goals))

    rospy.loginfo('Starting to navigate...')
    pick_and_place_navigator = PickAndPlaceNavigator(pick_goals, place_goals)
    pick_and_place_navigator.start()

    rospy.on_shutdown(lambda: pick_and_place_navigator.stop())
    rate = rospy.Rate(2)
    try:
        while not rospy.is_shutdown():
            if not pick_and_place_navigator.active:
                rospy.signal_shutdown('Pick and place navigator stopped for failure')
            rate.sleep()
    except rospy.ROSException:
        pass


if __name__ == '__main__':
    main()
