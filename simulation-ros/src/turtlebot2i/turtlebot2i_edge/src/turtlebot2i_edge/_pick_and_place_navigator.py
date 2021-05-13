import threading
import numpy as np
import rospy
import actionlib
from gym.utils.seeding import np_random
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class PickAndPlaceNavigator:
    def __init__(self, pick_goals, place_goals, seed=None):
        # it is recursive because check_goal() calls send_goal(), which can call again check_goal()
        self._main_lock = threading.RLock()

        self.pick_goals = pick_goals
        self.place_goals = place_goals
        self._goal = None
        self.active = False

        self._completed_pick_and_place_lock = threading.Lock()
        self.completed_pick_and_place = 0

        rospy.loginfo('Waiting for ROS action server to move base...')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        self._rng = None
        self._rng_lock = threading.Lock()
        self.seed(seed=seed)

    def start(self):
        with self._main_lock:
            if not self.active:
                rospy.loginfo('Pick-and-place navigator started')
                self.active = True
                self._send_goal()
            else:
                self.refresh()

    def stop(self):
        with self._main_lock:
            if self.active:
                self.active = False
                self._goal = None
                self._move_base_client.cancel_all_goals()
                rospy.loginfo('Pick-and-place navigator stopped')

    def refresh(self):
        with self._main_lock:
            if self.active and self._goal is not None:
                goal = self._get_goal(self._goal)
                self._move_base_client.send_goal(goal, done_cb=self._check_goal)
                rospy.loginfo('Pick-and-place navigator refreshed')

    def seed(self, seed=None):
        rng, _ = np_random(seed)
        with self._rng_lock:
            self._rng = rng

    def _send_goal(self):
        if self._goal is None or self._goal in self.place_goals:
            goals = self.pick_goals
        else:
            goals = self.place_goals

        with self._rng_lock:
            idx_goal = self._rng.choice(len(goals))

        self._goal = goals[idx_goal]
        goal = self._get_goal(self._goal)
        self._move_base_client.send_goal(goal, done_cb=self._check_goal)
        rospy.loginfo('New goal: %s' % str(self._goal))

    def _get_goal(self, goal):
        x, y, z = goal
        with self._rng_lock:
            yaw = self._rng.uniform(-np.pi, np.pi)
        pose = Pose(
            position=Point(x, y, z),
            orientation=Quaternion(*list(quaternion_from_euler(0, 0, yaw)))
        )
        pose_stamped = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='map'),
            pose=pose
        )
        goal = MoveBaseGoal(target_pose=pose_stamped)
        return goal

    def _check_goal(self, status, _):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo('Goal reached')
            with self._main_lock:
                if self._goal in self.place_goals:
                    self.completed_pick_and_place += 1
                    rospy.loginfo('Pick-and-place completed')
                if self.active:
                    self._send_goal()
        else:   # failed
            rospy.logwarn('Goal failed with status %d...' % status)
            self.stop()

    @property
    def active(self):
        with self._main_lock:
            return self._active

    @active.setter
    def active(self, active):
        with self._main_lock:
            self._active = active

    @property
    def completed_pick_and_place(self):
        with self._completed_pick_and_place_lock:
            return self._completed_pick_and_place

    @completed_pick_and_place.setter
    def completed_pick_and_place(self, completed_pick_and_place):
        with self._completed_pick_and_place_lock:
            self._completed_pick_and_place = completed_pick_and_place
