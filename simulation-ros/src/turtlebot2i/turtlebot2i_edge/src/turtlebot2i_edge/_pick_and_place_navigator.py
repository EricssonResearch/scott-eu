import threading
import numpy as np
import rospy
import actionlib
from gym.utils.seeding import np_random
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class PickAndPlaceNavigator:
    def __init__(self, pick_goals, place_goals, seed=None):
        self.pick_goals = pick_goals
        self.place_goals = place_goals
        self.completed_pick_and_place = 0
        self._goal = None
        self._active = False

        # it is recursive because check_goal() calls send_goal(), which can call again check_goal() for a feedback
        self._lock = threading.RLock()

        rospy.loginfo('Waiting for ROS action server to move base...')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        self._rng = None
        self.seed(seed=seed)

    def start(self):
        rospy.loginfo('Pick-and-place navigator started')
        with self._lock:
            self.completed_pick_and_place = 0
            self._active = True
        self._send_goal()

    def stop(self):
        with self._lock:
            self._active = False
            self._goal = None
            self._move_base_client.cancel_all_goals()
        rospy.loginfo('Pick-and-place navigator stopped')

    def refresh(self):
        with self._lock:
            if self._active and self._goal is not None:
                goal = self._get_goal(self._goal)
                self._move_base_client.send_goal(goal, feedback_cb=self._check_goal)

    def seed(self, seed=None):
        self._rng, seed_ = np_random(seed)

    def _send_goal(self):
        with self._lock:
            if self._goal is None or self._goal in self.place_goals:
                goals = self.pick_goals
            else:
                goals = self.place_goals

        idx_goal = self._rng.choice(len(goals))
        with self._lock:
            self._goal = goals[idx_goal]

        goal = self._get_goal(self._goal)
        with self._lock:
            self._move_base_client.send_goal(goal, feedback_cb=self._check_goal)
        rospy.loginfo('New goal: %s' % str(self._goal))

    def _get_goal(self, goal):
        x, y, z = goal
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

    def _check_goal(self, feedback):
        # move base reaches the goal with a certain tolerance (see xy_goal_tolerance in local planner parameters)
        position = feedback.base_position.pose.position
        position = np.array((position.x, position.y, position.z))
        with self._lock:
            if np.linalg.norm(self._goal - position) < 0.5:
                if self._goal in self.place_goals:
                    self.completed_pick_and_place += 1
                if self._active:
                    self._move_base_client.cancel_goal()
                    self._send_goal()

    @property
    def completed_pick_and_place(self):
        with self._lock:
            return self._completed_pick_and_place

    @completed_pick_and_place.setter
    def completed_pick_and_place(self, completed_pick_and_place):
        with self._lock:
            self._completed_pick_and_place = completed_pick_and_place
