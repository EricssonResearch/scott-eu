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
        self._goal = None
        self._active = False

        rospy.loginfo('Waiting for ROS action server to move base...')
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()

        # this lock protects self._active, self._goal and self._move_base_client
        # it is recursive because check_goal() calls send_goal(), which can call again check_goal() for a feedback
        self._lock = threading.RLock()

        self._rng = None
        self.seed(seed=seed)

    def start(self):
        rospy.loginfo('Pick-and-place navigator started')
        with self._lock:
            self._active = True
        self._send_goal()

    def stop(self):
        with self._lock:
            self._active = False
            self._goal = None
            self._move_base_client.cancel_all_goals()
        rospy.loginfo('Pick-and-place navigator stopped')

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

        x, y = goals[idx_goal]
        z = 0.063
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

        with self._lock:
            self._move_base_client.send_goal(goal, feedback_cb=self._check_goal)
        rospy.loginfo('New goal: %s' % str(self._goal))

    def _check_goal(self, feedback):
        # move base reaches the goal with a certain tolerance (see xy_goal_tolerance in local planner parameters)
        position = np.array((feedback.base_position.pose.position.x, feedback.base_position.pose.position.y))
        with self._lock:
            if np.linalg.norm(self._goal - position) < 0.5:
                if self._active:
                    self._move_base_client.cancel_goal()
                    self._send_goal()
