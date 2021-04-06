from __future__ import division

import threading
import rospy
import numpy as np
from rl.callbacks import Callback
from std_msgs.msg import Float64
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from turtlebot2i_safety.msg import SafetyZone


class SafetyLogger(Callback):
    sensors = 675

    def __init__(self, filepath, interval=100):
        super(SafetyLogger, self).__init__()

        if not filepath.endswith('.npz'):
            filepath += '.npz'
        self.filepath = filepath
        self.interval = interval
        self.logs = None

        self._collision_lock = threading.Lock()
        self._collisions = 0
        self._collision = False
        rospy.Subscriber('events/bumper', BumperEvent, self._update_collisions)

        self._risk_lock = threading.Lock()
        self._risk_value_sum = 0
        self._risk_value_count = 0
        self._risk_value_max = 0
        self._risk_value_x_speed_sum = 0
        self._risk_value_x_speed_max = 0
        rospy.Subscriber('safety/risk_val', Float64, self._update_risk_values, queue_size=1)

        self._speed_lock = threading.Lock()
        self._speed = 0
        self._speed_sum = 0
        self._speed_count = 0
        rospy.Subscriber('odom', Odometry, self._update_speed, queue_size=1)

        self._safety_zone_lock = threading.Lock()
        self._safety_zone = None
        rospy.Subscriber('safety/safety_zone', SafetyZone, self._update_safety_zone, queue_size=1)

        self._distance_lock = threading.Lock()
        self._in_warning_zone = False
        self._in_critical_zone = False
        self._in_warning_zone_start = None
        self._in_critical_zone_start = None
        self._warning_zone_duration = rospy.Duration.from_sec(0)
        self._critical_zone_duration = rospy.Duration.from_sec(0)
        self._mean_distance_sum = 0
        self._mean_distance_count = 0
        self._mean_min_distance = np.zeros(self.sensors)
        rospy.Subscriber('lidar/scan_transformed', LaserScan, self._lidar_callback)

        self._episode_start = None

    def on_step_begin(self, step, logs=None):
        with self._collision_lock:
            self._collision = False

    def on_episode_begin(self, episode, logs=None):
        with self._collision_lock:
            self._collisions = 0
            self._collision = False

        with self._risk_lock:
            self._risk_value_sum = 0
            self._risk_value_count = 0
            self._risk_value_max = 0
            self._risk_value_x_speed_sum = 0
            self._risk_value_x_speed_max = 0

        with self._speed_lock:
            self._speed = 0
            self._speed_sum = 0
            self._speed_count = 0

        with self._safety_zone_lock:
            self._safety_zone = None

        with self._distance_lock:
            self._in_warning_zone = False
            self._in_critical_zone = False
            self._in_warning_zone_start = None
            self._in_critical_zone_start = None
            self._warning_zone_duration = rospy.Duration.from_sec(0)
            self._critical_zone_duration = rospy.Duration.from_sec(0)
            self._mean_distance_sum = 0
            self._mean_distance_count = 0
            self._mean_min_distance = np.zeros(self.sensors)

        self._episode_start = rospy.Time.now()

        if self.logs is None:
            self.logs = []      # init here and not in the constructor to avoid warning in on_step_end()
        self.logs.append({
            'critical_zone_duration': 0,
            'warning_zone_duration': 0,
            'safe_zone_duration': 0,
            'risk_value_mean': 0,
            'risk_value_max': 0,
            'risk_value_x_speed_mean': 0,
            'risk_value_x_speed_max': 0,
            'collisions': 0,
            'mean_distance': 0,
            'mean_min_distance': 0,
            'min_distance': 0
        })

    def on_step_end(self, step, logs=None):
        if step % self.interval == 0:
            self._save()

    def on_episode_end(self, episode, logs=None):
        self._save()

    def _save(self):
        duration = rospy.Time.now() - self._episode_start
        safe_zone_duration = duration - self._critical_zone_duration - self._warning_zone_duration
        self.logs[-1]['duration'] = duration.to_sec()
        self.logs[-1]['critical_zone_duration'] = self._critical_zone_duration.to_sec()
        self.logs[-1]['warning_zone_duration'] = self._warning_zone_duration.to_sec()
        self.logs[-1]['safe_zone_duration'] = safe_zone_duration.to_sec()
        self.logs[-1]['risk_value_mean'] = self._risk_value_sum / self._risk_value_count
        self.logs[-1]['risk_value_max'] = self._risk_value_max
        self.logs[-1]['risk_value_x_speed_mean'] = self._risk_value_x_speed_sum / self._risk_value_count
        self.logs[-1]['risk_value_x_speed_max'] = self._risk_value_x_speed_max
        self.logs[-1]['collisions'] = self._collisions
        self.logs[-1]['mean_distance'] = self._mean_distance_sum / self._mean_distance_count
        self.logs[-1]['mean_min_distance'] = sum(self._mean_min_distance) / self.sensors
        self.logs[-1]['min_distance'] = min(self._mean_min_distance)
        np.savez(self.filepath, logs=self.logs)

    def _lidar_callback(self, data):
        distances = data.ranges[4:self.sensors+4]
        distance_min = min(distances)

        in_warning_zone_previous = self._in_warning_zone
        in_critical_zone_previous = self._in_critical_zone
        now = rospy.Time.now()

        if distance_min <= self._safety_zone.critical_zone_radius:
            if not self._in_critical_zone:
                self._in_critical_zone_start = now
            self._in_critical_zone = True
            self._in_warning_zone = False
        elif distance_min <= self._safety_zone.warning_zone_radius:
            if not self._in_warning_zone:
                self._in_warning_zone_start = now
            self._in_warning_zone = True
            self._in_critical_zone = False
        else:
            self._in_warning_zone = False
            self._in_critical_zone = False

        if not self._in_warning_zone and in_warning_zone_previous:
            self._warning_zone_duration += now - self._in_warning_zone_start
        elif not self._in_critical_zone and in_critical_zone_previous:
            self._critical_zone_duration += now - self._in_critical_zone_start

        self._mean_distance_sum += sum(distances) / self.sensors
        self._mean_distance_count += 1

        for i in range(self.sensors):
            self._mean_min_distance[i] = min(self._mean_min_distance[i], distances[i])

    def _update_collisions(self, event):
        if event.state == 1:
            with self._collision_lock:
                if not self._collision:
                    self._collisions += 1

    def _update_risk_values(self, risk_value):
        risk_value = risk_value.data
        with self._speed_lock:
            risk_value_x_speed = risk_value * self._speed
        with self._risk_lock:
            self._risk_value_sum += risk_value
            self._risk_value_count += 1
            self._risk_value_max = max(self._risk_value_max, risk_value)
            self._risk_value_x_speed_sum += risk_value_x_speed
            self._risk_value_x_speed_max = max(self._risk_value_x_speed_max, risk_value_x_speed)

    def _update_speed(self, data):
        with self._speed_lock:
            self._speed = np.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2)
            self._speed_sum += self._speed
            self._speed_count += 1

    def _update_safety_zone(self, safety_zone):
        with self._safety_zone_lock:
            self._safety_zone = safety_zone
