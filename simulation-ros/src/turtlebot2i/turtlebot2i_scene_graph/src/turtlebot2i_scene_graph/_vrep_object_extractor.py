from __future__ import division

import rospy
import vrep
import numpy as np
from shapely.geometry import Polygon, box, LineString
from turtlebot2i_scene_graph import VrepClient


class SceneObject(object):
    def __init__(self, name, handle):
        self.name = name
        self.handle = handle

        self.pose = None
        self.orientation = None
        self.size = None
        self.velocity = None
        self.bbox_min = None
        self.bbox_max = None

    def set_info(self, pose, orientation, size, velocity, bbox_min, bbox_max):
        self.pose = pose
        self.orientation = orientation
        self.size = size
        self.velocity = velocity
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max

    def __repr__(self):
        return self.name


class VisionSensor:
    def __init__(self, name, handle):
        self.name = name
        self.handle = handle

        self.robot = None
        self.pose = None
        self.res_x = None
        self.res_y = None
        self.angle = None
        self.angle_x = None
        self.angle_y = None
        self.far_clipping = None
        self.near_clipping = None
        self.fov = None

    def set_info(self, pose, res_x, res_y, angle, angle_x, angle_y, far_clipping, near_clipping):
        self.pose = pose
        self.res_x = res_x
        self.res_y = res_y
        self.angle = angle
        self.angle_x = angle_x
        self.angle_y = angle_y
        self.far_clipping = far_clipping
        self.near_clipping = near_clipping
        self._compute_fov()

    def _compute_fov(self):
        # Use robot orientation as the sensor's
        t = -self.robot.orientation[2]

        # Perspective Projection
        H = self.far_clipping
        D = H * np.tan(self.angle_y / 2)

        h = self.near_clipping
        d = h * np.tan(self.angle_y / 2)

        # TODO: Orthogonal Projection

        # Rotation center
        cx1, cy1, _ = self.pose
        cx2, cy2, _ = self.pose

        # Perspective Projection
        x1 = cx1 + H
        y1 = cy1 - D
        x2 = cx2 + H
        y2 = cy2 + D

        fov_upper_base_r = [[(x1 - cx1) * np.cos(t) + (y1 - cy1) * np.sin(t) + cx1,
                             (y1 - cy1) * np.cos(t) - (x1 - cx1) * np.sin(t) + cy1],
                            [(x2 - cx2) * np.cos(t) + (y2 - cy2) * np.sin(t) + cx2,
                             (y2 - cy2) * np.cos(t) - (x2 - cx2) * np.sin(t) + cy2]]

        x1 = cx1 + h
        y1 = cy1 - d
        x2 = cx2 + h
        y2 = cy2 + d

        fov_lower_base_r = [[(x1 - cx1) * np.cos(t) + (y1 - cy1) * np.sin(t) + cx1,
                             (y1 - cy1) * np.cos(t) - (x1 - cx1) * np.sin(t) + cy1],
                            [(x2 - cx2) * np.cos(t) + (y2 - cy2) * np.sin(t) + cx2,
                             (y2 - cy2) * np.cos(t) - (x2 - cx2) * np.sin(t) + cy2]]

        self.fov = {'upper_base': fov_upper_base_r, 'lower_base': fov_lower_base_r}

    def __repr__(self):
        return self.name


class Robot(SceneObject):
    def __init__(self, name, handle, vision_sensor):
        super(Robot, self).__init__(name, handle)
        self.vision_sensor = vision_sensor


class VrepObjectExtractor(VrepClient):
    def __init__(self):
        super(VrepObjectExtractor, self).__init__()
        self.walls = None
        self.static_objects = None
        self.dynamic_objects = None
        self.robots = None

    def load_objects(self, walls, robots, static_objects, dynamic_objects):
        """Loads specified objects.

        :param walls: names of the walls in the V-REP scene
        :param robots: names of the robots in the V-REP scene
        :param static_objects: names of the static objects in the V-REP scene
        :param dynamic_objects: names of the dynamic objects in the V-REP scene
        """
        self.walls = self._get_objects(walls)
        self.static_objects = self._get_objects(static_objects)
        self.dynamic_objects = self._get_dynamic_objects(dynamic_objects)
        self.robots = self._get_robots(robots)
        objects = self.walls + self.static_objects + self.dynamic_objects + self.robots
        for object_ in objects:
            rospy.loginfo('%s found' % object_.name)

    def detect_objects(self, robot):
        """Get objects visible for the robot."""
        pol_fov = Polygon([
            robot.vision_sensor.fov['upper_base'][0],
            robot.vision_sensor.fov['upper_base'][1],
            robot.vision_sensor.fov['lower_base'][0],
            robot.vision_sensor.fov['lower_base'][1]
        ])
        foc = robot.vision_sensor.fov['upper_base'][0]

        # objects in FOV
        objects = self.walls + self.static_objects + self.dynamic_objects + self.robots
        detected_objects = []
        for o in objects:
            pol_obj = box(o.bbox_min[0], o.bbox_min[1], o.bbox_max[0], o.bbox_max[1])
            if pol_fov.intersects(pol_obj) and o is not robot:
                detected_objects.append(o)

        # visible objects
        visible_objects = []
        for do in detected_objects:
            other_objects = [o for o in detected_objects if o is not do]
            x, y = box(do.bbox_min[0], do.bbox_min[1], do.bbox_max[0], do.bbox_max[1]).exterior.xy

            i = 0  # no of line
            visible = True
            while visible and i < len(x):
                test_line = LineString([foc, (x[i], y[i])])
                for oo in other_objects:
                    other_object_box = box(oo.bbox_min[0], oo.bbox_min[1], oo.bbox_max[0], oo.bbox_max[1])
                    if test_line.intersects(other_object_box):
                        visible = False
                i += 1

            if visible:
                visible_objects.append(do)
            else:  # if each corner is obstructed by other object, check the middle point
                mid_point = (((max(x) + min(x)) / 2), ((max(y) + min(y)) / 2))
                test_line = LineString([foc, mid_point])
                visible = True
                for oo in other_objects:
                    other_object_box = box(oo.bbox_min[0], oo.bbox_min[1], oo.bbox_max[0],
                                           oo.bbox_max[1])
                    if test_line.intersects(other_object_box):
                        visible = False
                if visible:
                    visible_objects.append(do)

        return visible_objects

    def refresh_objects(self):
        """Refreshes robots and dynamic objects."""
        # refresh handles
        scene_restarted = False
        objects = self.dynamic_objects + self.robots
        for object_ in objects:
            object_handle = self._get_object_handle(object_.name)
            if object_handle is None:
                rospy.logwarn('Scene stopped or restarting, skipping refresh...')
                return
            if object_handle != object_.handle:
                object_.handle = object_handle
                scene_restarted = True

        # reload dynamic objects and robots if scene has been restarted
        if scene_restarted:
            rospy.logwarn('Scene restarted, reloading dynamic objects and robots...')
            dynamic_objects = [object_.name for object_ in self.dynamic_objects]
            robots = [robot.name for robot in self.robots]
            self.reset_connection()
            self.dynamic_objects = self._get_dynamic_objects(dynamic_objects)
            self.robots = self._get_robots(robots)

        # refresh dynamic objects
        for dynamic_object in self.dynamic_objects:
            dynamic_object_info = self._get_object_info(dynamic_object.handle, vrep.simx_opmode_buffer)
            if dynamic_object_info is None:
                rospy.logwarn('Streaming not ready for %s, using blocking mode (slower)...' % dynamic_object.name)
                dynamic_object_info = self._get_object_info(dynamic_object.handle, vrep.simx_opmode_blocking)
                self._init_dynamic_object_streaming(dynamic_object.handle)      # blocking request cancels streaming
            dynamic_object.set_info(*dynamic_object_info)

        # refresh robots
        for robot in self.robots:
            robot_info = self._get_object_info(robot.handle, vrep.simx_opmode_buffer)
            if robot_info is None:
                rospy.logwarn('Streaming not ready for %s, using blocking mode (slower)...' % robot.name)
                robot_info = self._get_object_info(robot.handle, vrep.simx_opmode_blocking)
                self._init_dynamic_object_streaming(robot.handle)
            robot.set_info(*robot_info)

            vision_sensor_info = self._get_vision_sensor_info(robot.vision_sensor.handle, vrep.simx_opmode_buffer)
            if vision_sensor_info is None:
                rospy.logwarn('Streaming not ready for %s, using blocking mode (slower)...' % robot.vision_sensor.name)
                vision_sensor_info = self._get_vision_sensor_info(robot.vision_sensor.handle, vrep.simx_opmode_blocking)
                self._init_vision_sensor_streaming(robot.vision_sensor.handle)
            robot.vision_sensor.set_info(*vision_sensor_info)

    def _get_objects(self, object_names):
        objects = []
        for object_name in object_names:
            object_handle = self._get_object_handle(object_name)
            if object_handle is None:
                continue

            object_info = self._get_object_info(object_handle, vrep.simx_opmode_oneshot_wait)
            object_ = SceneObject(object_name, object_handle)
            object_.set_info(*object_info)
            objects.append(object_)
        return objects

    def _get_dynamic_objects(self, object_names):
        dynamic_objects = []
        for object_name in object_names:
            object_handle = self._get_object_handle(object_name)
            if object_handle is None:
                continue
                
            self._init_dynamic_object_streaming(object_handle)
            dynamic_object = SceneObject(object_name, object_handle)
            dynamic_objects.append(dynamic_object)
        return dynamic_objects

    def _get_robots(self, robot_names):
        robots = []
        for robot_name in robot_names:
            try:
                vision_sensor_name = 'camera_rgb#' + robot_name.split('#')[1]
            except IndexError:
                vision_sensor_name = 'camera_rgb'

            robot_handle = self._get_object_handle(robot_name)
            if robot_handle is None:
                continue

            vision_sensor_handle = self._get_object_handle(vision_sensor_name)
            if vision_sensor_handle is None:
                raise Exception('Robot with no vision sensor')
    
            vision_sensor = VisionSensor(vision_sensor_name, vision_sensor_handle)
            self._init_dynamic_object_streaming(robot_handle)
            self._init_vision_sensor_streaming(vision_sensor.handle)
            robot = Robot(robot_name, robot_handle, vision_sensor)
            vision_sensor.robot = robot
            robots.append(robot)
        return robots

    def _get_object_info(self, object_handle, mode):
        # get pose and orientation
        rc, pose = vrep.simxGetObjectPosition(self.clientID, object_handle, -1, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, orientation = vrep.simxGetObjectOrientation(self.clientID, object_handle, -1, mode)
        if rc != vrep.simx_return_ok:
            return None

        # get size
        rc, param_min_x = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 21, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_min_y = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 22, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_min_z = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 23, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_max_x = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 24, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_max_y = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 25, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_max_z = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 26, mode)
        if rc != vrep.simx_return_ok:
            return None
        size_x = param_max_x - param_min_x
        size_y = param_max_y - param_min_y
        size_z = param_max_z - param_min_z
        size = np.array([size_x, size_y, size_z])

        # calculate bounding box
        bbox_min = np.array([pose[0] - size_x / 2.0, pose[1] - size_y / 2.0, pose[2] - size_z / 2.0])
        bbox_max = np.array([pose[0] + size_x / 2.0, pose[1] + size_y / 2.0, pose[2] + size_z / 2.0])

        # rotation of bounding box (http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/matrix2d/)
        x_min, y_min, z_min = bbox_min
        x_max, y_max, z_max = bbox_max
        x_pos, y_pos, _ = pose
        _, _, z_rot = orientation
        r00 = np.cos(z_rot)
        r01 = -np.sin(z_rot)
        r10 = np.sin(z_rot)
        r11 = np.cos(z_rot)
        bbox_x_min_rotated = r00 * x_min + r01 * y_min + x_pos - r00 * x_pos - r01 * y_pos
        bbox_y_min_rotated = r10 * x_min + r11 * y_min + y_pos - r10 * x_pos - r11 * y_pos
        bbox_x_max_rotated = r00 * x_max + r01 * y_max + x_pos - r00 * x_pos - r01 * y_pos
        bbox_y_max_rotated = r10 * x_max + r11 * y_max + y_pos - r10 * x_pos - r11 * y_pos
        bbox_min = np.array([bbox_x_min_rotated, bbox_y_min_rotated, z_min])
        bbox_max = np.array([bbox_x_max_rotated, bbox_y_max_rotated, z_max])

        # get velocity
        rc, param_vel_x = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 11, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_vel_y = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 12, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_vel_z = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 13, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_vel_r = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 14, mode)
        if rc != vrep.simx_return_ok:
            return None
        velocity = np.array([param_vel_x, param_vel_y, param_vel_z, param_vel_r])

        return pose, orientation, size, velocity, bbox_min, bbox_max

    def _get_vision_sensor_info(self, vision_sensor_handle, mode):
        rc, pose = vrep.simxGetObjectPosition(self.clientID, vision_sensor_handle, -1, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_near_clipping = vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1000, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_far_clipping = vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1001, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_resolution_x = vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1002, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_resolution_y = vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1003, mode)
        if rc != vrep.simx_return_ok:
            return None
        rc, param_perspective_angle = vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1004, mode)
        if rc != vrep.simx_return_ok:
            return None

        ratio = param_resolution_x / param_resolution_y
        if ratio > 1:
            param_perspective_angle_x = param_perspective_angle
            param_perspective_angle_y = 2 * np.arctan(np.tan(param_perspective_angle / 2) / ratio)
        else:
            param_perspective_angle_x = 2 * np.arctan(np.tan(param_perspective_angle / 2) * ratio)
            param_perspective_angle_y = param_perspective_angle

        return (pose, param_resolution_x, param_resolution_y, param_perspective_angle, param_perspective_angle_x,
                param_perspective_angle_y, param_near_clipping, param_far_clipping)

    def _get_object_handle(self, object_name):
        rc, handle = vrep.simxGetObjectHandle(self.clientID, object_name, vrep.simx_opmode_oneshot_wait)
        if rc != 0:
            rospy.logwarn('%s not found' % object_name)
            return None
        return handle
    
    def _init_dynamic_object_streaming(self, object_handle):
        mode = vrep.simx_opmode_streaming
        vrep.simxGetObjectPosition(self.clientID, object_handle, -1, mode)
        vrep.simxGetObjectOrientation(self.clientID, object_handle, -1, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 21, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 22, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 23, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 24, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 25, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 26, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 11, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 12, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 13, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 14, mode)

    def _init_vision_sensor_streaming(self, vision_sensor_handle):
        mode = vrep.simx_opmode_streaming
        vrep.simxGetObjectPosition(self.clientID, vision_sensor_handle, -1, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1000, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1001, mode)
        vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1002, mode)
        vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1003, mode)
        vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1004, mode)
