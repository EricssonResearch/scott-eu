"""
Make sure to have the remote API server running in V-REP.

By default, V-REP launches it on port 19997, so you do not need to do anything if you connect to this port.
See https://forum.coppeliarobotics.com/viewtopic.php?t=7514.

Alternatively, you can open a server on other ports adding the following in a child script of a V-REP scene:
simRemoteApi.start(19999)

http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
"""

import rospy
import vrep
import socket
import time
import numpy as np
from shapely.geometry import Polygon, box, LineString


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


class VrepObjectExtractor:
    def __init__(self, host, port, walls, robots, static_objects, dynamic_objects):
        """
        Constructs V-REP object extractor.

        :param host: hostname or IP address of V-REP remote API server
        :param port: port of V-REP remote API server
        :param walls: names of the walls in the V-REP scene
        :param robots: names of the robots in the V-REP scene
        :param static_objects: names of the static objects in the V-REP scene
        :param dynamic_objects: names of the dynamic objects in the V-REP scene
        """
        self.clientID = -1
        self.mode = vrep.simx_opmode_buffer

        self.first_time = self.mode is vrep.simx_opmode_buffer  # True if data streaming mode
        self.start_connection(host, port)
        self.walls = self._get_objects(walls)
        self.robots = self._get_robots(robots)
        self.static_objects = self._get_objects(static_objects)
        self.dynamic_objects = self._get_objects(dynamic_objects)
        self.first_time = False

    def start_connection(self, host, port):
        """Opens remote API connection with V-REP. If there are other open connections, they are closed."""
        # close old connection
        if self.clientID != -1:
            vrep.simxFinish(self.clientID)

        # start new connection
        ip = socket.gethostbyname(host)
        self.clientID = vrep.simxStart(ip, port, True, True, 5000, 5)
        if self.clientID == -1:
            raise Exception('Connection to V-REP remote API server failed')

    def close_connection(self):
        """Close remote API connection with V-REP."""
        vrep.simxFinish(self.clientID)

    def detect_objects(self, robot):
        """Get objects that are in the FOV of the robot's vision sensor."""
        detected_objects = []

        self._refresh()

        pol_fov = Polygon([
            robot.vision_sensor.fov['upper_base'][0],
            robot.vision_sensor.fov['upper_base'][1],
            robot.vision_sensor.fov['lower_base'][0],
            robot.vision_sensor.fov['lower_base'][1]
        ])
        foc = robot.vision_sensor.fov['upper_base'][0]

        # fig = plt.figure(1, figsize=(5,5), dpi=90)
        # ax = fig.add_subplot(111)
        # x,y= pol_fov.exterior.xy
        # ax.plot(x, y)

        objects = self.walls + self.static_objects + self.dynamic_objects + self.robots
        visible_objects = []
        for o in objects:
            pol_obj = box(o.bbox_min[0], o.bbox_min[1], o.bbox_max[0], o.bbox_max[1])
            if pol_fov.intersects(pol_obj) and o is not robot:
                detected_objects.append(o)
            # x,y= pol_obj.exterior.xy
            # ax.plot(x, y)

        for o in detected_objects:
            other_obj = []  # copy.deepcopy(obj_list_detected)
            for temp_obj in detected_objects:
                if o.name != temp_obj.name:
                    other_obj.append(temp_obj)
            x, y = box(o.bbox_min[0], o.bbox_min[1], o.bbox_max[0], o.bbox_max[1]).exterior.xy
            i = 0  # no of line
            visible = True
            while visible and i < len(x):
                test_line = LineString([foc, (x[i], y[i])])
                for another_obj in other_obj:
                    another_obj_box = box(another_obj.bbox_min[0], another_obj.bbox_min[1], another_obj.bbox_max[0],
                                          another_obj.bbox_max[1])
                    if test_line.intersects(another_obj_box):
                        visible = False
                i += 1
            if visible:
                visible_objects.append(o)
            else:  # If each corner is obstructed by other object, check the middle point
                mid_point = (((max(x) + min(x)) / 2), ((max(y) + min(y)) / 2))
                test_line = LineString([foc, mid_point])
                visible = True
                for another_obj in other_obj:
                    another_obj_box = box(another_obj.bbox_min[0], another_obj.bbox_min[1], another_obj.bbox_max[0],
                                          another_obj.bbox_max[1])
                    if test_line.intersects(another_obj_box):
                        visible = False
                if visible:
                    visible_objects.append(o)

            # x,y= pol_obj.exterior.xy
            # ax.plot(x, y)
            # print(obj.name)
            # print(obj.bbox_min[0], obj.bbox_min[1], obj.bbox_max[0], obj.bbox_max[1])

        # plt.show()
        return visible_objects

    def _get_objects(self, object_names):
        objects = []
        for object_name in object_names:
            object_handle = self._get_object_handle(object_name)
            if object_handle is None:
                continue

            object_info = self._get_object_info(object_handle)
            object_ = SceneObject(object_name, object_handle)
            object_.set_info(*object_info)
            objects.append(object_)
        return objects

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

            robot_info = self._get_object_info(robot_handle)
            vision_sensor = self._get_vision_sensor(vision_sensor_name)
            robot = Robot(robot_name, robot_handle, vision_sensor)
            vision_sensor.robot = robot
            robot.set_info(*robot_info)
            robots.append(robot)
        return robots
    
    def _get_vision_sensor(self, vision_sensor_name):
        vision_sensor_handle = self._get_object_handle(vision_sensor_name)
        if vision_sensor_handle is None:
            raise Exception('Robot with no vision sensor')

        # we get the info to to initialize the data streaming, but we cannot set it because we do not have the robot
        # reference yet, so the fov cannot be computed
        _ = self._get_vision_sensor_info(vision_sensor_handle)
        vision_sensor = VisionSensor(vision_sensor_name, vision_sensor_handle)
        return vision_sensor

    def _get_object_info(self, object_handle):
        # start data streaming
        if self.first_time:
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
            time.sleep(0.1)     # streaming takes a while to get ready

        # get pose and orientation
        _, pose = vrep.simxGetObjectPosition(self.clientID, object_handle, -1, self.mode)
        _, orientation = vrep.simxGetObjectOrientation(self.clientID, object_handle, -1, self.mode)

        # get size
        _, param_min_x = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 21, self.mode)
        _, param_min_y = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 22, self.mode)
        _, param_min_z = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 23, self.mode)
        _, param_max_x = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 24, self.mode)
        _, param_max_y = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 25, self.mode)
        _, param_max_z = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 26, self.mode)
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
        _, param_vel_x = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 11, self.mode)
        _, param_vel_y = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 12, self.mode)
        _, param_vel_z = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 13, self.mode)
        _, param_vel_r = vrep.simxGetObjectFloatParameter(self.clientID, object_handle, 14, self.mode)
        velocity = np.array([param_vel_x, param_vel_y, param_vel_z, param_vel_r])

        return pose, orientation, size, velocity, bbox_min, bbox_max

    def _get_vision_sensor_info(self, vision_sensor_handle):
        # start data streaming
        if self.first_time:
            mode = vrep.simx_opmode_streaming
            vrep.simxGetObjectPosition(self.clientID, vision_sensor_handle, -1, mode)
            vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1000, mode)
            vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1001, mode)
            vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1002, mode)
            vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1003, mode)
            vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1004, mode)
            time.sleep(0.1)     # streaming takes a while to get ready

        _, pose = vrep.simxGetObjectPosition(self.clientID, vision_sensor_handle, -1, self.mode)
        _, param_near_clipping = vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1000, self.mode)
        _, param_far_clipping = vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1001, self.mode)
        _, param_resolution_x = vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1002, self.mode)
        _, param_resolution_y = vrep.simxGetObjectIntParameter(self.clientID, vision_sensor_handle, 1003, self.mode)
        _, param_perspective_angle = vrep.simxGetObjectFloatParameter(self.clientID, vision_sensor_handle, 1004, self.mode)

        try:
            ratio = param_resolution_x / param_resolution_y
        except ZeroDivisionError:
            return None

        if ratio > 1:
            param_perspective_angle_x = param_perspective_angle
            param_perspective_angle_y = 2 * np.arctan(np.tan(param_perspective_angle / 2) / ratio)
        else:
            param_perspective_angle_x = 2 * np.arctan(np.tan(param_perspective_angle / 2) * ratio)
            param_perspective_angle_y = param_perspective_angle

        return (pose, param_resolution_x, param_resolution_y, param_perspective_angle, param_perspective_angle_x,
                param_perspective_angle_y, param_near_clipping, param_far_clipping)

    def _get_object_handle(self, object_name):
        return_code, handle = vrep.simxGetObjectHandle(self.clientID, object_name, vrep.simx_opmode_oneshot_wait)
        if return_code != 0:
            rospy.logwarn('%s not found' % object_name)
            return None
        return handle

    def _refresh(self):
        for dynamic_object in self.dynamic_objects:
            dynamic_object_info = self._get_object_info(dynamic_object)
            dynamic_object.set_info(*dynamic_object_info)

        for robot in self.robots:
            robot_info = self._get_object_info(robot.handle)
            robot.set_info(*robot_info)
            vision_sensor_info = self._get_vision_sensor_info(robot.vision_sensor.handle)
            robot.vision_sensor.set_info(*vision_sensor_info)
