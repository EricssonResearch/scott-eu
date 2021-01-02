from __future__ import division

import math
import re
from shapely.geometry import box


def get_distance(i, j):
    dx = j.pose[0] - i.pose[0]
    dy = j.pose[1] - i.pose[1]
    if not re.match(r'wall*', j.name):
        # ri = math.sqrt(i.size[0]*i.size[0] + i.size[1]*i.size[1])
        # rj = math.sqrt(j.size[0]*j.size[0] + j.size[1]*j.size[1])
        temp_ij = dx * dx + dy * dy
        dist_ij = math.sqrt(temp_ij)  # - ri - rj
    else:
        raise NotImplementedError

    return dist_ij


def get_distance_bbox(i, j):
    pol_i = box(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
    pol_j = box(j.bbox_min[0], j.bbox_min[1], j.bbox_max[0], j.bbox_max[1])
    min_dist = pol_i.distance(pol_j)
    return min_dist


def get_support_bbox(i, j):
    pol_i = box(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
    pol_j = box(j.bbox_min[0], j.bbox_min[1], j.bbox_max[0], j.bbox_max[1])
    pol_support = pol_i.intersects(pol_j)
    return pol_support


def get_overlap_bbox(i, j):
    pol_i = box(i.bbox_min[0], i.bbox_min[1], i.bbox_max[0], i.bbox_max[1])
    pol_j = box(j.bbox_min[0], j.bbox_min[1], j.bbox_max[0], j.bbox_max[1])
    pol_overl = pol_i.overlaps(pol_j)
    return pol_overl


def get_velocity(j):
    vel_j = math.sqrt(j.velocity[0] * j.velocity[0] + j.velocity[1] * j.velocity[1] + j.velocity[2] * j.velocity[2])
    return vel_j


def get_direction(i, j):
    dx = j.pose[0] - i.pose[0]
    dy = j.pose[1] - i.pose[1]
    dire_tan = math.atan2(dy, dx) - i.orientation[2]
    dire_tan = dire_tan * 180 / math.pi
    if dire_tan > 180:
        dire_tan = dire_tan - 360
    elif dire_tan < -180:
        dire_tan = dire_tan + 360
    else:
        pass
    return dire_tan  # BUG might hide here.


def get_type(i):
    if re.match(r'80cmHighWall*', i.name):
        obj_type = 3  # wall
    elif re.match(r'Bill*', i.name):
        obj_type = 2  # human
    elif re.match(r'turtlebot*', i.name):
        obj_type = 1  # robot # non-human dynamic objects
    else:
        obj_type = 0  # static objects
    return obj_type


def get_orientation(i, j):
    obj_ori = j.orientation[2] * 180 / math.pi - i.orientation[2] * 180 / math.pi
    if obj_ori > 180:
        obj_ori = obj_ori - 360
    elif obj_ori < -180:
        obj_ori = obj_ori + 360
    else:
        pass
    return obj_ori  # BUG here


def get_size(j):
    size_x = j.bbox_max[0] - j.bbox_min[0]
    size_y = j.bbox_max[1] - j.bbox_min[1]
    return size_x, size_y
