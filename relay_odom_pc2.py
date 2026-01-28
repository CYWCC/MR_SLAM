#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import argparse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from dislam_msgs.msg import SubMap

import sensor_msgs.point_cloud2 as pc2


# ------------------ 工具函数 ------------------

def quat_to_rot(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])


def odom_to_T(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = quat_to_rot([q.x, q.y, q.z, q.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def cloud_to_np(msg):
    pts = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        pts.append([p[0], p[1], p[2], p[3]])
    return np.array(pts, dtype=np.float32)


def np_to_cloud(pts, frame, stamp):
    header = rospy.Header()
    header.stamp = stamp
    header.frame_id = frame
    fields = [
        pc2.PointField("x", 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField("y", 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField("z", 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField("intensity", 12, pc2.PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(header, fields, pts)


def transform_points(pts, T):
    if len(pts) == 0:
        return pts
    xyz = pts[:, :3]
    homo = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
    out = (T @ homo.T).T
    res = pts.copy()
    res[:, :3] = out[:, :3]
    return res


def voxel_downsample(pts, leaf):
    if len(pts) == 0:
        return pts
    key = np.floor(pts[:, :3] / leaf).astype(np.int32)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[idx]


# ------------------ 主节点 ------------------

class SubmapNode:
    def __init__(self, args):
        self.dis_th = args.distance_thresh
        self.voxel = args.voxel_leaf

        self.world_frame = args.world_frame
        self.body_frame = args.body_frame

        self.is_first_odom = True
        self.x0 = self.y0 = self.z0 = 0.0

        self.T_w_b = np.eye(4)
        self.accum_world = np.empty((0, 4), dtype=np.float32)

        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(args.cloud_body_topic, PointCloud2, self.cloud_cb, queue_size=5)

        self.pub_world = rospy.Publisher(args.cloud_world_topic, PointCloud2, queue_size=5)
        self.pub_submap = rospy.Publisher(args.submap_topic, SubMap, queue_size=5)
        self.pub_odom = rospy.Publisher(args.odom_relay_topic, Odometry, queue_size=50)

    def odom_cb(self, msg):
        p = msg.pose.pose.position

        if self.is_first_odom:
            self.x0, self.y0, self.z0 = p.x, p.y, p.z
            self.is_first_odom = False

        self.T_w_b = odom_to_T(msg)

        dist = math.sqrt(
            (p.x - self.x0)**2 +
            (p.y - self.y0)**2 +
            (p.z - self.z0)**2
        )

        if dist > self.dis_th:
            self.x0, self.y0, self.z0 = p.x, p.y, p.z
            self.publish_submap(msg)

        self.pub_odom.publish(msg)

    def cloud_cb(self, msg):
        body_pts = cloud_to_np(msg)
        world_pts = transform_points(body_pts, self.T_w_b)

        self.accum_world = np.vstack([self.accum_world, world_pts])

        cloud_msg = np_to_cloud(world_pts, self.world_frame, msg.header.stamp)
        self.pub_world.publish(cloud_msg)

    def publish_submap(self, odom_msg):
        T_b_w = np.linalg.inv(self.T_w_b)
        body_pts = transform_points(self.accum_world, T_b_w)
        body_pts = voxel_downsample(body_pts, self.voxel)

        pc_msg = np_to_cloud(body_pts, self.body_frame, odom_msg.header.stamp)

        submap = SubMap()
        submap.keyframePC = pc_msg
        submap.pose = odom_msg.pose.pose

        self.pub_submap.publish(submap)

        self.accum_world = np.empty((0, 4), dtype=np.float32)


# ------------------ main ------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom_topic", required=True)
    parser.add_argument("--cloud_body_topic", required=True)
    parser.add_argument("--cloud_world_topic", required=True)
    parser.add_argument("--odom_relay_topic", required=True)
    parser.add_argument("--submap_topic", required=True)

    parser.add_argument("--world_frame", default="map")
    parser.add_argument("--body_frame", default="base_link")
    parser.add_argument("--distance_thresh", type=float, default=5.0)
    parser.add_argument("--voxel_leaf", type=float, default=0.2)

    args, _ = parser.parse_known_args()

    rospy.init_node("simple_submap_relay")
    SubmapNode(args)
    rospy.spin()
