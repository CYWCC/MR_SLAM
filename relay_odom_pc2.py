#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import argparse

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from dislam_msgs.msg import SubMap

import sensor_msgs.point_cloud2 as pc2

# TF（可选）
try:
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
except Exception:
    tf2_ros = None
    TransformStamped = None


# ------------------ 工具函数 ------------------

def quat_to_rot(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ], dtype=np.float32)


def odom_to_T(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = quat_to_rot([q.x, q.y, q.z, q.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def cloud_to_np(msg):
    # 兼容没有 intensity 的点云
    field_names = [f.name for f in msg.fields]
    has_i = "intensity" in field_names

    pts = []
    if has_i:
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            pts.append([p[0], p[1], p[2], p[3]])
    else:
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append([p[0], p[1], p[2], 0.0])

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
    return pc2.create_cloud(header, fields, pts.astype(np.float32))


def transform_points(pts, T):
    if pts is None or pts.size == 0:
        return pts
    xyz = pts[:, :3]
    homo = np.hstack([xyz, np.ones((xyz.shape[0], 1), dtype=np.float32)])
    out = (T @ homo.T).T
    res = pts.copy()
    res[:, :3] = out[:, :3]
    return res


def voxel_downsample(pts, leaf):
    if pts is None or pts.size == 0 or leaf <= 0:
        return pts
    key = np.floor(pts[:, :3] / leaf).astype(np.int32)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[idx]


# ------------------ 主节点 ------------------

class RelayLIOPubLike:
    """
    目标：尽可能对齐 C++ LIO_Pub 行为
    - 输入：odom + cloud_registered_body
    - 输出（可选/默认）：
        - odom_relay_topic (Odometry)
        - cloud_world_topic (PointCloud2, world_frame) [可选]
        - submap_topic (SubMap, keyframePC frame=body_frame)
        - merged_cloud_topic (PointCloud2, frame=body_frame)
        - new_keyframe_topic (Bool pulse)
        - TF(world_frame -> body_frame) [可选]
    """

    def __init__(self, args):
        self.dis_th = args.distance_thresh
        self.leaf = args.voxel_leaf

        self.world_frame = args.world_frame
        self.body_frame = args.body_frame

        self.downsample_each_frame = args.downsample_each_frame
        self.publish_world_cloud = args.publish_world_cloud
        self.publish_merged_cloud = args.publish_merged_cloud
        self.publish_new_keyframe = args.publish_new_keyframe
        self.publish_tf = args.publish_tf

        self.have_odom = False
        self.is_first_odom = True
        self.x0 = self.y0 = self.z0 = 0.0
        self.T_w_b = np.eye(4, dtype=np.float32)

        # 为了避免 vstack O(N^2)，用 list 累积（行为等价）
        self.accum_world_list = []

        # pubs
        self.pub_odom = rospy.Publisher(args.odom_relay_topic, Odometry, queue_size=50)

        self.pub_world = rospy.Publisher(args.cloud_world_topic, PointCloud2, queue_size=5) \
            if self.publish_world_cloud else None

        self.pub_submap = rospy.Publisher(args.submap_topic, SubMap, queue_size=5)

        self.pub_merged = rospy.Publisher(args.merged_cloud_topic, PointCloud2, queue_size=5) \
            if self.publish_merged_cloud else None

        self.pub_key = rospy.Publisher(args.new_keyframe_topic, Bool, queue_size=5) \
            if self.publish_new_keyframe else None

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if (self.publish_tf and tf2_ros) else None

        # subs
        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(args.cloud_body_topic, PointCloud2, self.cloud_cb, queue_size=5)

        rospy.loginfo("RelayLIOPubLike started.")
        rospy.loginfo(f"world_frame={self.world_frame}, body_frame={self.body_frame}, dis_th={self.dis_th}, leaf={self.leaf}")
        rospy.loginfo(f"downsample_each_frame={self.downsample_each_frame}, publish_world_cloud={self.publish_world_cloud}, "
                      f"publish_merged_cloud={self.publish_merged_cloud}, publish_new_keyframe={self.publish_new_keyframe}, publish_tf={self.publish_tf}")

    def odom_cb(self, msg):
        p = msg.pose.pose.position

        if self.is_first_odom:
            self.x0, self.y0, self.z0 = p.x, p.y, p.z
            self.is_first_odom = False

        self.T_w_b = odom_to_T(msg)
        self.have_odom = True

        # C++ 有 TF： parent=NameSpace/odom, child=NameSpace+SensorName
        self._pub_tf_if_needed(msg)

        dist = math.sqrt(
            (p.x - self.x0) ** 2 +
            (p.y - self.y0) ** 2 +
            (p.z - self.z0) ** 2
        )

        if dist > self.dis_th:
            self.x0, self.y0, self.z0 = p.x, p.y, p.z
            self.publish_keyframe_outputs(msg)

        # 你的 relay：转发 odom
        self.pub_odom.publish(msg)

    def cloud_cb(self, msg):
        if not self.have_odom:
            return

        body_pts = cloud_to_np(msg)
        if body_pts.size == 0:
            return

        # 先从 body -> world（对齐“cloud_registered是世界系”）
        world_pts = transform_points(body_pts, self.T_w_b)

        # 对齐 C++：每帧先下采样再累积（可开关）
        if self.downsample_each_frame and self.leaf > 0:
            world_pts = voxel_downsample(world_pts, self.leaf)

        self.accum_world_list.append(world_pts)

        # 你原来的功能：发布世界系 cloud_registered（可开关）
        if self.pub_world is not None:
            cloud_msg = np_to_cloud(world_pts, self.world_frame, msg.header.stamp)
            self.pub_world.publish(cloud_msg)

    def publish_keyframe_outputs(self, odom_msg):
        # new_keyframe 脉冲（对齐 C++）
        if self.pub_key is not None:
            b = Bool()
            b.data = True
            self.pub_key.publish(b)

        if not self.accum_world_list:
            return

        accum_world = np.vstack(self.accum_world_list)
        self.accum_world_list = []

        # C++：transform = odom2isometry(msg); transform.inverse() 把世界云转回 body
        T_b_w = np.linalg.inv(self.T_w_b)
        body_pts = transform_points(accum_world, T_b_w)

        # C++：关键帧时也再 voxel 一次（这里保留）
        if self.leaf > 0:
            body_pts = voxel_downsample(body_pts, self.leaf)

        # 生成 output：frame_id=NameSpace+SensorName（对应这里的 body_frame）
        pc_msg = np_to_cloud(body_pts, self.body_frame, odom_msg.header.stamp)

        # submap（对齐 C++）
        submap = SubMap()
        submap.keyframePC = pc_msg
        submap.pose = odom_msg.pose.pose
        self.pub_submap.publish(submap)

        # merged_cloud_registered（对齐 C++，可选）
        if self.pub_merged is not None:
            self.pub_merged.publish(pc_msg)

    def _pub_tf_if_needed(self, odom_msg):
        if self.tf_broadcaster is None:
            return

        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.body_frame

        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)


# ------------------ main ------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # inputs
    parser.add_argument("--odom_topic", required=True)
    parser.add_argument("--cloud_body_topic", required=True)

    # outputs（基础）
    parser.add_argument("--cloud_world_topic", required=True)   # world cloud_registered（可关）
    parser.add_argument("--odom_relay_topic", required=True)
    parser.add_argument("--submap_topic", required=True)

    # outputs（对齐 C++ 新增）
    parser.add_argument("--merged_cloud_topic", default="merged_cloud_registered")
    parser.add_argument("--new_keyframe_topic", default="new_keyframe")

    # frames / params
    parser.add_argument("--world_frame", default="map")
    parser.add_argument("--body_frame", default="base_link")
    parser.add_argument("--distance_thresh", type=float, default=5.0)
    parser.add_argument("--voxel_leaf", type=float, default=0.2)

    # switches（对齐 C++ 行为 + 兼容你原行为）
    parser.add_argument("--downsample_each_frame", type=int, default=1,
                        help="1=每帧先下采样再累积（对齐C++）；0=不每帧下采样")
    parser.add_argument("--publish_world_cloud", type=int, default=1,
                        help="1=发布每帧世界系 cloud_world_topic；0=不发布")
    parser.add_argument("--publish_merged_cloud", type=int, default=1,
                        help="1=发布 merged_cloud_topic（关键帧点云，body_frame）；0=不发布")
    parser.add_argument("--publish_new_keyframe", type=int, default=1,
                        help="1=发布 new_keyframe_topic Bool脉冲；0=不发布")
    parser.add_argument("--publish_tf", type=int, default=1,
                        help="1=广播 TF(world_frame->body_frame)；0=不广播")

    args, _ = parser.parse_known_args()

    # int -> bool
    args.downsample_each_frame = bool(args.downsample_each_frame)
    args.publish_world_cloud = bool(args.publish_world_cloud)
    args.publish_merged_cloud = bool(args.publish_merged_cloud)
    args.publish_new_keyframe = bool(args.publish_new_keyframe)
    args.publish_tf = bool(args.publish_tf)

    rospy.init_node("simple_submap_relay", anonymous=True)
    RelayLIOPubLike(args)
    rospy.spin()
