#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
将 swarm_ros_bridge 接收到的 bspline/Bspline 轨迹消息，转换为 nav_msgs/Path，
方便在地面站 RViz 中直接显示。

订阅：
  /planning/swarm_traj_recv_1  (bspline/Bspline)
  /planning/swarm_traj_recv_2  (bspline/Bspline)

发布：
  /planning/swarm_traj_path_1  (nav_msgs/Path)
  /planning/swarm_traj_path_2  (nav_msgs/Path)

使用方法（确保已在同一个 ROS 网络下 source 对应工作空间）：
  chmod +x scripts/bspline_to_path_converter.py
  rosrun swarm_ros_bridge bspline_to_path_converter.py

然后在 RViz 里新增两个 Path 显示项，分别订阅：
  /planning/swarm_traj_path_1
  /planning/swarm_traj_path_2

注意：
  1）这里简单地把 B 样条的控制点 pos_pts 直接当成 Path 的离散路径点，
     不是严格意义上的“连续采样曲线”，但足以做可视化。
  2）yaw_pts 若长度与 pos_pts 相同，则用来设置每个 Pose 的航向角；否则统一设为 0。
  3）Path 使用的坐标系 frame_id 默认设为 "map"，如有需要可以改成 "world" 等。
"""

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# 尝试直接导入 bspline 包；如果失败，则显式把 racer 工作空间里的
# Python 消息路径加入 sys.path 后再导入。
try:
    from bspline.msg import Bspline
except ImportError:
    import sys
    sys.path.append("/home/ubuntu20/racer/devel/.private/bspline/lib/python3/dist-packages")
    from bspline.msg import Bspline

import tf.transformations as tft


class BsplineToPathConverter(object):
    def __init__(self):
        # 参数：输出 Path 使用的坐标系
        self.frame_id = rospy.get_param("~frame_id", "map")

        # 发布器：1 号机和 2 号机
        self.path_pub_1 = rospy.Publisher(
            "/planning/swarm_traj_path_1", Path, queue_size=10
        )
        self.path_pub_2 = rospy.Publisher(
            "/planning/swarm_traj_path_2", Path, queue_size=10
        )

        # 订阅器：1 号机和 2 号机的 B 样条轨迹
        rospy.Subscriber(
            "/planning/swarm_traj_recv_1", Bspline, self.bspline_cb_1, queue_size=10
        )
        rospy.Subscriber(
            "/planning/swarm_traj_recv_2", Bspline, self.bspline_cb_2, queue_size=10
        )

        rospy.loginfo("[bspline_to_path] using frame_id = %s", self.frame_id)

    def bspline_to_path(self, msg):
        """将单条 Bspline 消息转换为 nav_msgs/Path。"""
        path = Path()
        # 使用当前时间作为 Header 时间戳
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id

        n_pts = len(msg.pos_pts)
        n_yaw = len(msg.yaw_pts)

        for i, pt in enumerate(msg.pos_pts):
            ps = PoseStamped()
            ps.header = path.header

            # 位置直接使用 B 样条中的控制点位置
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.position.z = pt.z

            # 航向角：如果 yaw_pts 有对应元素则使用，否则置 0
            if i < n_yaw:
                yaw = msg.yaw_pts[i]
            else:
                yaw = 0.0

            qx, qy, qz, qw = tft.quaternion_from_euler(0.0, 0.0, yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw

            path.poses.append(ps)

        return path

    def bspline_cb_1(self, msg):
        path = self.bspline_to_path(msg)
        self.path_pub_1.publish(path)

    def bspline_cb_2(self, msg):
        path = self.bspline_to_path(msg)
        self.path_pub_2.publish(path)


def main():
    rospy.init_node("bspline_to_path_converter")
    converter = BsplineToPathConverter()
    rospy.loginfo("[bspline_to_path] converter node started.")
    rospy.spin()


if __name__ == "__main__":
    main()
