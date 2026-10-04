#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from plan_env.msg import ChunkData
from collections import defaultdict

class ChunkDataMonitor(object):
    def __init__(self):
        self.stats = {}
        for name in ["/multi_map_manager/chunk_data_recv_1",
                     "/multi_map_manager/chunk_data_recv_2"]:
            self.stats[name] = {
                "total_msgs": 0,
                "highest_idx": 0,
                "highest_latest_idx": 0,
                "idx_seen": set()
            }
            rospy.Subscriber(name, ChunkData,
                             self.cb, callback_args=name, queue_size=50)

        self.last_print = rospy.Time.now()
        self.print_period = rospy.Duration(5.0)  # 每 5 秒打印一次

    def cb(self, msg, topic_name):
        st = self.stats[topic_name]
        st["total_msgs"] += 1
        st["highest_idx"] = max(st["highest_idx"], msg.idx)
        st["highest_latest_idx"] = max(st["highest_latest_idx"], msg.latest_idx)
        st["idx_seen"].add(msg.idx)

        now = rospy.Time.now()
        if now - self.last_print > self.print_period:
            self.last_print = now
            self.print_stats()

    def print_stats(self):
        rospy.loginfo("========== ChunkData Monitor ==========")
        for name, st in self.stats.items():
            if st["total_msgs"] == 0:
                rospy.loginfo("%s: no data yet", name)
                continue
            missing = max(0, st["highest_latest_idx"] - len(st["idx_seen"]))
            rospy.loginfo(
                "%s: msgs=%d, highest_idx=%d, latest_idx(max)=%d, "
                "distinct_idxs=%d, approx_missing=%d",
                name,
                st["total_msgs"],
                st["highest_idx"],
                st["highest_latest_idx"],
                len(st["idx_seen"]),
                missing
            )

if __name__ == "__main__":
    rospy.init_node("chunk_data_monitor")
    monitor = ChunkDataMonitor()
    rospy.spin()