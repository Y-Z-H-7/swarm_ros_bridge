#!/usr/bin/env python3
"""Publish isolated, non-control synthetic load for a bridge scheduler smoke test."""
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, PointCloud2


def main():
    rospy.init_node("bridge_scheduler_smoke_publisher")
    state_pub = rospy.Publisher("/bridge_test/drone_state_send_3", PoseStamped, queue_size=2)
    battery_pub = rospy.Publisher("/bridge_test/mavros/battery", BatteryState, queue_size=2)
    map_pub = rospy.Publisher("/bridge_test/occupancy_all_3", PointCloud2, queue_size=2)
    rate = rospy.Rate(20)
    count = 0
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        state = PoseStamped()
        state.header.stamp = now
        state.header.frame_id = "bridge_test"
        state.pose.position.x = count
        state_pub.publish(state)
        battery = BatteryState()
        battery.header.stamp = now
        battery.percentage = 0.8
        battery.voltage = 24.0
        battery_pub.publish(battery)
        if count % 4 == 0:
            cloud = PointCloud2()
            cloud.header.stamp = now
            cloud.header.frame_id = "bridge_test"
            cloud.height = 1
            cloud.width = 100000
            cloud.point_step = 1
            cloud.row_step = 100000
            cloud.data = bytes(100000)
            cloud.is_dense = True
            map_pub.publish(cloud)
        count += 1
        rate.sleep()


if __name__ == "__main__":
    main()
