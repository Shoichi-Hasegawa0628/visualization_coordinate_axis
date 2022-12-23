#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import geometry_msgs.msg
import math


# メイン
if __name__ == '__main__':
    # ノードの生成
    rospy.init_node('dynamic_tf2_broadcaster')

    # ブロードキャスターの生成
    br = tf2_ros.TransformBroadcaster()

    # Transformの生成
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "map"
    t.child_frame_id = "human"

    # メインループ
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        x = rospy.Time.now().to_sec() * math.pi

        # Transformの生成
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = 10 * math.sin(x)
        t.transform.translation.y = 10 * math.cos(x)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Transformの送信
        br.sendTransform(t)

        # スリープ
        rate.sleep()
