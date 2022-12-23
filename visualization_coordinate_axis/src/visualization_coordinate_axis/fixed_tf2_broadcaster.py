#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import csv
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

# tf2ブロードキャスト
class FixedTFBroadcaster:
    # 初期化
    def __init__(self):
        # パブリッシャーの生成
        self.pub_tf = rospy.Publisher(
            "/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        datas = self.load_data()
        self.main(datas)

    def main(self, datas):
        # メインループ
        while not rospy.is_shutdown():
            for i in range(len(datas)): # areaの数
                for j in range(len(datas[i])): # 物体の数

                    rospy.sleep(0.1)

                    t = geometry_msgs.msg.TransformStamped()
                    t.header.frame_id = "map"
                    t.header.stamp = rospy.Time.now()
                    t.child_frame_id = "{}_{}".format(datas[i][j][1], str(i+1))


                    t.transform.translation.x = float(datas[i][j][2])
                    t.transform.translation.y = float(datas[i][j][3])
                    t.transform.translation.z = float(datas[i][j][4])
                    t.transform.rotation.x = 0.0
                    t.transform.rotation.y = 0.0
                    t.transform.rotation.z = 0.0
                    t.transform.rotation.w = 1.0

                    # メッセージの生成
                    tfm = tf2_msgs.msg.TFMessage([t])

                    # メッセージのパブリッシュ
                    self.pub_tf.publish(tfm)

    def load_data(self):
        files = os.listdir("/root/HSR/catkin_ws/src/visualization_coordinate_axis/visualization_coordinate_axis/data/")
        datas = []
        data = []
        for i in range(len(files)):
            with open("/root/HSR/catkin_ws/src/visualization_coordinate_axis/visualization_coordinate_axis/data/object_position_{}.csv".format(str(i+1))) as f:
                reader = csv.reader(f)
                for row in reader:
                    data.append(row)
            datas.append(data)
            data = []
        # print(datas)
        return datas


# メイン
if __name__ == '__main__':
    # ノードの初期化
    rospy.init_node('fixed_tf2_broadcaster')

    # tf2ブロードキャスターの生成
    tfb = FixedTFBroadcaster()

    # ノード終了まで待つ
    rospy.spin()
