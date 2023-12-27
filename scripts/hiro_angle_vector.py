#!/usr/bin/env python

import os
import copy
import sys
import glob
import pprint
import random
import datetime
import argparse

import numpy as np
import mujoco
import rospy
from rospkg import RosPack
from std_msgs.msg import Float32MultiArray, String


rospack = RosPack()


class RobotArmController:
    def __init__(self):
        self.link_num = 20
        rospy.Subscriber("mujoco_qpos", Float32MultiArray, callback=self.qpos_callback, queue_size=1)
        self.ctrl_ref_pub = rospy.Publisher("mujoco_ctrl_ref", Float32MultiArray, queue_size=1)
        self.ctrl_ref_msg = Float32MultiArray()
        self.qpos = np.zeros(self.link_num, dtype=np.float32)

    def move_to_target(self, target_qpos):
        while not rospy.is_shutdown():
            cur_qpos = self.qpos
            self.ctrl_ref_msg.data = np.concatenate([target_qpos, np.array([1.0])])
            # ctrl_ref_msg.data = np.concatenate([np.array([-0.2685, -0.31315, -2.30818, -0.53463, 1.05814, -0.19955, 0.94664, -0.94664, -0.94664, 0.94664, 0.2685, -0.31315, -2.30818, 0.53463, 1.05814, 0.19955, 0.94664, -0.94664, -0.94664, 0.94664]), np.array([1.0])])
            self.ctrl_ref_pub.publish(self.ctrl_ref_msg)
            rospy.sleep(rospy.Duration(0.1))

            for val1, val2 in zip(target_qpos, cur_qpos):
                if abs(val1 - val2) > 0.1:
                    break
            else:
                break
        rospy.sleep(rospy.Duration(0.1))

    def qpos_callback(self, msg):
        self.qpos = msg.data


def main():
    rospy.init_node("hiro_angle_vector_node")
    controller = RobotArmController()

    # # reset pose
    # controller.move_to_target(np.array([-0.2685, -0.31315, -2.30818, -0.53463, 1.05814, -0.19955, 0.94664, -0.94664, -0.94664, 0.94664, 0.2685, -0.31315, -2.30818, 0.53463, 1.05814, 0.19955, 0.94664, -0.94664, -0.94664, 0.94664]))
    # # stretch pose
    # controller.move_to_target(np.array([0.56586, -0.9341, -1.53767, -0.2974, 0.73449, -0.7848, 0.94664, -0.94664, -0.94664, 0.94664, -0.24142, -0.35211, -1.42493, 0.27801, 0.15532, 0.26306, 0.94664, -0.94664, -0.94664, 0.94664]))
    # controller.move_to_target(np.array([0.56586, -0.9341, -1.53767, -0.2974, 0.73449, -0.7848, 0.94664, -0.94664, -0.94664, 0.94664, -0.43357, -0.53925, -1.21353, -0.22949, 0.28646, 0.34723, 0.94664, -0.94664, -0.94664, 0.94664]))
    # # 30 in y axis
    # controller.move_to_target(np.array([0.56586, -0.9341, -1.53767, -0.2974, 0.73449, -0.7848, 0.94664, -0.94664, -0.94664, 0.94664, -0.37355, -0.46242, -1.29133, -0.23519, 0.2738, 0.28723, 0.94664, -0.94664, -0.94664, 0.94664]))
    # controller.move_to_target(np.array([0.56586, -0.9341, -1.53767, -0.2974, 0.73449, -0.7848, 0.94664, -0.94664, -0.94664, 0.94664, -0.43357, -0.53925, -1.21353, -0.22949, 0.28646, 0.34723, -0.39181, 0.39181, 0.39181, -0.39181]))
    controller.move_to_target(np.array([0.56586, -0.9341, -1.53767, -0.2974, 0.73449, -0.7848, 0.94664, -0.94664, -0.94664, 0.94664, -0.10046, -0.31257, -1.77624, -0.28446, 0.53237, -0.07567, -0.39181, 0.39181]))

if __name__ == '__main__':
    main()
