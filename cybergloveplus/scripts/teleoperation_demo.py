#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import moveit_commander
from moveit_msgs.msg import DisplayRobotState
import rospy

"""
move shadow to specified pose which from cyberglove
"""


def clip(x, maxv=None, minv=None):
    if maxv is not None and x > maxv:
        x = maxv
    if minv is not None and x < minv:
        x = minv
    return x


def main():
    rospy.init_node("shadow_human_teleoperate")
    mgi = moveit_commander.MoveGroupCommander("right_hand")
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/hand/robot_state", DisplayRobotState)
        joints = msg.state.joint_state.position
        # shadow hand with biotac
        joint = [0, 0, joints[3], joints[2], joints[1],
                 0.3498509706185152, joints[8], joints[7], joints[6], joints[5],
                 0.3498509706185152, joints[12], joints[11], joints[10], 0.3498509706185152,
                 joints[16], joints[15], joints[14], 0.3498509706185152, joints[21],
                 joints[20], joints[19], joints[18], 0.3498509706185152]

        # clip joints into executable ranges
        joint[0] = clip(joint[0], 1.57, 0)
        joint[1] = clip(joint[1], 1.57, 0)
        joint[2] = clip(joint[2], 0.349, -0.349)
        joint[3] = clip(joint[3], 1.57, 0)
        joint[4] = clip(joint[4], 1.57, 0)
        joint[5] = clip(joint[5], 1.57, 0)

        joint[6] = clip(joint[6], 0.785, 0)

        joint[7] = clip(joint[7], 0.349, -0.349)
        joint[8] = clip(joint[8], 1.57, 0)
        joint[9] = clip(joint[9], 1.57, 0)
        joint[10] = clip(joint[10], 1.57, 0)

        joint[11] = clip(joint[11], 0.349, -0.349)
        joint[12] = clip(joint[12], 1.57, 0)
        joint[13] = clip(joint[13], 1.57, 0)
        joint[14] = clip(joint[14], 1.57, 0)

        joint[15] = clip(joint[15], 0.349, -0.349)
        joint[16] = clip(joint[16], 1.57, 0)
        joint[17] = clip(joint[17], 1.57, 0)
        joint[18] = clip(joint[18], 1.57, 0)

        joint[19] = clip(joint[19], 1.047, -1.047)
        joint[20] = clip(joint[20], 1.222, 0)
        joint[21] = clip(joint[21], 0.209, -0.209)
        joint[22] = clip(joint[22], 0.524, -0.524)
        joint[23] = clip(joint[23], 1.57, 0)

        mgi.set_joint_value_target(joint)
        mgi.go()


if __name__ == "__main__":
    main()
