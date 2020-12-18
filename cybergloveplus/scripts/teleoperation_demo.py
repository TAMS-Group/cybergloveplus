"""
move shadow to specified pose which from my cmp2 network
"""
from __future__ import print_function
import moveit_commander
from moveit_msgs.msg import DisplayRobotState
import rospy
from shadow_teleop.srv import *

mgi = moveit_commander.MoveGroupCommander("right_hand")


def clip(x, maxv=None, minv=None):
    if maxv is not None and x > maxv:
        x = maxv
    if minv is not None and x < minv:
        x = minv
    return x


def main():
    rospy.init_node("shadow_human_dataset")
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/hand/robot_state", DisplayRobotState)
        joints = msg.state.joint_state.position
        # shadow hand with biotac
        feature = [joints[23], joints[22], joints[3], joints[2], joints[1],
                   0.3498509706185152, joints[8], joints[7], joints[6], joints[5],
                   0.3498509706185152, joints[12], joints[11], joints[10], 0.3498509706185152,
                   joints[16], joints[15], joints[14], 0.3498509706185152, joints[21],
                   joints[20], joints[19], joints[18], 0.3498509706185152]

        # clip joints into executable ranges
        feature[0] = clip(feature[0], 1.57, 0)
        feature[1] = clip(feature[1], 1.57, 0)
        feature[2] = clip(feature[2], 1.57, 0)
        feature[4] = clip(feature[4], 1.57, 0)
        feature[5] = clip(feature[5], 1.57, 0)
        feature[6] = clip(feature[6], 1.57, 0)
        feature[9] = clip(feature[9], 1.57, 0)
        feature[10] = clip(feature[10], 1.57, 0)
        feature[11] = clip(feature[11], 1.57, 0)
        feature[13] = clip(feature[13], 1.57, 0)
        feature[14] = clip(feature[14], 1.57, 0)
        feature[15] = clip(feature[15], 1.57, 0)
        feature[3] = clip(feature[3], 0.349, -0.349)
        feature[7] = clip(feature[7], 0.349, -0.349)
        feature[12] = clip(feature[12], 0.349, -0.349)
        feature[16] = clip(feature[16], 0.349, -0.349)
        feature[8] = clip(feature[8], 0.785, 0)
        feature[18] = clip(feature[18], 0.524, -0.524)
        feature[19] = clip(feature[19], 0.209, -0.209)
        feature[20] = clip(feature[20], 1.222, 0)
        feature[21] = clip(feature[21], 1.047, -1.047)

        goal = feature
        start = mgi.get_current_joint_values()

        # collision check and manipulate
        csl_client = rospy.ServiceProxy("CheckSelfCollision", checkSelfCollision)
        try:
            shadow_pos = csl_client(start, tuple(goal))
            if shadow_pos.result:
                rospy.loginfo("Move Done!")
            else:
                rospy.logwarn("Failed to move!")
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))


if __name__ == "__main__":
    main()
