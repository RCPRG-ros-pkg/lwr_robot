#! /usr/bin/env python

import roslib; roslib.load_manifest('irp6_launch')
import rospy
import actionlib

from trajectory_msgs.msg import *
from control_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('lwr_arm_controller/joint_trajectory_action', FollowJointTrajectoryAction)
  client.wait_for_server()

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['lwr_arm_0_joint', 'lwr_arm_1_joint', 'lwr_arm_2_joint', 'lwr_arm_3_joint', 'lwr_arm_4_joint', 'lwr_arm_5_joint', 'lwr_arm_6_joint']
  goal.trajectory.points.append(JointTrajectoryPoint([1.825164385348162, 1.803825933603007, 0.000000000000000, 2.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182], [0, 0, 0, 0, 0, 0, 0], [], rospy.Duration(1.0)))
  goal.trajectory.points.append(JointTrajectoryPoint([1.925164385348162, 1.803825933603007, 0.000000000000000, 2.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182], [0, 0, 0, 0, 0, 0, 0], [], rospy.Duration(1.1)))
  goal.trajectory.points.append(JointTrajectoryPoint([1.825164385348162, 1.203825933603007, 0.000000000000000, 2.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182], [0, 0, 0, 0, 0, 0, 0], [], rospy.Duration(1.2)))
  goal.trajectory.points.append(JointTrajectoryPoint([1.825164385348162, 1.803825933603007, 0.000000000000000, 2.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182], [0, 0, 0, 0, 0, 0, 0], [], rospy.Duration(1.3)))
  goal.trajectory.points.append(JointTrajectoryPoint([1.825164385348162, 1.803825933603007, 0.400000000000000, 1.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182], [0, 0, 0, 0, 0, 0, 0], [], rospy.Duration(1.4)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

  client.send_goal(goal)

  client.wait_for_result()

