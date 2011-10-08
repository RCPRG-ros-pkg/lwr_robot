#! /usr/bin/env python

import roslib; roslib.load_manifest('irp6_launch')
import rospy
import actionlib

from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('lwr_arm_controller/joint_trajectory_action', JointTrajectoryAction)
  client.wait_for_server()

  goal = JointTrajectoryGoal()
  goal.trajectory.joint_names = ['lwr_arm_0_joint', 'lwr_arm_1_joint', 'lwr_arm_2_joint', 'lwr_arm_3_joint', 'lwr_arm_4_joint', 'lwr_arm_5_joint', 'lwr_arm_6_joint']
  goal.trajectory.points.append(JointTrajectoryPoint([1.825164385348162, 1.803825933603007, 0.000000000000000, 2.008362700195485, 1.321511416992363, -1.519663789792947, -1.768925223209182], [0, 0, 0, 0, 0, 0, 0], [], rospy.Duration(5.0)))

  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

  client.send_goal(goal)

  client.wait_for_result()

