#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class SAPObject:
    def __init__(self):
            # Get instance from moveit_commander
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            # Get group_commander from MoveGroupCommander
            self.group_name1 = "arm"
            self.group_name2 = "gripper"
            #group_name = "manipulator"
            self.move_group1 = moveit_commander.MoveGroupCommander(self.group_name1)
            self.move_group2 = moveit_commander.MoveGroupCommander(self.group_name2)
            self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


    def Opengripper(self):
        joint_goal2 = self.move_group2.get_current_joint_values()
        joint_goal2[0] = 0.013
        try:
            self.move_group2.go(joint_goal2, wait=True)
            self.move_group2.stop()
            return True
        except:
            print("Fail to open gripper")
            return False

    def Closegripper(self):
        joint_goal2 = self.move_group2.get_current_joint_values()
        joint_goal2[0] = -0.010
        try:
            self.move_group2.go(joint_goal2, wait=True)
            self.move_group2.stop()
            return True
        except:
            print("Fail to close gripper")
            return False

    def move_homepos(self):
        # Move using joint space
        # # Move to home position
        joint_goal1 = self.move_group1.get_current_joint_values()

        joint_goal1[0] = 0
        joint_goal1[1] = -1.000
        joint_goal1[2] = 0.300
        joint_goal1[3] = 0.700
        try:
            self.move_group1.go(joint_goal1, wait=True)
            self.move_group1.stop()
            return True
        except:
            print("Fail to move arm")
            return False
    
    def move_targetpos(self, position):
        try:
            self.move_group1.set_position_target(position)
            self.move_group1.go(wait=True)
            self.move_group1.stop()
            self.move_group1.clear_pose_targets()
            return True
        except:
            print("Fail to move targetpos")
            return False

    def move_wheel(self, vel):
        twist=Twist()
        twist.linear.x=vel
        self.pub.publish(twist)
        rospy.sleep(2)
        twist.linear.x=0
        self.pub.publish(twist)

        



if __name__ == "__main__":
    # Initialize moveit_commander and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Project', anonymous=False)

    base=SAPObject()

    rospy.sleep(3)


    rospy.sleep(3)
    # move to home position
    base.move_homepos()
    base.Closegripper()

    rospy.sleep(4)

    #move to taget position
    x = 0.27
    y = 0.00
    z = 0.26
    position=[x , y , z]
    base.move_targetpos(position)

    rospy.sleep(7)



    # move to home position and ready to grip
    base.move_homepos()
    rospy.sleep(3)
    base.Opengripper()


    quit()
