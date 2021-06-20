import rospy
import time
import sys
import copy
import CM
import patrol
import rotate
import move
import GL
import searching
from geometry_msgs.msg import Twist
import arm_move
import grap_staff
import roslaunch

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
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

    def rotate_wheel(self, angular_vel):
	twist=Twist()
	twist.angular.z=angular_vel
	self.pub.publish(twist)
	rospy.sleep(4)
	twist.angular.z=0
	self.pub.publish(twist)



#patrol
while(1):
	a=input("Select box num you want!(1:left 2:middle 3:right) : ")
	if(a==1):
		patrol.patrol([[(3.15,0.45,0.0),(0.0,0.0,0.0,1.0)]])
		break
	elif(a==2):
		patrol.patrol([[(3.15,-0.08,0.0),(0.0,0.0,0.0,1.0)]])
		break
	elif(a==3):
		patrol.patrol([[(3.15,-0.45,0.0),(0.0,0.0,0.0,1.0)]])
		break



#ar mark
#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_applications/turtlebot3_automatic_parking_vision/launch/turtlebot3_automatic_parking_vision.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(25)
# 3 seconds later
launch.shutdown()



#open arm node
#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/launch/turtlebot3_manipulation_bringup.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)

#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/move_group.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)

#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/moveit_rviz.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)


#door push
moveit_commander.roscpp_initialize(sys.argv)
#rospy.init_node('Project', anonymous=False)
base=SAPObject()
rospy.sleep(3)
# move to home position
base.move_homepos()
base.Closegripper()
rospy.sleep(4)
#move to taget position
x = 0.25
y = 0.00
z = 0.27
position=[x , y , z]
base.move_targetpos(position)
rospy.sleep(5)
base.move_wheel(-0.12)
rospy.sleep(3)
# move to home position and ready to grip
base.move_homepos()
rospy.sleep(3)
base.Opengripper()


#ar mark
#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_applications/turtlebot3_automatic_parking_vision/launch/turtlebot3_automatic_parking_vision.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(10)
# 3 seconds later


#open arm node
#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/launch/turtlebot3_manipulation_bringup.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)

#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/move_group.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)

#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/moveit_rviz.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)




# Initialize moveit_commander and node
moveit_commander.roscpp_initialize(sys.argv)
#rospy.init_node('Project', anonymous=False)
base=SAPObject()
rospy.sleep(2)
#move to taget position
x = 0.27
y = 0.00
z = 0.25
position=[x , y , z]
base.move_targetpos(position)
rospy.sleep(5)
base.Closegripper()
rospy.sleep(2)
base.move_homepos()
rospy.sleep(5)
# move to home position and ready to grip
base.move_wheel(-0.15)

#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation.launch"])
launch.start()
rospy.loginfo("started")

GL.GL()
searching.searching()


patrol.patrol([[(0,0,0.0),(0.0,0.0,0,1.0)]])


#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/launch/turtlebot3_manipulation_bringup.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)

#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/move_group.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)

#rospy.init_node('Project', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/moveit_rviz.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(3)

# Initialize moveit_commander and node
moveit_commander.roscpp_initialize(sys.argv)
#rospy.init_node('Project', anonymous=False)
base=SAPObject()
rospy.sleep(2)
base.rotate_wheel(0.4)
rospy.sleep(6)
#move to taget position
x = 0.20
y = 0.00
z = 0.20
position=[x , y , z]
base.move_targetpos(position)
rospy.sleep(5)
base.Opengripper()
rospy.sleep(2)
base.move_homepos()




'''
patrol.patrol([[(0,0,0.0),(0.0,0.0,0,1.0)]])
patrol.patrol([[(3.20,-0.08,0.0),(0.0,0.0,0.0,1.0)]])
patrol.patrol([[(3.20,-0.45,0.0),(0.0,0.0,0.0,1.0)]])
patrol.patrol([[(3.20,0.45,0.0),(0.0,0.0,0.0,1.0)]])
base=SAPObject()
base.move_wheel(0.1)
'''


