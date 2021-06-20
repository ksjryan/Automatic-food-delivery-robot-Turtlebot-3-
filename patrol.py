#!/usr/bin/env python2

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rospy import client

#(3.0,-0.8,0.0)
#(3.0,-0.25,0.0)
#(3.0,0.4,0.0)

#patrol point
waypoint=[[(1,0.5,0.0),(0.0,0.0,0.0,1.0)]]


def goal_pose(pose):
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id='map'
    goal.target_pose.pose.position.x=pose[0][0]
    goal.target_pose.pose.position.y=pose[0][1]
    goal.target_pose.pose.position.z=pose[0][2]
    goal.target_pose.pose.orientation.x=pose[1][0]
    goal.target_pose.pose.orientation.y=pose[1][1]
    goal.target_pose.pose.orientation.z=pose[1][2]
    goal.target_pose.pose.orientation.w=pose[1][3]

    return goal

def patrol(point):
    rospy.init_node('Project')
    client=actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    for pose in point:
        goal=goal_pose(pose)
        client.send_goal(goal)
        wait=client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            client.get_result()


if __name__=='__main__':

    try:
        patrol(waypoint)
    except rospy.ROSInterruptException:
        pass

