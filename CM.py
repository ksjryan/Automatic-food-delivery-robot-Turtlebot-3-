#! /usr/bin/env python2
     
import rospy
from std_srvs.srv import Empty, EmptyRequest
import sys 

def CM():
	rospy.init_node('Project', anonymous=False)
	rospy.wait_for_service('/move_base/clear_costmaps')
	disperse_particles_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
	msg = EmptyRequest()
	result = disperse_particles_service(msg)
	print (result)

if __name__ == '__main__':
    try:
        # Testing our function
        CM()
    except rospy.ROSInterruptException:
        pass
