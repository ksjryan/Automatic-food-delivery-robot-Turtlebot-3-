#! /usr/bin/env python2
     
import rospy
from std_srvs.srv import Empty, EmptyRequest
import sys 

def GL():
	rospy.init_node('Project', anonymous=False)
	rospy.wait_for_service('/global_localization')
	disperse_particles_service = rospy.ServiceProxy('/global_localization', Empty)
	msg = EmptyRequest()
	result = disperse_particles_service(msg)
	print (result)

if __name__ == '__main__':
    try:
        # Testing our function
        GL()
    except rospy.ROSInterruptException:
        pass
