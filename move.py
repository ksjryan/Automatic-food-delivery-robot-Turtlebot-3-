#! /usr/bin/evn python2
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897


def move():
	velocity=0.05
	length=0.05
	current_length = 0
	k=0

	# go forward
	rospy.init_node('Project', anonymous=False)
	pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rate=rospy.Rate(10)
	mov=Twist()
	mov.linear.x=velocity

	t2 = rospy.Time.now().to_sec()
	while (k<3):
	    pub.publish(mov)
	    rate.sleep()

	    t0 = rospy.Time.now().to_sec()

	    while(current_length < length):
		print('b')
		pub.publish(mov)
		t1 = rospy.Time.now().to_sec()
		current_length = velocity*(t1-t0)

	    mov.linear.x = 0
	    t = rospy.Time.now().to_sec()
	    k=100
	mov.linear.x = 0
	pub.publish(mov)

if __name__ == '__main__':
    try:
        # Testing our function
        move()
    except rospy.ROSInterruptException:
        pass
