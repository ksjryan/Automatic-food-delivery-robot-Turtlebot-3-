#! /usr/bin/evn python2
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

speed = 90
angle = 5.5
	
def rotate():
	angular_speed = speed*2*PI/360
	relative_angle = angle*2*PI/360

	current_angle = 0

	# rotate 90 degree (clockwise)
	rospy.init_node('Project', anonymous=False)
	pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	rate=rospy.Rate(1)
	mov=Twist()
	mov.angular.z=-abs(angular_speed)

	t2 = rospy.Time.now().to_sec()

	pub.publish(mov)
 	rate.sleep()

	t0 = rospy.Time.now().to_sec()

	while(current_angle < relative_angle):
		print('a')
		pub.publish(mov)
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)

	mov.angular.z = 0
	t = rospy.Time.now().to_sec()
	mov.angular.z = 0
	pub.publish(mov)



if __name__ == '__main__':
    try:
	rotate()
    except rospy.ROSInterruptException:
        pass
