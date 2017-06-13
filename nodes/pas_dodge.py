#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Pose, Point, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

rospy.init_node('pas_dodge', anonymous=True)

Odom = Odometry()
Cmd_vel = Twist()
Robot_marker = Marker()
Path_marker = Marker()
Ref_marker = Marker()

# path function parameters
T_step =100.0	# time step in ms
pi = 3.1416
Ax = 1.0	# amplitude
Ay = 1.0
Px = 2500	# period in ms
Py = 5000
Lx = 1000.0	# phase lag
Ly = 0.0

#make as dynamic reconfigure parameters
# PID parameters
Kpx = 5.0 	# PID gain
Kdx = 2.0
Kpy = 5.0
Kdy = 2.0
X_vel_thres = 3.0
Y_vel_thres = 3.0

# dun need to touch
Ref_x = 0.0
Ref_y = 0.0
Pre_er_x = 0.0
Pre_er_y = 0.0

Robot_marker.header.stamp = rospy.get_rostime();
Robot_marker.header.frame_id = "odom";
Robot_marker.ns = "points";
Robot_marker.type = Robot_marker.SPHERE
Robot_marker.action = Robot_marker.ADD
Robot_marker.id = 0
Robot_marker.scale.x = 0.3
Robot_marker.scale.y = 0.3
Robot_marker.scale.z = 0.3
Robot_marker.color.g = 1.0
Robot_marker.color.a = 1.0

Ref_marker.header.stamp = rospy.get_rostime();
Ref_marker.header.frame_id = "odom";
Ref_marker.ns = "points";
Ref_marker.type = Robot_marker.SPHERE
Ref_marker.action = Robot_marker.ADD
Ref_marker.id = 1
Ref_marker.scale.x = 0.1
Ref_marker.scale.y = 0.1
Ref_marker.scale.z = 0.1
Ref_marker.color.r = 1.0
Ref_marker.color.a = 1.0

Path_marker.header.stamp = rospy.get_rostime();
Path_marker.header.frame_id = "odom";
Path_marker.ns = "points";
Path_marker.type = Path_marker.POINTS
Path_marker.action = Path_marker.ADD
Path_marker.pose.orientation.w = 1.0
Path_marker.id = 0
Path_marker.scale.x = 0.02
Path_marker.scale.y = 0.02
Path_marker.scale.z = 0.02
Path_marker.color.g = 1.0
Path_marker.color.b = 1.0
Path_marker.color.a = 1.0


Pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
Pub_robot_marker = rospy.Publisher('robot_marker', Marker, queue_size=5)
Pub_path_marker = rospy.Publisher('path_marker', Marker, queue_size=5)
Pub_ref_marker = rospy.Publisher('ref_marker', Marker, queue_size=5)

def odomCallback(data):
	global Pre_er_x, Pre_er_y
	Odom = data

	Robot_marker.pose.position.x = Odom.pose.pose.position.x
	Robot_marker.pose.position.y = Odom.pose.pose.position.y
	Pub_robot_marker.publish(Robot_marker)
	Pub_path_marker.publish(Path_marker)

	cur_er_x = Ref_x - Odom.pose.pose.position.x
	cur_er_y = Ref_y - Odom.pose.pose.position.y
	er_x = cur_er_x - Pre_er_x
	er_y = cur_er_y - Pre_er_y


	Cmd_vel.linear.x = Kpx*cur_er_x + Kdx*er_x
	Cmd_vel.linear.y = Kpy*cur_er_y + Kdy*er_y

	if Cmd_vel.linear.x > X_vel_thres:
		Cmd_vel.linear.x = X_vel_thres
	elif Cmd_vel.linear.x < -X_vel_thres:
		Cmd_vel.linear.x = -X_vel_thres
	if Cmd_vel.linear.y > Y_vel_thres:
		Cmd_vel.linear.y = Y_vel_thres
	elif Cmd_vel.linear.y < -Y_vel_thres:
		Cmd_vel.linear.y = -Y_vel_thres

	Pub_cmd_vel.publish(Cmd_vel)
	Pre_er_x = cur_er_x
	Pre_er_y = cur_er_y


def x_plot(t):
	return Ax*math.sin(2*pi*t*T_step/Px + Lx)

def y_plot(t):
	return Ay*math.cos(2*pi*t*T_step/Py + Ly)

def main():
	global Ref_x, Ref_y
	
	rospy.Subscriber("/odom", Odometry, odomCallback)
	print("-----passive_dodging initialized-----")
	rate = rospy.Rate(1/(T_step/1000))

	points = []
	for num in range(0,1000):
		p = Point()
		p.x = x_plot(num)
		p.y = y_plot(num)
		p.z = 0.0
		points.append(p)
	Path_marker.points = points

	t = 0
	pre_er_x = 0
	pre_er_y = 0
	while not rospy.is_shutdown():

		Ref_x = x_plot(t)
		Ref_y = y_plot(t)
		Ref_marker.pose.position.x = Ref_x
		Ref_marker.pose.position.y = Ref_y
		Pub_ref_marker.publish(Ref_marker)
		
		t += 1
		rate.sleep()
    	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("\n-----Exiting passive_dodging-----")
