#!/usr/bin/env python

import rospy
import numpy as np
from numpy import linalg as LA
from math import sin, cos, atan2, radians, sqrt, fmod, pi, exp
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointList
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, Point

def send_control(v,si_dot):

	global cmd_vel, uav_hdg

	#print "Angular Rate :", si_dot
	#print "Uav Heading :", uav_hdg
	#print "Velocity in X :", v*cos(uav_hdg)
	#print "Velocity in Y :", v*sin(uav_hdg)

	cmd_vel.twist.linear.y = v*cos(uav_hdg)
	cmd_vel.twist.linear.x = v*sin(uav_hdg)
	cmd_vel.twist.angular.z = si_dot

	vel_pub.publish(cmd_vel)

def wp_check(wp_no):

	global wps_xy, uav_xy

	wp2_xy = np.array(wps_xy[wp_no+1])
	uav_wp_dist = sqrt(pow((wp2_xy[0] - uav_xy.x),2)+pow((wp2_xy[1] - uav_xy.y),2)) 

	#print""
	#print "WP Dist :", uav_wp_dist
	#print ""

	if (uav_wp_dist < 3):
		print "WP Changed"
		return wp_no + 1
	else:
		return wp_no

def lqr_control(wp_no):
	global wps_xy, uav_xy, uav_vel, uav_hdg 

	wp1_xy = np.array(wps_xy[wp_no])
	wp2_xy = np.array(wps_xy[wp_no+1])
	uav_pose_xy = np.array([uav_xy.x,uav_xy.y])
	d_xy = xtrack_err(uav_pose_xy,wp1_xy,wp2_xy)
	print "Cross Track Error :", d_xy

	# LQR Tuning Variables
	Rmin = 6
	q2 = 1
	k = 0.0001
	q1 = sqrt(exp(k*(abs(d_xy))))
	print "Q1 :", q1

	vy = uav_vel.y
	vx = uav_vel.x
	course_angle = atan2(vy, vx)
	desired_course = atan2((wp2_xy[1] - wp1_xy[1]),(wp2_xy[0] - wp1_xy[0]))
	print "Desired Course :", desired_course

	v = sqrt(vx*vx + vy*vy)
	print "Velocity v :", v
	print "UAV heading :", uav_hdg
	vd = v*sin(uav_hdg - desired_course)
	print "D dot :", vd  

	u = (q1*d_xy + sqrt(2*q1 + q2*q2)*vd)
	print "Value of U :", u
	# Constraining the control input
	
	if(abs(u) > (v*v)/Rmin):
		print "Constrains Triggerd"

		if (u > 0):
			u = (v*v)/Rmin
		else:
			u = -(v*v)/Rmin	
	
	si_dot = u/v
	print "Si_dot : ", si_dot


	#print "Course Angle : ",course_angle

	return si_dot

def xtrack_err(uav_pose,wp_1,wp_2):

	a = wp_1 - wp_2
	b = uav_pose - wp_2
	d = LA.norm(np.cross(a,b)) / LA.norm(a)

	# To check whether the point is left or right of the track
	tmp = (uav_pose[0] - wp_1[0])*(wp_2[1] - wp_1[1]) - (uav_pose[1] - wp_1[1])*(wp_2[0] - wp_1[0]);
	if(tmp < 0):
		return d
	else:
		return -d

def callbac_get_wps(msg):
    global wps_gps
    # wp_gps format [[lat1,lon1],[lat2,lon2]]
    waypoints = msg.waypoints

    for i in range(0,len(waypoints)):
    	wp = [waypoints[i].x_lat,waypoints[i].y_long]
    	wps_gps.append(wp)

def convert_wps_xy(wps_gps,home_gps):

	for i in range(0,len(wps_gps)):
		wp_lat = radians(wps_gps[i][0])
		wp_lon = radians(wps_gps[i][1])

		wp_x, wp_y = convert_2_xy(home_gps.x,home_gps.y,wp_lat,wp_lon,0,0) 
		wps_xy.append([wp_x,wp_y])

	#print wps_xy
	return wps_xy

def convert_2_xy(pt_1_lat,pt_1_lon,pt_2_lat,pt_2_lon,curr_x,curr_y):
	r=6371;
	diff_lat=pt_2_lat-pt_1_lat
	diff_lon=pt_2_lon-pt_1_lon
	a =  (sin(diff_lat/2.0) * sin(diff_lat/2.0)) + cos(pt_1_lat) * cos(pt_2_lat) * (sin(diff_lon/2.0) * sin(diff_lon/2.0));
	c = 2 * atan2(sqrt(a), sqrt(1-a));
	d = r * c * 1000;
	y = sin(diff_lon) * cos(pt_1_lat);
	x = sin(pt_1_lat)*cos(pt_2_lat)*cos(diff_lon)-cos(pt_1_lat)*sin(pt_2_lat);
	x=-x;
	brng = fmod(atan2(y, x)+2*pi,2*pi);
	curr_x = curr_x +(d * cos(brng));
	curr_y = curr_y +(d * sin(brng));
	return curr_x, curr_y

def callbac_uav_pose(msg):
	global home_set, uav_gps, home_gps, uav_xy, wps_gps, wps_xy

	uav_gps.x = radians(msg.latitude)
	uav_gps.y = radians(msg.longitude)

	if home_set != True:
		# Take first gps lock as home position
		home_gps.x = radians(msg.latitude)
		home_gps.y = radians(msg.longitude)
		home_set = True
		wps_xy = convert_wps_xy(wps_gps,home_gps)
		print wps_xy
 
	uav_xy.x, uav_xy.y = convert_2_xy(home_gps.x,home_gps.y,uav_gps.x,uav_gps.y,0,0)
	#print "UAV Y : ", uav_y
	#print "UAV X : ", uav_x

def callbac_uav_hdg(msg):
	global uav_hdg
	#print "UAV Heading in Degree :", msg.data
	uav_hdg = radians(msg.data)

def callbac_uav_vel(msg):
	global uav_vel

	uav_vel.x = msg.twist.linear.x
	uav_vel.y = msg.twist.linear.y

################################################## Global Variables ##############################
rospy.init_node ('lqr_path_following',anonymous = True)

home_set = False
home_gps = Point()

uav_gps = Point()
uav_hdg = 0
uav_xy = Point()
uav_vel = Point()

wps_xy = []
wps_gps = []

cmd_vel = TwistStamped()
##################################################  Subscribers  ##################################

# UAV poses in lat lon and heading
rospy.Subscriber("/mavros/global_position/global", NavSatFix, callbac_uav_pose, queue_size=100)
rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, callbac_uav_hdg, queue_size=100)
rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, callbac_uav_vel, queue_size=100)
# get mission waypoints
rospy.Subscriber("/mavros/mission/waypoints", WaypointList, callbac_get_wps, queue_size=100)


##################################################  Publishers  ###################################

vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 100 )


rate = 10
loop_rate = rospy.Rate(rate)

def path_following():

	global wps_xy

	print "LQR Path Following Node Initialized"
	wp_no = 0
	total_wps = len(wps_xy)
	v = 2
	while not rospy.is_shutdown():

		if home_set:

			wp_no = wp_check(wp_no)

			#if wp_no == total_wps:
				#break 

			si_dot = lqr_control(wp_no)

			send_control(v,si_dot)

		loop_rate.sleep()




if __name__ == '__main__':
	try:
		path_following()
	except rospy.ROSInterruptException:
		pass 





