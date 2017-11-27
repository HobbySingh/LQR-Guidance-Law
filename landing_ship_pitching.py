#!/usr/bin/env python
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn, VFR_HUD, WaypointList
from sensor_msgs.msg import TimeReference
import rospy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import sys, random, math
from math import sqrt,cos,sin,atan2
import os
import roslib
import numpy as np
import csv
import time
import tf
import matplotlib.pyplot as plt
import numpy
from datetime import datetime
from time import time
from mavros_msgs.srv import SetMode, CommandBool

#Variable Initialise
#wp load /home/mandeep/Desktop/waypoint1.txt

###############################Initaializing Global Variables######################################
height = 0
vx = 0
vy = 0
vz = 0
wz = 0
v = 0
throttle = 0
airspeed = 0
groundspeed = 0
curr_x = 0
curr_y = 0
roll = 0
yaw = 0
pitch = 0
pitch_rate = 0
first_waypt = [0,0]
pehlibar = True
g = 9.8 
waypoint_6 = False # waypoint before landing point | enter ino cruise mode after passing into this mode
successful_landing = False
#####################################################################################

def getVelocity(measurement):
    global vx,vy,v
    x=measurement.twist.linear.x
    y=measurement.twist.linear.y
    z=measurement.twist.linear.z
    vx=math.sqrt(x ** 2)
    vy=math.sqrt(y ** 2)
    vz=math.sqrt(z ** 2)
    v = math.sqrt((vx**2) + (vy**2))

def getWind(measurement):
    global wz
    wz=measurement.twist.linear.z

def dist(a,b):
    return math.sqrt((a[1]-b[1])**2+(a[0]-b[0])**2)

############################## Make home as (0,0) and localize the plane position respect to that######################### 
def getGPS(measurement):
    global curr_x,curr_y,pehlibar,first_waypt
    x=measurement.pose.pose.position.x
    y=measurement.pose.pose.position.y
    if pehlibar == True:
        first_waypt = [x,y]
        #print "Origin: ",first_waypt
        pehlibar = False
    curr_x,curr_y = getXY_waypt(x,y,first_waypt[0],first_waypt[1],0,0)
    #if (curr_x != 0):
    #    slope = curr_y/curr_x
    #print "Curr_x : ", curr_x, "Curr_y : ", curr_y
    #print "Slope : ", to_deg(math.atan2(curr_y,curr_x))
    #print "Curr_Distance from home", sqrt(curr_x*curr_x + curr_y*curr_y) 


def getsome(measurement):
    global throttle, airspeed, groundspeed
    throttle=measurement.throttle
    airspeed = measurement.airspeed
    groundspeed = measurement.groundspeed

def getAltitude(measurement):
    global height
    height = measurement.data
    if height < 0:
        height = -1*height
    #print height    

def getPitch(measurement):
    global pitch, roll, yaw, pitch_rate
    a = measurement.orientation.x
    b = measurement.orientation.y
    c = measurement.orientation.z
    d = measurement.orientation.w
    [r,p,y] = tf.transformations.euler_from_quaternion((a,b,c,d))
    pitch = p
    roll = r
    yaw = y
    #print "Yaw value ", to_deg(yaw)
    pitch_rate = measurement.angular_velocity.y
    #print pitch

def letsland():
    #################################### Initializing Variables ######################
    global psi,dist_home, change_mode, waypoint_6, height, vx, vy, vz, wz, v, curr_x, curr_y, roll, pitch, yaw, g, first_waypt, throttle, groundspeed, airspeed
    
    h = [] # height for plotting data
    d = [] # dist. from landing point for plotting data
    t = [] # expected glise slope for plotting
    p = [] # pitch graph
    r = []
    y = []
    x = []
    zer = []
    aileron = []
    sim_tim = []

    time_counter = 0
    runway = [] # runway position points
    tmp = []

    #######Introducing noise in ship pitch and altitude
    mean = 0
    std = 1

    V = 0 # resultant velocity in 3d  
    pwm = 0 # pwm value for pitch
    dist_home = 0

    cte = 0 #cross track error
    prev_cte = 0
    prev_cte_xy = 0
    max_diff = 0# max. error in expected altitude vs real altitude 

    #### PID constants 
    kp = 0.23#0.28 #0.4 #13 
    kd = 0.4#0.5#0.1 # 15
    ki = 0.001 #0.005

    kp_xy = 0.025#1 #0.5 
    kd_xy = 0.09 #1.5 # 15
    ki_xy = 0.0001 #0.005    

    k = 0.05
    q2 = 0.0001#0.0001#1.2    

    angle = 3 # glide_slope_angle
    offset = 6
    ship_speed = 5
    ship_pitch = 0

    time_speed = 0.2

    f_obj1 = open("/home/mandeep/Desktop/sea_state3.dat","w")

    print "Param Values kp : " +  str(kp) + " kd : " + str(kd) + " ki : " + str(ki) + " offset : " + str(offset) + " SHip speed : " + str(ship_speed) + " Time Speed : ", str(time_speed) 
    print "Param Values kp_xy : " +  str(kp_xy) + " kd_xy : " + str(kd_xy) + " ki_xy : " + str(ki_xy) + " K: ", str(k) + " q2: ", str(q2)

    # PID related variable
    proportional = 0
    integral = 0
    derivative = 0
    start_integral = 1

    proportional_xy = 0
    integral_xy = 0
    derivative_xy =0
    start_integral_xy = 1
    total_cte_xy = 0

    prev_pitch = 0
    previous_error = 0
    prev_theta_des = 0  
    prev_psi = 0

    previous_error_xy = 0
    prev_theta_des_xy = 0
    prev_y = 0 
    prev_derivative_xy = 0

    landed = 0 # flag to check landed or not
    lp_x = 0  # Landing Point X
    lp_y = 0  # Landing Point Y
    lp_offset = 0  

    csv_path = "/home/mandeep/Downloads/waves.dat"
    f_obj = open(csv_path,"r")
    #ship_pitch = (float(f_obj.readline()[:-1]))/2    

    # Variables to calculate landng time from second last waypoint
    initial_time = time()
    current_time = 0
    string = "Moving Runway (kp : " + str(kp) + " kd : " + str(kd)  + " ki : " + str(ki) + " offset : " + str(offset) + " ship_speed : " + str(ship_speed)  + ")"
    flare_param = 0
    slope = 0 # slope for calculating moving landing point from home positioning
    dist_land = 0 # distance from landing point

    # intializing topics and services
    pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1000)
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('/mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/arming')

    control = OverrideRCIn()

    while (waypoint_6 == False): # wait till second last waypoint
        slope = 0 # no special purpose

    resp1 = change_mode(custom_mode="FBWB")
    print (resp1)
    approach_time = time()
    time_loop = time() # for printing at 1 sec interval
    time_to_land = time() 

    #print "Height", " ", "PWM", " ", "Elev_Angle", " ", "Error_Alt", " ", "Pitch", " ", "Pitch_Rate", " ", "Throttle", " ", "AirSpeed", " ", "GroundSpeed"
    
    while(landed == 0): 

        current_time = time()

        # Calculating moving landing point for moving at 5 m/s
        if(current_time - initial_time > 1):
            initial_time = current_time
            lp_offset = lp_offset + ship_speed
            lp_x,lp_y = landing_point(lp_x, lp_y, lp_offset, curr_x, curr_y, slope)
            print "Dist from Landing Point" , dist_land


        # for printing kabhii kabhii
        if(time() - time_loop > time_speed):

            psi_local = psi 

            time_counter = time_counter + time_speed

            ship_pitch = ((float(f_obj.readline()[:-1])))#/15
            print " Ship Pitch :", ship_pitch
            #angle = abs(3)# + (ship_pitch)/4)
            V = sqrt(vx*vx + vy*vy + vz*vz)

            # distance between two points
            dist_land = sqrt((curr_x - lp_x)*(curr_x - lp_x))
            # desired altitude = dist_from_home * tan (3)
            #print "dist_land", dist_land
            h_des = dist_land*math.tan(to_rad(angle)) + (ship_pitch) + 6
            #print 'Current Altitude',  height
            #print "H_Des", h_des
            h_des_dot = (sqrt(vx*vx + vy*vy) - ship_speed)*math.tan(to_rad(angle))
            #print "Velocity" , v
            #print "H_Des_Dot", h_des_dot
            error_alt = (height - h_des)
            error_xy = curr_y
            #print "Error_xy1 : ", error_xy 
            #print "error Alt", error_alt
            error_alt_rate = vz - h_des_dot   

            R = sqrt((curr_x)*(curr_x) + (curr_y)*(curr_y))
            #theta_M = math.atan2(curr_y,curr_x + 5000)
            theta_M = math.atan2(curr_x + 5000,-curr_y)

            V_xy = sqrt(vx*vx + vy*vy)

            print "Curr_x : ", curr_x , "Curr_y : ", curr_y
            if(abs(psi_local - prev_psi) > 1 and prev_psi != 0):
                psi_local = prev_psi
                print " GOT in IF "
            print "Psi Degrees : ", psi_local
            print "Theta_M Degrees: ", to_deg(theta_M) 
            print "Sin(psi - 90)", math.sin(to_rad(psi_local - 90))             
            psi_local_rad = to_rad(psi_local - 90)
            error_xy_rate = (V_xy*0.1)*math.sin(psi_local_rad)# - theta_M)

            q1 = math.exp(-k*abs(error_xy))#-0.7

            #if((abs(error_xy) < 0.2) and (abs(sqrt(q1)*error_xy) < abs(sqrt(2*sqrt(q1) + q2)*error_xy_rate))):
            #    error_xy_rate = (V/3)*math.sin(psi_local_rad - theta_M)
            print " V_xy : ", V_xy
            print "error_xy_rate : ", error_xy_rate 
            print "error_xy : ", error_xy  
            '''
            q1 = -0.7
            q2 = 1.2
            gain = 3
            '''

            if (V == 0):
                theta_des = (1.5*((-0.35*error_alt) + (sqrt(0.79)*error_alt_rate)))
                yaw_des = (-sqrt(q1)*error_xy + sqrt(2*sqrt(q1) + q2)*error_xy_rate)
                #yaw_des = (gain*((q1*error_xy) + (sqrt(q2)*error_xy_rate)))
            else:
                theta_des = (1.5*((-0.35*error_alt) + (sqrt(0.79)*error_alt_rate)))/V
                yaw_des = (-sqrt(q1)*error_xy + sqrt(2*sqrt(q1) + q2)*error_xy_rate)#/V
                #yaw_des = (gain*((q1*error_xy) + (sqrt(q2)*error_xy_rate)))/V
            #print "Theta des", theta_des5

            print "sqrt(q1)*error_xy : ", sqrt(q1)*error_xy
            print "sqrt(2*sqrt(q1) + q2)*error_xy_rate : ", sqrt(2*sqrt(q1))*error_xy_rate    
            print "Yaw des", yaw_des
            print "theta_Des ", theta_des

            # cross track error in pitch
            cte_xy = roll - yaw_des
            cte = pitch - theta_des

            proportional = kp*(cte)
            derivative = kd*(cte - prev_cte)
            if(start_integral):
                integral = integral + ki*(cte)
            
            eta = proportional + derivative + integral
            #print "Pitch - Theta des", (pitch - theta_des)
            ele_angle = math.degrees(float(eta)) + offset # -27 for auto mode
            #print "Elevator angles", ele_angle
            if (ele_angle > 25):
                start_integral = 0
                ele_angle = 25
            if (ele_angle < -25):
                ele_angle = -25
                start_integral = 0
            if(25 > ele_angle > -25):
                start_integral = 1 
            #print "Pitch angle in degree", ele_angle


            proportional_xy = kp_xy*(cte_xy)
            derivative_xy = kd_xy*(cte_xy - prev_cte_xy)
            #if (derivative_xy == 0):
            #    derivative_xy = prev_derivative_xy
            total_cte_xy = total_cte_xy + cte_xy
            if(start_integral_xy):
                integral_xy = ki_xy*(total_cte_xy)
            eta_xy = proportional_xy + derivative_xy + integral_xy

            #print "Pitch - Theta des", (pitch - theta_des)
            yaw_angle = math.degrees(float(eta_xy)) #+ 1 #10

            #print "Elevator angles", ele_angle
            if (yaw_angle > 25):
                start_integral_xy = 0
                yaw_angle = 25
            if (yaw_angle < -25):
                yaw_angle = -25
                start_integral_xy = 0
            if(25 > yaw_angle > -25):
                start_integral_xy = 1

            #print "Pitch pwm actual what should be going", (1500 + ele_angle*11)
            pwm = 1500 + ele_angle*11
            pwm_xy = 1500 + yaw_angle*11
            #print "PWM Value", pwm
            if((time() - approach_time) > 20): #20            
                control.channels[1] = pwm
                control.channels[0] = pwm_xy
                control.channels[4] = 1500                       
                pub.publish(control)
                #print "Control getting published"
            else:
                control.channels[0] = pwm_xy
                control.channels[4] = 1500                
                pub.publish(control)                
                proportional = 0
                derivative = 0
                integral = 0
                eta = 0
                print "Control not getting published"  

            
            #print "Curr_x : ", curr_x, " Curr_y : ", curr_y, "lp_x : ", lp_x, "lp_y : ", lp_y            
            print "Altitude : ", height            
            #print "error_xy : ", error_xy, " error_xy_rate : ", error_xy_rate
            '''
            if(yaw_des < 0):
                print "Right Side Desired"
            else:
                print "Left Side Desired"
            if(roll < 0):
                print "Tilted Right Side"
            else:
                print "Tilted Left Side"    
            #print "q1 : ", q1
            #print "q1*error_xy :",  q1*error_xy
            #print "sqrt(2*q1 + (q2*q2))*error_xy_rate : ", sqrt(2*q1 + (q2*q2))*error_xy_rate

            print "yaw_des : ", yaw_des
            print "yaw_des : ", yaw_des, " =  3*( ( ", -0.35*error_xy, " ) + ( ", sqrt(0.79)*error_xy_rate, " ) )/ ", V 
            '''
            
            print "Current Roll : ", roll , "Roll_des : ", yaw_des
            print "cte_xy : ", cte_xy, " = ( ", roll , " - ", yaw_des , " ) "
            print "                           eta_xy : ",eta_xy, " = proportional :", proportional_xy, " derivative : ", derivative_xy, " integral : " , integral_xy, "eta : ", eta_xy 
            print" Demanded Yaw angle : ", yaw_angle
            print "                           pwm_xy : ", pwm_xy ,"1500 + ( ", yaw_angle," )*11" 
            print "roll in degree : ", to_deg(roll)
            print "Dist from Landing Point" , dist_land
            #print height, " ", pwm, " ", ele_angle, " ", error_alt, " ", pitch, " ", pitch_rate, " ", throttle, " ", airspeed, " ", groundspeed   
            
            #print height, " ", pwm, " ", ele_angle, " ", error_alt, " ", pitch, " ", pitch_rate, " ", throttle, " ", airspeed, " ", groundspeed   
            
            # store value in array for plotting in end
            if (curr_x < 0):
                #dist_home = sqrt(curr_x*curr_x + curr_y*curr_y)           
                dist_home = (-1)*sqrt(curr_x*curr_x)
            else:
                #dist_home = (-1)*sqrt(curr_x*curr_x + curr_y*curr_y)
                dist_home = (1)*sqrt(curr_x*curr_x)
            runway.append(sqrt((lp_x)*(lp_x)))
            tmp.append(ship_pitch)
            t.append(h_des) 
            h.append(height)
            d.append(dist_home)
             # its common
            p.append(to_deg(-pitch))
            sim_tim.append(time_counter)
            y.append(curr_y)
            x.append(curr_x)
            aileron.append(yaw_angle)
            zer.append(0)             
            # 10 things being written into file
            # dist, height, r, p, y, time, ship_pitch, position, error_alt , ele_angle
            f_obj1.write(str(dist_home) + "," + str(height) + "," + str(to_deg(roll)) + "," + str(to_deg(-pitch)) + "," + str(to_deg(yaw)) + "," + str(time_counter) + "," + str(ship_pitch) + "," + str(lp_x) + "," + str(error_alt) + "," + str(ele_angle) + "," + str(curr_y) + "," + str(curr_x) + "," + str(yaw_angle) + "\n")
       

            if (abs(dist_land*math.tan(to_rad(angle)) - height) > max_diff):
                max_diff = abs(dist_home*math.tan(to_rad(angle)) - height) 

            prev_pitch = pitch
            previous_error = error_alt
            prev_theta_des = theta_des
            prev_cte = cte

            prev_cte_xy = cte_xy
            prev_y = curr_y
            prev_derivative_xy = derivative_xy             

            prev_psi = psi_local
            print "Prev_Psi : ", prev_psi
            time_loop = time()            
        # cut throttle and pitch high (flare)
        #height <= 0.2
        if(((1 + ship_pitch < height < 5 + ship_pitch) and (14 < (curr_x - lp_x) < 15)) or (height < 0.2 + ship_pitch)) :
            time_to_land = time() - time_to_land
            #flare_time = time() - flare_time
            landed = 1
            control.channels[2] = 1100
            control.channels[1] = 1100
            control.channels[0] = 1500
            control.channels[3] = 1500
            pub.publish(control)

            # disarm
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armService(False)
            except rospy.ServiceException, e:
                print "Service arm call failed: %s"%e

    #print "Time to Land from last waypoint" , time_to_land
    #print "Flare before ", flare_time, " seconds"
    #print " Final Landing Point Distance", sqrt(lp_x*lp_x + lp_y*lp_y)
    #print d
    f_obj1.close()
    print "Curr_x : ", curr_x, "Curr_y : ", curr_y, " Lp_x : ", lp_x, " Lp_y : ", lp_y, "Altitude : ", height
    print "Max Error in Altitude", max_diff
    print "Distance from Landing Point ", curr_x - lp_x

    if( (14 < (curr_x - lp_x) < 15) and ( 1 + ship_pitch < height < 5 + ship_pitch) ):
        successful_landing = True
    else:
        successful_landing = False

    print "Landing Successful : ", successful_landing

    plt1 = plt.scatter(d,h,color = 'blue',linewidth = 0.2) #actual uav glidesope     
    plt2 = plt.scatter(d,t, color = 'red') # plot expected glideslope
    plt3 = plt.scatter(runway,tmp,color = 'green') # plot runway position

    plt.title(string)
    plt.xlabel("Dist. from Landing Point in m")
    plt.ylabel("Height of UAV in m")        
    #plt.legend((plt1, plt3),('UAV Glidslope', 'Runway Position'))
    plt.legend((plt1, plt2, plt3),('UAV Glidslope', 'Expected Glideslope', 'Runway Position'))    
    #plt.axis([-4500, 4000, -50, 500])

    plt.show()         

    plt4 = plt.scatter(sim_tim,p,color = 'blue')
    plt.show()

    string = "Kp:"+ str(kp_xy)+ " kd:"+str(kd_xy)+ " ki:"+str(ki_xy)+ " lqr_gain:"+str(k)# + " gain:"+str(gain) 
    plt.title(string)
    plt5 = plt.scatter(d,y)
    plt.show()  

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, h)
    plt.show()

def landing_point(lp_x,lp_y,lp_offset, curr_x,curr_y,m):
    #m - slope
    dist = 2*lp_offset
    lp_x = dist/sqrt(m*m + 1)
    lp_y = m*lp_x
    #print "Landing Point , slope :", lp_x,lp_y,m 
    return lp_x,lp_y
#######   HAVERSINE START
def getXY_waypt(x,y,origin_x,origin_y,pre_x,pre_y):
    '''
    latrad = to_rad(lat)
    lonrad = to_rad(lon)
    lat_pre = to_rad(lat_pre)
    lon_pre = to_rad(lon_pre)
    '''
    if x==origin_x:
        pre_x=0
        pre_y=0
    else:
        pre_x = x - origin_x
        pre_y = y - origin_y
    return pre_x,pre_y
    

def to_rad(deg):
    rad=(deg*math.pi)/180.0 
    return rad

def to_deg(rad):
    deg=(rad*180)/math.pi
    return deg

def haversine_waypt(lat1,lon1,lat2,lon2,pre_x,pre_y):
    r=6372.8;
    
    dlat = lat2-lat1
    dlon = lon2-lon1
    a = (math.sin(dlat/2))**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2))**2
    c = 2 * math.atan2(math.sqrt(a),math.sqrt(1-a))
    d = r * c * 1000 
    y = math.sin(dlon) * math.cos(lat2);
    x = -1 * (math.sin(lat1)*math.cos(lat2)*math.cos(dlon)-math.cos(lat1)*math.sin(lat2))
    brng = math.fmod(math.atan2(y, x)+2*math.pi,2*math.pi)
    pre_x = pre_x +(d * math.cos(brng))
    pre_y = pre_y +(d * math.sin(brng))
    '''
    origin_x = r*math.cos(lat2)*cos(lon2)
    origin_y = r*math.cos(lat2)*sin(lon2)

    curr_x = r*math.cos(lat1)*cos(lon1)
    curr_y = r*math.cos(lat1)*sin(lon1)

    pre_x = curr_x - origin_x
    pre_y = curr_y - origin_y
    '''
    return pre_x,pre_y
def waypoint_update(measurement):
    global waypoint_6
    waypoints = measurement.waypoints
    #print "Size of waypoints ", len(waypoints)
    waypoint_6 = waypoints[6].is_current
    print "IS waypoint 6 arrived", waypoint_6

def get_psi(measurement):
    global psi
    psi = measurement.data

def listener():
    global change_mode
    rospy.init_node('listener', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    #r = rosrate(0.05)
    # rospy.Subscriber("/mavros/global_position/global", NavSatFix, plot_gps_measurements)
    rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_update)
    rospy.Subscriber("/mavros/VFR_HUD", VFR_HUD, getsome)
    rospy.Subscriber("/mavros/global_position/local", Odometry, getGPS)
    rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, getVelocity)
    rospy.Subscriber("/mavros/wind_estimation", TwistStamped, getWind)
    rospy.Subscriber("/mavros/imu/data", Imu, getPitch)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, getAltitude)
    rospy.Subscriber("/mavros/global_position/compass_hdg",Float64,get_psi)     

 
    #wp load 
    letsland()
    rospy.spin()

if __name__ == '__main__':
    listener()