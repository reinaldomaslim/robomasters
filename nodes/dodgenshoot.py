#!/usr/bin/env python

""" 
reinaldo and yan paing oo 
active and passive dodging for Robomasters base 
"""


import rospy
import actionlib
import numpy as np
import math
import cv2

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped, Vector3
from sensor_msgs.msg import RegionOfInterest, CameraInfo, LaserScan, Joy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from sklearn.cluster import DBSCAN, KMeans
from sklearn import metrics
from sklearn.metrics.pairwise import euclidean_distances, manhattan_distances
from sklearn.preprocessing import StandardScaler, normalize
from collections import Counter
from time import time

class BaseDodge(object):
    x0, y0, yaw0= 0, 0, 0
    enemy_pos=[]
    clustered_enemy_pos=[]

    ## Movement PID constants
    del_T = 100.0   # time step in ms
    p_ang = 450.0
    i_ang = 0.0
    d_ang = 250.0
    p_x = 250.0
    i_x = 0.6
    d_x = 200.0

    p_y = 240.0
    i_y = 0.9
    d_y = 200.0

    lin_vel_thres = 350.0 # max 660
    ang_vel_thres = 130.0 # max 660
    bias = 1024.0
    pre_ang_error = 0.0
    pre_x_error = 0.0
    pre_y_error = 0.0
    ang_integral = 0.0
    x_integral = 0.0
    y_integral = 0.0
    lin_integral_threshold = 50.0
    ang_integral_threshold = 50.0

    # path function parameters
    T_step =100.0   #time step in ms, 10Hz
    Ax = 1.0    #amplitude
    Ay = 1.0
    Px = 2500   #period in ms
    Py = 5000
    Lx = 1000.0 #phase lag
    Ly = 0.0
    t=0
    path = 1
    counter = 1


    #preferred direction of active dodging
    isleft=True

    #turret control params
    state_x = 0
    state_y = 0
    updatetime = time()

    # PID parameters
    xMax = 640
    yMax = 480

    Kp = 0.51
    Ki = 0
    Kd = 0.6

    target_x = 320
    target_y = 240
    
    errorp_x = 0
    errorp_y = 0

    #cmd vel params
    cmd_x=bias
    cmd_y=bias
    cmd_yaw=bias
    cmd_yaw_turret=bias
    cmd_pitch=bias
    cmd_shoot=0

    def __init__(self, nodename):
        rospy.init_node(nodename, anonymous=False)
    
        rospy.Subscriber("/odometry", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/enemy_yolo", Marker, self.enemy_callback, queue_size = 50)
        rospy.Subscriber('/armor', Vector3, self.armor_callback, queue_size = 50)
        self.cmd_vel_pub=rospy.Publisher("/cmd_vel", Joy, queue_size=10)

        #self.initMarker()

        rate=rospy.Rate(10)
    
        while not rospy.is_shutdown():
            
            if len(self.clustered_enemy_pos)==1:
                #if only one, active dodging 
                self.active_dodge()      
            else:
                #more than one, passive dodge
                self.passive_dodge()
            
            msg=Joy()
            
            #do priority that governs yawing (lidar or vision)
            if time()-self.updatetime<0.15:
                #armor detected
                msg.buttons=[self.cmd_x, self.cmd_y, self.cmd_yaw_turret, self.cmd_pitch, self.cmd_shoot]

            else:
                self.state_x=0
                self.state_y=0
                self.cmd_shoot=0
                self.cmd_pitch=self.bias
                msg.buttons=[self.cmd_x, self.cmd_y, self.cmd_yaw, self.cmd_pitch, self.cmd_shoot]

            self.cmd_vel_pub.publish(msg)

            rate.sleep()

        self.stop()


    def stop(self):
        msg=Joy()
        msg.buttons = [self.bias, self.bias, self.bias, self.bias, 0]
        self.cmd_vel_pub.publish(msg)

    def active_dodge(self):
        if len(self.clustered_enemy_pos)==2:
            target=np.average(np.asarray(self.clustered_enemy_pos))


        target=self.clustered_enemy_pos[0]

        #rotate to face target
        heading=math.atan2(target[1]-self.y0, target[0]-self.x0)
        heading_threshold=50*math.pi/180
        difference=abs(math.atan2(math.sin(self.yaw0-heading), math.cos(self.yaw0-heading)))

        if difference>heading_threshold:

            print("rotate")
            print(math.atan2(math.sin(self.yaw0-heading), math.cos(self.yaw0-heading))*180/math.pi)
            self.rotate(heading)
        else:
            print("translate")
            d=0.3
            #direction to the left
            beta1=heading+math.pi/2
            #direction to the right
            beta2=heading-math.pi/2

            #predict position a timestep ahead
            pred1=[self.x0+d*math.cos(beta1), self.y0+d*math.sin(beta1)]
            pred2=[self.x0+d*math.cos(beta2), self.y0+d*math.sin(beta2)]

            heading1=math.atan2(target[1]-pred1[1], target[0]-pred1[0])
            heading2=math.atan2(target[1]-pred2[1], target[0]-pred2[0])

            if self.inside_arena(pred1)==True and self.inside_arena(pred2)==True:
                #go to preferred direction
                if self.isleft==True:
                    self.translate(pred1[0], pred1[1], heading1)
                else:
                    self.translate(pred2[0], pred2[1], heading2)

            elif self.inside_arena(pred1)==True and self.inside_arena(pred2)==False:
                    self.translate(pred1[0], pred1[1], heading1)
                    self.isleft=True
            elif self.inside_arena(pred1)==False and self.inside_arena(pred2)==True:
                    self.translate(pred2[0], pred2[1], heading2)
                    self.isleft=False
            else:
                #stuck in corner, translate to origin
                self.translate(0, 0, self.yaw0) 



    def x_plot(self,t,Lx,Ay,Ax):
        #print("Ax      : ",Ax)
        #print("Lx      : ",Lx)
        #print("Ay      : ",Ay)
        return Ax*math.sin(3/2*math.pi*t*self.T_step/self.Px + Lx)#*math.sin(1*math.pi*t*self.T_step/self.Px + Lx)


    def y_plot(self,t,Lx,Ay,Ax):
        #print("Ax      : ",Ax)
        #print("Lx      : ",Lx)
        #print("Ay      : ",Ay)
        return Ay*math.cos(2*math.pi*t*self.T_step/self.Py + self.Ly)#*math.sin(1*math.pi*t*self.T_step/self.Px + Lx)

    def passive_dodge(self):

        if self.path == 1:
            ref_x = self.x_plot(self.t,0,0.7,0.25)
            ref_y = self.y_plot(self.t,0,0.7,0.25)


        elif self.path == 2:
            ref_x = self.x_plot(self.t,-26,-0.45,-0.625)
            ref_y = self.y_plot(self.t,26,0.45,0.625)


        elif self.path == 3:
            ref_x = self.x_plot(self.t,-26,0.45,0.625)
            ref_y = self.y_plot(self.t,-26,0.45,0.625)


        elif self.path == 4:
            ref_x = self.x_plot(self.t,0,-0.7,-0.25)
            ref_y = self.y_plot(self.t,0,0.7,0.25)


        elif self.path == 5:
            ref_x = self.x_plot(self.t,26,0.45,0.625)
            ref_y = self.y_plot(self.t,26,0.45,0.625)


        elif self.path == 6:
            ref_x = self.x_plot(self.t,26,-0.45,-0.625)
            ref_y = self.y_plot(self.t,-26,0.45,0.625)

 
        if self.t > self.counter*35:
            self.path += 1
            self.counter += 1
        
        if self.path > 6:
            self.path = 1

        self.t += 1

        #print("Path    : ",self.path)
        #print("Time    : ",self.t)
        #print("Counter : ",self.counter)

        if self.inside_arena([ref_x, ref_y])==True:
            #if target is inside arena
            if self.path < 4:
                self.translate(ref_x, ref_y, self.yaw0 + 3*self.path)
            else:
                self.translate(ref_x, ref_y, self.yaw0 - 3*self.path)


    def inside_arena(self, pos):
        #check whether pos is inside arena, assuming origin 0,0 in middle
        #border [x_min, x_max, y_min, y_max]
        borders=[-1, 1, -1, 1]
        if pos[0]<borders[0] or pos[0]>borders[1] or pos[1]<borders[2] or pos[1]>borders[3]:
            return False
        return True


    def translate(self, x_target, y_target, angle):

        x_error=(x_target-self.x0)*math.cos(self.yaw0)+(y_target-self.y0)*math.sin(self.yaw0)
        y_error=-(x_target-self.x0)*math.sin(self.yaw0)+(y_target-self.y0)*math.cos(self.yaw0)
        ang_error=math.atan2(math.sin(angle-self.yaw0), math.cos(angle-self.yaw0))


        x_derivative = (x_error - self.pre_x_error) / self.del_T
        y_derivative = (y_error - self.pre_y_error) / self.del_T
        ang_derivative = (ang_error - self.pre_ang_error) / self.del_T

        # integrals (PID)
        self.x_integral += x_error * self.del_T
        if self.x_integral > self.lin_integral_threshold:
            self.x_integral = self.lin_integral_threshold
        elif self.x_integral < -self.lin_integral_threshold:
            self.x_integral = -self.lin_integral_threshold

        self.y_integral += y_error * self.del_T
        if self.y_integral > self.lin_integral_threshold:
            self.y_integral = self.lin_integral_threshold
        elif self.y_integral < -self.lin_integral_threshold:
            self.y_integral = -self.lin_integral_threshold

        self.ang_integral += ang_error * self.del_T
        if self.ang_integral > self.ang_integral_threshold:
            self.ang_integral = self.ang_integral_threshold
        elif self.ang_integral < -self.ang_integral_threshold:
            self.ang_integral = -self.ang_integral_threshold
        
        # output velocities
        x_linear_vel = (self.p_x * x_error) + (self.d_x * x_derivative) + (self.i_x * self.x_integral)
        if x_linear_vel > self.lin_vel_thres:
            x_linear_vel = self.lin_vel_thres
        elif x_linear_vel < -self.lin_vel_thres:
            x_linear_vel = -self.lin_vel_thres

        if abs(x_linear_vel)>450:
            x_linear_vel=x_linear_vel*450/abs(x_linear_vel)


        self.cmd_x = self.bias + x_linear_vel

        y_linear_vel = (self.p_y * y_error) + (self.d_y * y_derivative) + (self.i_y * self.y_integral)
        if y_linear_vel > self.lin_vel_thres:
            y_linear_vel = self.lin_vel_thres
        elif y_linear_vel < -self.lin_vel_thres:
            y_linear_vel = -self.lin_vel_thres


        if abs(y_linear_vel)>220:
            y_linear_vel=y_linear_vel*220/abs(y_linear_vel)

        self.cmd_y = self.bias - y_linear_vel

        angular_vel = (self.p_ang * ang_error) + (self.d_ang * ang_derivative) + (self.i_ang * self.ang_integral)
        if angular_vel > self.ang_vel_thres:
            angular_vel = self.ang_vel_thres
        elif angular_vel < -self.ang_vel_thres:
            angular_vel = -self.ang_vel_thres

        self.cmd_yaw = self.bias - angular_vel


        self.pre_x_error = x_error
        self.pre_y_error = y_error
        self.pre_ang_error = ang_error


    def rotate(self, angle):

        ang_error=math.atan2(math.sin(angle-self.yaw0), math.cos(angle-self.yaw0))
        derivative = (ang_error - self.pre_ang_error) / self.del_T
        self.ang_integral += ang_error * self.del_T
        if self.ang_integral > self.ang_integral_threshold:
            self.ang_integral = self.ang_integral_threshold
        elif self.ang_integral < -self.ang_integral_threshold:
            self.ang_integral = -self.ang_integral_threshold
        angular_vel = (self.p_ang * ang_error) + (self.d_ang * derivative) + (self.i_ang * self.ang_integral)


        if angular_vel > self.ang_vel_thres:
            angular_vel = self.ang_vel_thres
        elif angular_vel < -self.ang_vel_thres:
            angular_vel = -self.ang_vel_thres


        self.cmd_x=self.bias
        self.cmd_y=self.bias
        self.cmd_yaw = int(self.bias - angular_vel)


        self.pre_ang_error = ang_error



    def enemy_callback(self, msg):

        self.clustered_enemy_pos=[]
        for point in msg.points:
            self.clustered_enemy_pos.append([point.x, point.y])
    


    def armor_callback(self, msg):

        state_x = msg.x
        state_y = msg.y

        error_x = self.target_x - state_x
        output_x = self.Kp*error_x + self.Ki*(error_x+self.errorp_x) + self.Kd*(error_x-self.errorp_x)
        self.cmd_yaw_turret = self.bias - output_x
    
        error_y = state_y - self.target_y
        output_y = self.Kp*error_y + self.Ki*(error_y+self.errorp_y) + self.Kd*(error_y-self.errorp_y)
        self.cmd_pitch = self.bias - output_y
    
        self.errorp_x = error_x
        self.errorp_y = error_y

        if abs(error_x) < 100 and abs(error_y) < 100:
            self.cmd_shoot = 1
        else:
            self.cmd_shoot = 0

        self.updatetime=time()



    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True

    def print_marker(self, mark):
        self.markers.points=list()
        if mark is None:
            return

        for x in mark:
            #markerList store points wrt 2D world coordinate
            
            p=Point()

            p.x=x[0]
            p.y=x[1]
            p.z=0

            self.markers.points.append(p)
        self.markers_pub.publish(self.markers)

    def initMarker(self):
        # Set up our waypoint markers
        print("initializing markers")
        marker_scale = 0.2
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'markers'
        marker_id = 0
        marker_color = {'r': 0.7, 'g': 0.5, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        self.markers_pub = rospy.Publisher('clustered_enemy', Marker, queue_size=5)

        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        # self.markers.type = Marker.ARROW
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.scale.z = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()
  

if __name__ == '__main__':
    try:
        BaseDodge(nodename="base_dodging")
    except rospy.ROSInterruptException:
        rospy.loginfo("dodging exit.")
