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

class MissionPlanner(object):
    x0, y0, yaw0= 0, 0, 0
    enemy_pos=[]

    ## Movement PID constants
    del_T = 100.0   # time step in ms
    
    p_ang =140.0 #140, 140
    i_ang = .02 #0.013, 0.01
    d_ang = 55000.0 #50000, 56000

    p_x = 450.0 #350
    i_x = 0.1 #0.2
    d_x = 100.0 #200, 300

    p_y = 550.0 #600
    i_y = 0.1 #0.1
    d_y = 100.0 #120, 180


    x_lin_vel_thres = 660.0 # max 660
    y_lin_vel_thres=660
    ang_vel_thres = 660.0 # max 660
    bias = 1024.0
    pre_ang_error = 0.0
    pre_x_error = 0.0
    pre_y_error = 0.0
    ang_integral = 0.0
    x_integral = 0.0
    y_integral = 0.0
    lin_integral_threshold = 50.0
    ang_integral_threshold = 20.0

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
    updatetime = time()
    stash = []
    #xMax = rospy.get_param('/usb_cam/image_width') / 10
    #yMax = rospy.get_param('/usb_cam/image_height') / 10

    # needs tuning again
    Kp = 1.6
    Ki = 0.001
    Kd = 0.8

    #target_x = xMax/2
    #target_y = yMax/2
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
        print self.d_ang
        rospy.Subscriber("/odometry", Odometry, self.odom_callback, queue_size = 50)
        #rospy.Subscriber("/enemy_yolo", Marker, self.enemy_callback, queue_size = 50)
        #rospy.Subscriber('/roi', RegionOfInterest, self.armor_callback, queue_size = 50)
        self.cmd_vel_pub=rospy.Publisher("/cmd_vel", Joy, queue_size=10)

        rate=rospy.Rate(10)
        msg=Joy()

        heading_threshold=20*math.pi/180
        while not rospy.is_shutdown():
            self.translate(0, 0, 0)
            #self.passive_dodge()

            # if abs(self.yaw0-0)>heading_threshold:
            #     self.rotate(0)
            # else:
            #     #else translate to goal    
            #     self.translate(0, 0, 0)
            # if len(self.enemy_pos)==1:
            #     #if only one, active dodging 
            #     self.active_dodge()      
            # else:
            #     #more than one, passive dodge
            #     continue
            
            # msg=Joy()
            
            # #do priority that governs yawing (lidar or vision)
            # if time() - self.updatetime < 3:
            #     #update time is only updated in shooting mode. armor detected
            #     msg.buttons=[self.cmd_x, self.cmd_y, self.cmd_yaw_turret, self.cmd_pitch, self.cmd_shoot]

            # else:
            #     #no shooting, dodge and reset turret pid
            #     self.state_x=0
            #     self.state_y=0
            #     self.cmd_shoot=0
            #     self.cmd_pitch=self.bias
            #     msg.buttons=[self.cmd_x, self.cmd_y, self.cmd_yaw, self.cmd_pitch, self.cmd_shoot]
            msg.buttons=[int(self.cmd_x), int(self.cmd_y), int(self.cmd_yaw), int(self.cmd_pitch), int(self.cmd_shoot)]
            #msg.buttons=[int(self.cmd_x), 1024, int(self.cmd_yaw), int(self.cmd_pitch), int(self.cmd_shoot)]
            print(msg)
            self.cmd_vel_pub.publish(msg)

            rate.sleep()

        self.stop()


    def stop(self):
        msg=Joy()
        msg.buttons = [self.bias, self.bias, self.bias, self.bias, 0]
        self.cmd_vel_pub.publish(msg)

    def active_dodge(self):
        
        target=self.enemy_pos[0]

        #rotate to face target
        heading=math.atan2(target[1]-self.y0, target[0]-self.x0)
        heading_threshold=25*math.pi/180

        difference=abs(math.atan2(math.sin(self.yaw0-heading), math.cos(self.yaw0-heading)))

        if difference>heading_threshold:

            #print("rotate")
            print(math.atan2(math.sin(self.yaw0-heading), math.cos(self.yaw0-heading))*180/math.pi)
            self.rotate(heading)
        else:
            
            #print("translate")
            #the higher d, the faster 
            d=0.4
            
            #direction to the left
            beta1=heading+math.pi/2
            #direction to the right
            beta2=heading-math.pi/2

            #check if out of radius, assume middle of the arena is origin
            if math.sqrt(self.x0**2+self.y0**2)>0.7:
                #add to origin vector
                delta=math.atan2(-self.y0, -self.x0)
                #print(delta*180/math.pi)
                pred1=[self.x0+d*math.cos(beta1)+0.1*math.cos(delta), self.y0+d*math.sin(beta1)+0.1*math.sin(delta)]
                pred2=[self.x0+d*math.cos(beta2)+0.1*math.cos(delta), self.y0+d*math.sin(beta2)+0.1*math.sin(delta)]
            else:
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
            ref_x = self.x_plot(self.t,0,0.5,0.25)
            ref_y = self.y_plot(self.t,0,0.5,0.25)


        elif self.path == 2:
            ref_x = self.x_plot(self.t,-26,-0.45,-0.5)
            ref_y = self.y_plot(self.t,26,0.45,0.5)


        elif self.path == 3:
            ref_x = self.x_plot(self.t,-26,0.45,0.5)
            ref_y = self.y_plot(self.t,-26,0.45,0.5)


        elif self.path == 4:
            ref_x = self.x_plot(self.t,0,-0.5,-0.25)
            ref_y = self.y_plot(self.t,0,0.5,0.25)


        elif self.path == 5:
            ref_x = self.x_plot(self.t,26,0.45,0.5)
            ref_y = self.y_plot(self.t,26,0.45,0.5)


        elif self.path == 6:
            ref_x = self.x_plot(self.t,26,-0.45,-0.5)
            ref_y = self.y_plot(self.t,-26,0.45,0.5)

 
        if self.t > self.counter*35:
            self.path += 1
            self.counter += 1
        
        if self.path > 6:
            self.path = 1

        self.t += 1

        #print("Path    : ",self.path)
        #print("Time    : ",self.t)
        #print("Counter : ",self.counter)

        if self.x0 > 0.5 or self.y0> 0.5:
            print("return")
            self.translate(0, 0, 0)
        else:
            if self.inside_arena([ref_x, ref_y])==True:
                #if target is inside arena
                if self.path < 4:
                    self.translate(ref_x, ref_y, self.yaw0 + 3*self.path)
                else:
                    self.translate(ref_x, ref_y, self.yaw0 - 3*self.path)

    def inside_arena(self, pos):
        #check whether pos is inside arena, assuming origin 0,0 in middle
        #border [x_min, x_max, y_min, y_max]
        borders=[-0.7, 0.7, -0.7, 0.7]
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

        if abs(x_linear_vel)>self.x_lin_vel_thres:
            x_linear_vel=x_linear_vel*self.x_lin_vel_thres/abs(x_linear_vel)

        self.cmd_x = self.bias + x_linear_vel

        y_linear_vel = (self.p_y * y_error) + (self.d_y * y_derivative) + (self.i_y * self.y_integral)

        if abs(y_linear_vel)>self.y_lin_vel_thres:
            y_linear_vel=y_linear_vel*self.y_lin_vel_thres/abs(y_linear_vel)

        self.cmd_y = self.bias - y_linear_vel

        print(self.d_ang *ang_derivative)
        print(self.p_ang * ang_error)

        angular_vel = (self.p_ang * ang_error) + (self.d_ang * ang_derivative) + (self.i_ang * self.ang_integral)


        if abs(angular_vel)>self.ang_vel_thres:
            angular_vel=angular_vel*self.ang_vel_thres/abs(angular_vel)


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

        print(self.d_ang * derivative)
        
        angular_vel = (self.p_ang * ang_error) + (self.d_ang * derivative) + (self.i_ang * self.ang_integral)


        if abs(angular_vel)>self.ang_vel_thres:
            angular_vel=angular_vel*self.ang_vel_thres/abs(angular_vel)

        self.cmd_x=self.bias
        self.cmd_y=self.bias
        self.cmd_yaw = int(self.bias - angular_vel)


        self.pre_ang_error = ang_error



    def enemy_callback(self, msg):

        self.enemy_pos=[]
        for point in msg.points:
            self.enemy_pos.append([point.x, point.y])
    


    def armor_callback(self, msg):

        center = [0, 0]
        center[0] = (msg.x_offset + msg.width/2) / 10
        center[1] = (msg.y_offset + msg.height/2) / 10

        if len(self.stash)==10:
            del self.stash[0]
        self.stash.append(center)

        heatmap = np.zeros((self.xMax,self.yMax), dtype=np.uint8)
        for ctr in self.stash:
            heatmap[ctr[0], ctr[1]] += 1
        
        state_x, state_y = np.unravel_index(heatmap.argmax(), heatmap.shape)

        error_x = self.target_x - state_x
        output_x = self.Kp*error_x + self.Ki*(error_x+self.errorp_x) + self.Kd*(error_x-self.errorp_x)
        self.cmd_yaw_turret = self.bias - output_x
    
        error_y = state_y - self.target_y
        output_y = self.Kp*error_y + self.Ki*(error_y+self.errorp_y) + self.Kd*(error_y-self.errorp_y)
        self.cmd_pitch = self.bias - output_y
    
        self.errorp_x = error_x
        self.errorp_y = error_y

        if abs(error_x) < xMax/10 and abs(error_y) < yMax/6:
            self.cmd_shoot = 1
        else:
            self.cmd_shoot = 0

        self.updatetime=time()



    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
  

if __name__ == '__main__':
    try:

        MissionPlanner(nodename="mission_planner")
    except rospy.ROSInterruptException:
        rospy.loginfo("base mission planner exit.")
