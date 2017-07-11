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
from sensor_msgs.msg import RegionOfInterest, CameraInfo, LaserScan, Joy, Image
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

    p_y = 850.0 #600
    i_y = 0.1 #0.1
    d_y = 120.0 #120, 180


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
    state_x = 0.0
    state_y = 0.0
    statep_x = state_x
    statep_y = state_y
    stash = []
    prevsh = 0
    updatetime = time()

    #camera parameters
    # xMax = rospy.get_param('/usb_cam/image_width') / 10.0
    # yMax = rospy.get_param('/usb_cam/image_height') / 10.0
    xMax = 1024.0/10.0
    yMax = 576.0/10.0
    error_x = xMax/2.0

    #heatmap to publish
    img = Image()
    img.header.frame_id = '/heatmap'
    img.height = int(xMax)
    img.width = int(yMax)
    img.encoding = 'mono8'
    img.step = int(yMax)


    Kp_x = 1.3
    Ki_x = 0
    Kd_x = 1.5

    Kp_y = 1.5
    Ki_y = 0
    Kd_y = 0.8

    target_x = xMax/2.0
    target_y = yMax/2.0
    
    errorp_x = 0
    errorp_y = 0

    #cmd vel params
    cmd_x=bias
    cmd_y=bias
    cmd_yaw=bias
    cmd_yaw_turret=bias
    cmd_pitch=bias
    cmd_shoot=0

    startshoottime=0

    # path_marker params
    path_marker=[]
    path_marker_done=[]

    # ref_marker
    ref_marker = Marker()


    def __init__(self, nodename):
        rospy.init_node(nodename, anonymous=False)
        print self.d_ang
        rospy.Subscriber("/odometry", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/enemy_yolo", Marker, self.enemy_callback, queue_size = 50)
        rospy.Subscriber('/roi', RegionOfInterest, self.armor_callback, queue_size = 50)
        self.cmd_vel_pub=rospy.Publisher("/cmd_vel", Joy, queue_size=10)
        self.pubimg = rospy.Publisher('/heatmap', Image, queue_size=1)
        self.pubpath_marker = rospy.Publisher('path_marker', Marker, queue_size=5)
        self.pubref_marker = rospy.Publisher('ref_marker', Marker, queue_size=5)



        # path_marker
        for i in range(6):
            self.path_marker_done.append(False)
            self.path_marker.append(Marker())
            self.path_marker[i].header.stamp = rospy.get_rostime();
            self.path_marker[i].header.frame_id = "odom";
            self.path_marker[i].ns = "points";
            self.path_marker[i].type = self.path_marker[i].POINTS
            self.path_marker[i].action = self.path_marker[i].ADD
            self.path_marker[i].pose.orientation.w = 1.0
            self.path_marker[i].id = 0
            self.path_marker[i].scale.x = 0.02
            self.path_marker[i].scale.y = 0.02
            self.path_marker[i].scale.z = 0.02
            self.path_marker[i].color.g = 1.0
            self.path_marker[i].color.b = 1.0
            self.path_marker[i].color.a = 1.0

        #  ref_marker
            self.ref_marker.header.stamp = rospy.get_rostime();
            self.ref_marker.header.frame_id = "odom";
            self.ref_marker.ns = "points";
            self.ref_marker.type = self.ref_marker.POINTS
            self.ref_marker.action = self.ref_marker.ADD
            self.ref_marker.pose.orientation.w = 1.0
            self.ref_marker.id = 0
            self.ref_marker.scale.x = 0.02
            self.ref_marker.scale.y = 0.02
            self.ref_marker.scale.z = 0.02
            self.ref_marker.color.r = 1.0
            self.ref_marker.color.a = 1.0

        self.pitch_up()

        rate=rospy.Rate(10)
        msg=Joy()


        heading_threshold=20*math.pi/180
        while not rospy.is_shutdown():
            #self.translate(0, 0, 0)
            #self.passive_dodge()

            # if abs(self.yaw0-0)>heading_threshold:
            #     self.rotate(0)
            # else:
            #     #else translate to goal    
            #     self.translate(0, 0, 0)
            if len(self.enemy_pos)==1:
                 #if only one, active dodging 
                # self.active_dodge()    
                self.passive_dodge()      
            else:

                #more than one, passive dodge
                self.passive_dodge()
            
            # msg=Joy()
            
            #do priority that governs yawing (lidar or vision)
            if time() - self.updatetime < 1 and ((self.state_x - self.statep_x < self.xMax/5 and self.state_y - self.statep_y < self.yMax/2) or not self.stash) and abs(self.error_x)<self.xMax/4:
                # print("shoot")
                #update time is only updated in shooting mode. armor detected
                if self.cmd_shoot==1 and time() - self.startshoottime < 3:
                    self.cmd_x = self.bias
                    self.cmd_y = self.bias

                # if time()-self.startshoottime>3 and time()-self.startshoottime <4:
                #     self.cmd_pitch=1524 #pitch up

                msg.buttons=[int(self.cmd_x), int(self.cmd_y), int(self.cmd_yaw_turret), int(self.cmd_pitch), int(self.cmd_shoot)]

            else:
                
                #no shooting, dodge and reset turret pid
                if time() - self.updatetime > 2:
                    self.stash = []

                heatmap = np.zeros((int(self.xMax),int(self.yMax)), dtype=np.uint8)
                self.img.data=np.resize(heatmap, int(self.xMax)*int(self.yMax)).astype(np.uint8).tolist()
                self.pubimg.publish(self.img)
                self.state_x=0
                self.state_y=0
                self.cmd_shoot=0
                self.cmd_pitch=self.bias
                msg.buttons=[int(self.cmd_x), int(self.cmd_y), int(self.cmd_yaw), int(self.cmd_pitch), int(self.cmd_shoot)]
            #msg.buttons=[int(self.cmd_x), int(self.cmd_y), int(self.cmd_yaw), int(self.cmd_pitch), int(self.cmd_shoot)]
            #print(msg)
            self.cmd_vel_pub.publish(msg)

            rate.sleep()

        self.stop()
    def generate_path(self):
        pass

    def pitch_up(self):
        msg=Joy()
        #pitch up to ensure armor in field of view
        self.cmd_pitch=1524
        msg.buttons=[self.bias, self.bias, self.bias, int(self.cmd_pitch), 0]


    def stop(self):
        msg=Joy()
        msg.buttons = [self.bias, self.bias, self.bias, self.bias, 0]
        self.cmd_vel_pub.publish(msg)

    def active_dodge(self):
        
        target=self.enemy_pos[0]

        #rotate to face target
        heading=math.atan2(target[1]-self.y0, target[0]-self.x0)
        heading_threshold=40*math.pi/180

        difference=abs(math.atan2(math.sin(self.yaw0-heading), math.cos(self.yaw0-heading)))

        if difference>heading_threshold:

            #print("rotate")
            print(math.atan2(math.sin(self.yaw0-heading), math.cos(self.yaw0-heading))*180/math.pi)
            self.rotate(heading)
        else:
            
            #print("translate")
            #the higher d, the faster 
            d=0.3
            
            #direction to the left
            beta1=heading+math.pi/2
            #direction to the right
            beta2=heading-math.pi/2

            #check if out of radius, assume middle of the arena is origin
            if math.sqrt(self.x0**2+self.y0**2)>0.7:
                #add to origin vector
                delta=math.atan2(-self.y0, -self.x0)
                #print(delta*180/math.pi)
                pred1=[self.x0+d*math.cos(beta1)+0.2*math.cos(delta), self.y0+d*math.sin(beta1)+0.2*math.sin(delta)]
                pred2=[self.x0+d*math.cos(beta2)+0.2*math.cos(delta), self.y0+d*math.sin(beta2)+0.2*math.sin(delta)]
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
            # ref_x = self.x_plot(self.t,0,0.5,0.25)
            # ref_y = self.y_plot(self.t,0,0.5,0.25)
            ref_x = self.x_plot(self.t,0,0.4,0.25)
            ref_y = self.y_plot(self.t,0,0.4,0.25)
            if self.path_marker_done[self.path-1] == False:
                points = []
                for i in range(0,1000):
                    p = Point()
                    # p.x = self.x_plot(i,0,0.5,0.25)
                    # p.y = self.y_plot(i,0,0.5,0.25)
                    p.x = self.x_plot(i,0,0.4,0.25)
                    p.y = self.y_plot(i,0,0.4,0.25)
                    p.z = 0.0
                    points.append(p)
                self.path_marker[self.path-1].points = points
                self.path_marker_done[self.path-1] = True


        elif self.path == 2:
            # ref_x = self.x_plot(self.t,-26,-0.45,-0.4)
            # ref_y = self.y_plot(self.t,26,0.45,0.4)
            ref_x = self.x_plot(self.t,-26,-0.4,-0.4)
            ref_y = self.y_plot(self.t,26,0.4,0.4)
            if self.path_marker_done[self.path-1] == False:
                points = []
                for i in range(0,1000):
                    p = Point()
                    # p.x = self.x_plot(i,-26,-0.45,-0.4)
                    # p.y = self.y_plot(i,26,0.45,0.4)
                    p.x = self.x_plot(i,-26,-0.4,-0.4)
                    p.y = self.y_plot(i,26,0.4,0.4)
                    p.z = 0.0
                    points.append(p)
                self.path_marker[self.path-1].points = points
                self.path_marker_done[self.path-1] = True


        elif self.path == 3:
            # ref_x = self.x_plot(self.t,-26,0.45,0.4)
            # ref_y = self.y_plot(self.t,-26,0.45,0.4)
            ref_x = self.x_plot(self.t,-26,0.4,0.4)
            ref_y = self.y_plot(self.t,-26,0.4,0.4)
            if self.path_marker_done[self.path-1] == False:
                points = []
                for i in range(0,1000):
                    p = Point()
                    # p.x = self.x_plot(i,-26,0.45,0.4)
                    # p.y = self.y_plot(i,-26,0.45,0.4)
                    p.x = self.x_plot(i,-26,0.4,0.4)
                    p.y = self.y_plot(i,-26,0.4,0.4)
                    p.z = 0.0
                    points.append(p)
                self.path_marker[self.path-1].points = points
                self.path_marker_done[self.path-1] = True



        elif self.path == 4:
            # ref_x = self.x_plot(self.t,0,-0.5,-0.25)
            # ref_y = self.y_plot(self.t,0,0.5,0.25)
            ref_x = self.x_plot(self.t,0,-0.4,-0.25)
            ref_y = self.y_plot(self.t,0,0.4,0.25)
            if self.path_marker_done[self.path-1] == False:
                points = []
                for i in range(0,1000):
                    p = Point()
                    # p.x = self.x_plot(i,0,-0.5,-0.25)
                    # p.y = self.y_plot(i,0,0.5,0.25)
                    p.x = self.x_plot(i,0,-0.4,-0.25)
                    p.y = self.y_plot(i,0,0.4,0.25)
                    p.z = 0.0
                    points.append(p)
                self.path_marker[self.path-1].points = points
                self.path_marker_done[self.path-1] = True


        elif self.path == 5:
            # ref_x = self.x_plot(self.t,26,0.45,0.4)
            # ref_y = self.y_plot(self.t,26,0.45,0.4)
            ref_x = self.x_plot(self.t,26,0.4,0.4)
            ref_y = self.y_plot(self.t,26,0.4,0.4)
            if self.path_marker_done[self.path-1] == False:
                points = []
                for i in range(0,1000):
                    p = Point()
                    # p.x = self.x_plot(i,26,0.45,0.4)
                    # p.y = self.y_plot(i,26,0.45,0.4)
                    p.x = self.x_plot(i,26,0.4,0.4)
                    p.y = self.y_plot(i,26,0.4,0.4)
                    p.z = 0.0
                    points.append(p)
                self.path_marker[self.path-1].points = points
                self.path_marker_done[self.path-1] = True


        elif self.path == 6:
            # ref_x = self.x_plot(self.t,26,-0.45,-0.4)
            # ref_y = self.y_plot(self.t,-26,0.45,0.4)
            ref_x = self.x_plot(self.t,26,-0.4,-0.4)
            ref_y = self.y_plot(self.t,-26,0.4,0.4)
            if self.path_marker_done[self.path-1] == False:
                points = []
                for i in range(0,1000):
                    p = Point()
                    # p.x = self.x_plot(i,26,-0.45,-0.4)
                    # p.y = self.y_plot(i,-26,0.45,0.4)
                    p.x = self.x_plot(i,26,-0.4,-0.4)
                    p.y = self.y_plot(i,-26,0.4,0.4)
                    p.z = 0.0
                    points.append(p)
                self.path_marker[self.path-1].points = points
                self.path_marker_done[self.path-1] = True

        self.pubpath_marker.publish(self.path_marker[self.path-1])
        points = Point()
        points.x = ref_x
        points.y = ref_y
        self.ref_marker.points = [points]
        self.pubref_marker.publish(self.ref_marker)


 
        if self.t > self.counter*35:
            self.path += 1
            self.counter += 1
        
        if self.path > 6:
            self.path = 1

        self.t += 1

        #print("Path    : ",self.path)
        #print("Time    : ",self.t)
        #print("Counter : ",self.counter)

        if abs(self.x0) > 0.5 or abs(self.y0) > 0.5:
            print("return")
            self.translate(0, 0, self.yaw0)
        else:
            if self.inside_arena([ref_x, ref_y])==True:
                #if target is inside arena
                if self.path < 4:
                    # self.translate(ref_x, ref_y, self.yaw0 + self.path*20*math.pi/180)
                    self.translate(ref_x, ref_y, self.yaw0 + self.path*10*math.pi/180)
                else:
                    # self.translate(ref_x, ref_y, self.yaw0 - self.path*20*math.pi/180)
                    self.translate(ref_x, ref_y, self.yaw0 - self.path*10*math.pi/180)

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

        #calculate center and size of roi
        roi = [0.0, 0.0, 0.0] #center_x, center_y, size
        roi[0] = (msg.x_offset + msg.width/2) / 10.0
        roi[1] = (msg.y_offset + msg.height/2) / 10.0
        roi[2] = msg.width * msg.height / 10.0

        if len(self.stash)==5:
            del self.stash[0]
        self.stash.append(roi)
        
        heatmap = np.zeros((int(self.xMax),int(self.yMax)), dtype=np.uint8)
        for obj in self.stash:
            heatmap[int(obj[0]), int(obj[1])] += 0.1*obj[2]

        self.img.data=np.resize(heatmap, int(self.xMax)*int(self.yMax)).astype(np.uint8).tolist()
        self.pubimg.publish(self.img)
        
        self.statep_x = self.state_x
        self.statep_y = self.state_y
        self.state_x, self.state_y = np.unravel_index(heatmap.argmax(), heatmap.shape)

        self.error_x = self.target_x - self.state_x
        output_x = self.Kp_x*self.error_x + self.Ki_x*(self.error_x+self.errorp_x) + self.Kd_x*(self.error_x-self.errorp_x)
        self.cmd_yaw_turret = self.bias - output_x
    
        error_y = self.state_y - self.target_y
        output_y = self.Kp_y*error_y + self.Ki_y*(error_y+self.errorp_y) + self.Kd_y*(error_y-self.errorp_y)
        self.cmd_pitch = self.bias - output_y
    
        self.errorp_x = self.error_x
        self.errorp_y = error_y

        if abs(self.error_x) < self.xMax/10 and abs(error_y) < self.yMax/6:
            self.cmd_shoot = 1
            if self.prevsh == 0:
                self.startshoottime = time()
            self.prevsh = 1
        else:
            self.cmd_shoot = 0
            self.prevsh = 0

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
