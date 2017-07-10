#!/usr/bin/env python

""" reinaldo
"""


import rospy
import actionlib
import numpy as np
import math
import cv2

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from sensor_msgs.msg import RegionOfInterest, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from sklearn.cluster import DBSCAN, KMeans
from sklearn import metrics
from sklearn.metrics.pairwise import euclidean_distances, manhattan_distances
from sklearn.preprocessing import StandardScaler, normalize
from collections import Counter

class FindEnemy(object):
    x0, y0, yaw0= 0, 0, 0
    currentScan=LaserScan()
    lookback=3 #lookback to the previous scans, if 0 means yolo
    radius=5 #radius for detections, rplidar max is 6m
    counter=0
    grid=[]

    x0, y0, yaw0=0,0, 0

    wall_x=-2.8
    wall_y=-2.8

    x_off=0.215
    #ignore detections on own body, units in angle


    def __init__(self, nodename):
        rospy.init_node(nodename, anonymous=False)
    
        self.initMarker()

        # Define a marker publisher.
        self.markers_pub = rospy.Publisher('/enemy_yolo', Marker, queue_size=5)

        rospy.Subscriber("/odometry", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 50)
        
        rate=rospy.Rate(10)
    
        while not rospy.is_shutdown():


            rate.sleep()

    def scan_callback(self, msg):

        if self.counter>self.lookback:
            self.counter=0
        if self.counter==0:
            #refresh
            self.grid=[]

        for i in range(len(msg.ranges)):

            if msg.ranges[i]<self.radius:

                theta=math.atan2(math.sin(i*msg.angle_increment+self.yaw0), math.cos(i*msg.angle_increment+self.yaw0))#-math.pi/2
                d=msg.ranges[i]
                x=-d*math.sin(theta)+self.x_off+self.x0
                y=d*math.cos(theta)+self.y0
                #print(x, y)
                self.grid.append([x, y])
                
        #print(grid)
        enemy_position=self.detect_enemy(self.grid)
        #print(self.enemy_position)
        self.print_marker(enemy_position)
        #self.print_marker([[0, 1]])
        

    def detect_enemy(self, grid):

        X = StandardScaler().fit_transform(grid)
        db = DBSCAN(eps=0.2, min_samples=4).fit(X)
        
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_
        # print(labels)
        # Number of clusters in labels, ignoring noise if present.
        self.n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        grid=np.asarray(grid)
        clusters = [grid[labels == i] for i in range(self.n_clusters_)]
        cluster_centers=[]
        #print(clusters)

        for cluster in clusters:
            kmeans=KMeans(n_clusters=1).fit(cluster)
            center=kmeans.cluster_centers_

            pos_x=center[0][0]
            pos_y=center[0][1]

            # if pos_x<self.wall_x or pos_y<self.wall_y:
            #     #these are false positives from walls
            #     continue

            cluster_centers.append([pos_x, pos_y])

        return cluster_centers


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

            if abs(p.x)>1.7 or abs(p.y)>1.5:
                continue

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

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def odom_callback(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        _, _, self.yaw0 = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.odom_received = True
  

if __name__ == '__main__':
    try:
        FindEnemy(nodename="find_enemy")
    except rospy.ROSInterruptException:
        rospy.loginfo("Find enemy exit.")
