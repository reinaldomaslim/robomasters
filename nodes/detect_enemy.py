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
    box_length=0.2

    def __init__(self, nodename):
        rospy.init_node(nodename, anonymous=False)
    
        self.initMarker()

        rospy.Subscriber("/odometry", Odometry, self.odom_callback, queue_size = 50)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 50)
        self.enemy_pose_pub=rospy.Publisher("/enemy", PoseStamped, queue_size=10)

        rate=rospy.Rate(10)
    
        while not rospy.is_shutdown():


            rate.sleep()

    def scan_callback(self, msg):
        window_length=3
        resolution=0.02
        size=window_length/resolution
        mid_point=int(size/2)

        grid=[]

        for i in range(len(msg.ranges)):
            if msg.ranges[i]<window_length/2:
                theta=i*msg.angle_increment#-math.pi/4
                d=msg.ranges[i]
                x=d*math.cos(theta)
                y=d*math.sin(theta)
                grid.append([x, y])
                
        #print(grid)
        self.enemy_position=self.detect_enemy(grid, resolution, window_length/2)
        #print(self.enemy_position)

        #self.print_marker([[0, 1]])
        self.print_marker(self.enemy_position)

    def detect_enemy(self, grid, resolution, origin):
        msg=PoseStamped()
        #origin=int(grid.shape[0]/2)
        #X=normalize(grid)
        
        #this line takes longest time to run 
        #D = manhattan_distances(frontiers_array, frontiers_array)
        X = StandardScaler().fit_transform(grid)
        print(X)
        db = DBSCAN(eps=0.5, min_samples=10).fit(X)
        #db = DBSCAN(eps=0.5, min_samples=5, algorithm='ball_tree', metric='haversine').fit(X)
        
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

            pos_x=-center[0][0]
            pos_y=center[0][1]
            cluster_centers.append([pos_x, pos_y])


            msg.header.frame_id="odom"
            direction=math.atan2(self.y0-pos_y, self.x0-pos_x)
            msg.pose.position.x = pos_x
            msg.pose.position.y = pos_y
            q_angle = quaternion_from_euler(0, 0, direction)
            msg.pose.orientation = Quaternion(*q_angle)
            self.enemy_pose_pub.publish(msg)


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
        self.markers_pub = rospy.Publisher('markers_marker', Marker, queue_size=5)

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
