#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import math
import cv2


from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo, LaserScan, CompressedImage
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from sensor_msgs.msg import RegionOfInterest, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from sklearn.cluster import DBSCAN, KMeans
from sklearn import metrics
from sklearn.metrics.pairwise import euclidean_distances, manhattan_distances
from sklearn.preprocessing import StandardScaler, normalize
from collections import Counter


class CameraMasking(object):
    x0, y0, yaw0= 0, 0, 0
    enemy_pos=[]
    clustered_enemy_pos=[]

    start=[]
    end=[]

    cam_x_offset=0 #offset from lidar

    def __init__(self):
        
        print("starting laser cam masking")
        rospy.init_node('camera_mask', anonymous=True)
        #publish enemy clustered over time
        #self.initMarker()
        self.image_pub =rospy.Publisher("/image_rect_color_masked", Image, queue_size=10)
        
        rospy.Subscriber("/odometry", Odometry, self.odom_callback, queue_size = 50)

        rospy.Subscriber("/image_rect_color", Image, self.img_callback, queue_size = 50)
        rospy.Subscriber("/enemy_yolo", Marker, self.enemy_callback, queue_size = 50)


        while not rospy.is_shutdown():
            
            rospy.sleep(1)


    def img_callback(self, msg):
        width=1024
        fov=74*math.pi/180

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.img_shape=image_np.shape

        #print(image_np.shape)
        xsize, ysize=image_np.shape[0], image_np.shape[1]

        #region_select=cv2.flip(image_np,  1)
        mask=np.zeros_like(image_np)

        XX, YY = np.meshgrid(np.arange(0, xsize), np.arange(0, ysize))

        rad2px=width/fov #image width=1024, field of view=78 deg
        
        for i in range(len(self.clustered_enemy_pos)):
            enemy_heading=math.atan2(self.clustered_enemy_pos[i][1]-self.y0-self.cam_x_offset*math.sin(self.yaw0), self.clustered_enemy_pos[i][0]-self.x0-self.cam_x_offset*math.cos(self.yaw0))-self.yaw0
            enemy_heading=math.atan2(math.sin(enemy_heading), math.cos(enemy_heading))
            enemy_mid=int((fov/2-enemy_heading)*rad2px) #position of enemy in pixels
            #print(enemy_mid)   
            if enemy_mid>width+100 or enemy_mid<-100:
                continue

            start=int(enemy_mid-width/4)
            end=int(enemy_mid+width/4)
            if start<0:
                start=0
            if end>width:
                end=width

            mask[:, start:end, :]=[1, 1, 1]

        region_select=np.multiply(image_np, mask)
            

        #cv2.imshow("Image window", region_select)
        #cv2.waitKey(3)
        #region_select=cv2.flip(region_select,  1)
        #### Create Image ####
        new_img = Image()
        new_img =CvBridge().cv2_to_imgmsg(region_select, "rgb8")
        # Publish new image
        self.image_pub.publish(new_img)


    def enemy_callback(self, msg):
        self.clustered_enemy_pos=[]
        for point in msg.points:
            self.clustered_enemy_pos.append([point.x, point.y])
        # below is time-based clustering
        # #size of stash
        # n_stash=20
        # #for a detected edge, add it into the list. If list is full, replace the first element.
        # for point in msg.points:

        #     if len(self.enemy_pos)==n_stash:
        #         #remove the first element
        #         del self.enemy_pos[0]
        #     self.enemy_pos.append([point.x, point.y])
        
        # #perform clustering to enemy
        # X=np.asarray(self.enemy_pos)
        # db = DBSCAN(eps=0.03, min_samples=5).fit(X)
        
        # core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        # core_samples_mask[db.core_sample_indices_] = True
        # labels = db.labels_
        
        # n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        # #print(n_clusters_)
        # if n_clusters_==0:
        #     return

        # clusters = [X[labels == i] for i in range(n_clusters_)]
        # self.clustered_enemy_pos=[]
        # #print(len(clusters))

        # for i in range(len(clusters)):
        #     position_kmeans=KMeans(n_clusters=1).fit(clusters[i])
        #     position_centers=position_kmeans.cluster_centers_[0]
        #     #print(position_centers)
        #     self.clustered_enemy_pos.append(position_centers)

        #self.print_marker(self.clustered_enemy_pos)
        #print(self.clustered_enemy_pos)

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
        marker_color = {'r': 0.2, 'g': 1.0, 'b': 0.2, 'a': 1.0}

        # Define a marker publisher.
        self.markers_pub = rospy.Publisher('/enemy', Marker, queue_size=5)

        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        # self.markers.type = Marker.ARROW
        self.markers.type = Marker.SPHERE_LIST
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
        camMask=CameraMasking()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished")