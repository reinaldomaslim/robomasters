#!/usr/bin/env python  
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math



class TfBroadcaster(object):
    initialize_localizer=True

    #the cg of robot when firstly launched wrt to map frame
    cg_origin=[-0.210, 0.265]


    def __init__(self, nodename):
        rospy.init_node('tf_broadcaster')
        listener = tf.TransformListener()
        rospy.Subscriber("/localization", Twist, self.encoderCallback, queue_size = 10)
        #self.odom_pub=rospy.Publisher("/odometry", Odometry, queue_size=10)
        self.odom_pub=rospy.Publisher("/odometry", Odometry, queue_size=10)

        r = rospy.Rate(100)

        while not rospy.is_shutdown():

            br = tf.TransformBroadcaster()
            #laser tf
            br.sendTransform((0.0, 0.0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, math.pi),
                             rospy.Time.now(),
                             "laser",
                             "base_link")

            #localizer tf
            br.sendTransform((-0.048, -0.12, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "encoder",
                             "base_link")

            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "base_link",
                             "odom")

            #will be given by rplidar slam gmapping
            br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "odom",
                             "map")

            r.sleep()


    def encoderCallback(self, msg):
        #if it's the first time, memorize its initial readings
        br = tf.TransformBroadcaster()

        if self.initialize_localizer is True:
            self.x_off=msg.linear.x 
            self.y_off=msg.linear.y
            self.yaw_off=msg.angular.z
            self.initialize_localizer=False

        #for subsequent, modify the reading to compensate
        else:
            linear_x=msg.linear.x-self.x_off-self.cg_origin[0]
            linear_y=msg.linear.y-self.y_off-self.cg_origin[1]
            angular_z=msg.angular.z-self.yaw_off

            #broadcast odom tf
            #center-encoder
            del_x=0.048
            del_y=0.12
            #odom frame given by localizer, perform tranform first
            yaw_c=math.atan2(math.sin(angular_z*math.pi/180), math.cos(angular_z*math.pi/180))

            x_c=-linear_y+del_x*math.cos(yaw_c)-del_y*math.sin(yaw_c)-del_x
            y_c=linear_x+del_x*math.sin(yaw_c)+del_y*math.cos(yaw_c)-del_y


            br.sendTransform((x_c, y_c, 0),
                             tf.transformations.quaternion_from_euler(0, 0, angular_z*math.pi/180),
                             rospy.Time.now(),
                             "base_link",
                             "odom")
            
            #publish odometry
            odom=Odometry()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x=x_c
            odom.pose.pose.position.y=y_c
            q=Quaternion()
            q.x, q.y, q.z, q.w=tf.transformations.quaternion_from_euler(0, 0, angular_z*math.pi/180)
            odom.pose.pose.orientation=q
            self.odom_pub.publish(odom)





if __name__ == '__main__':
    try:
        TfBroadcaster(nodename="tf")
    except rospy.ROSInterruptException:
        rospy.loginfo("tf broadcaster exit.")

