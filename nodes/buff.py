#!/usr/bin/env python

## PID control for yaw and pitch of turret towards target from target input from webcam
## Subsribe to /yolo2_detections topic to obtain position data
## Publish angular velocity of turret to arduino through /cmd_vel topic

## Small buff autoshooting 

import rospy
import numpy as np
from time import time
from sensor_msgs.msg import Joy, Image
from yolo2.msg import Detection, ImageDetections

state_x = 0.0
state_y = 0.0
stash = []
updatetime = time()
im_size=[640/10.0, 480/10.0]

#heatmap to publish
pubimg = rospy.Publisher('/heatmap', Image, queue_size=1)
img = Image()
img.header.frame_id = '/heatmap'
img.height = int(im_size[1])
img.width = int(im_size[0])
img.encoding = 'mono8'
img.step = int(im_size[0])

def callback(msg):

    if len(msg.detections)==0:
	return

    global updatetime
    updatetime = time()

    frame = np.zeros((int(im_size[0]),int(im_size[1])), dtype=np.float32)

    for detection in msg.detections:
        x1 = ( detection.x - detection.width/2 ) * im_size[0]
        y1 = ( detection.y - detection.height/2) * im_size[1]
        x2 = ( detection.x + detection.width/2 ) * im_size[0]
        y2 = ( detection.y + detection.height/2) * im_size[1]
        frame[int(x1):int(x2), int(y1):int(y2)] += detection.confidence*50

    global stash
    if len(stash)==3:
        del stash[0]
    stash.append(frame)
    heatmap = np.sum(np.asarray(stash), axis=0)
    
    global img, pubimg
    img.data=np.resize(np.ravel(heatmap.T, order='C'), int(im_size[0])*int(im_size[1])).astype(np.uint8).tolist()
    pubimg.publish(img)

    global state_x, state_y
    first = np.unravel_index(heatmap.argmax(), heatmap.shape)
    last =  np.unravel_index(heatmap.size-np.argmax(np.resize(heatmap,heatmap.size)[::-1])-1, heatmap.shape)
    state_x = (first[0]+last[0])/2
    state_y = (first[1]+last[1])/2
    #print "center = ", state_x, state_y

def turret():

    rospy.init_node('small_buff', anonymous=True)
    
    global state_x, state_y, updatetime
    pub = rospy.Publisher('/cmd_vel', Joy, queue_size=10)
    rospy.Subscriber('/yolo2/detections', ImageDetections, callback)
    rate = rospy.Rate(10) # 10Hz

    bias = 1024

    Kp_x = 3.8
    Ki_x = 0
    Kd_x = 2.1

    Kp_y = 1.5
    Ki_y = 0
    Kd_y = 0.5

    target_x = im_size[0]/2.0
    target_y = im_size[1]/2.0
    
    errorp_x = 0.0
    errorp_y = 0.0

    while not rospy.is_shutdown():
        if time() - updatetime < 0.5:
            error_x = target_x - state_x
            output_x = Kp_x*error_x + Ki_x*(error_x+errorp_x) + Kd_x*(error_x-errorp_x)
            output_x = bias - output_x
        
            error_y = state_y - target_y
            output_y = Kp_y*error_y + Ki_y*(error_y+errorp_y) + Kd_y*(error_y-errorp_y)
            output_y = bias - output_y
        
            errorp_x = error_x
            errorp_y = error_y

            if abs(error_x) < im_size[0]/10 and abs(error_y) < im_size[1]/6:
                shoot = 1
            else:
                shoot = 0

        else:
            global stash
            stash = []
            heatmap = np.zeros((int(im_size[0]),int(im_size[1])), dtype=np.uint8)
            global img, pubimg
            img.data=np.resize(np.ravel(heatmap.T, order='C'), int(im_size[0])*int(im_size[1])).astype(np.uint8).tolist()
            pubimg.publish(img)
            state_x = 0.0
            state_y = 0.0
            output_x = bias
            output_y = bias
            shoot = 0
            
        output = Joy()
        output.buttons = [output_x, output_y, shoot]

        # rospy.loginfo("x_pos = %d", state_x)
        # rospy.loginfo("y_pos = %d", state_y)
        # rospy.loginfo("yaw = %d", output.buttons[2])
        # rospy.loginfo("pitch = %d", output.buttons[3])
        # rospy.loginfo("shoot = %d", output.buttons[4])

        pub.publish(output)
        rate.sleep()

if __name__ == '__main__':
    try:
        turret()
    except rospy.ROSInterruptException:
        pass
