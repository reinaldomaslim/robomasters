#!/usr/bin/env python

## PID control for yaw and pitch of turret towards target from target input from webcam
## Subsribe to /roi topic to obtain position data
## Publish angular velocity of turret to arduino through /cmd_vel topic

import rospy
import numpy as np
from time import time
from sensor_msgs.msg import Joy, RegionOfInterest, Image

#global variables
state_x = 0.0
state_y = 0.0
statep_x = state_x
statep_y = state_y
stash = []
updatetime = time()

#camera parameters
# xMax = rospy.get_param('/usb_cam/image_width') / 10.0
# yMax = rospy.get_param('/usb_cam/image_height') / 10.0
xMax = 1024.0/10.0
yMax = 576.0/10.0

#heatmap to publish
img = Image()
img.header.frame_id = '/heatmap'
img.height = int(yMax)
img.width = int(xMax)
img.encoding = 'mono8'
img.step = int(xMax)
pubimg = rospy.Publisher('/heatmap', Image, queue_size=1)

def callback(msg):
    global updatetime
    updatetime = time()

    #calculate center and size of roi
    roi = [0.0, 0.0, 0.0] #center_x, center_y, size
    roi[0] = (msg.x_offset + msg.width/2) / 10.0
    roi[1] = (msg.y_offset + msg.height/2) / 10.0
    roi[2] = msg.width * msg.height / 10.0

    global stash
    if len(stash)==5:
            del stash[0]
    stash.append(roi)
    
    heatmap = np.zeros((int(xMax),int(yMax)), dtype=np.uint8)
    for obj in stash:
        heatmap[int(obj[0]), int(obj[1])] += 0.1*obj[2]

    global img, pubimg
    img.data=np.resize(np.ravel(heatmap.T, order='C'), int(xMax)*int(yMax)).astype(np.uint8).tolist()
    pubimg.publish(img)
    
    global state_x, state_y, statep_x, statep_y
    statep_x = state_x
    statep_y = state_y
    state_x, state_y = np.unravel_index(heatmap.argmax(), heatmap.shape) #only first occurrence returned

def turret():
    global state_x, state_y, updatetime, statep_x, statep_y, stash, img, pubimg
    pub = rospy.Publisher('/cmd_vel', Joy, queue_size=10)
    rospy.init_node('turret', anonymous=True)
    rospy.Subscriber('/roi', RegionOfInterest, callback)
    rate = rospy.Rate(10) # 10Hz

    # PID parameters
    outNeutral = 1024
    
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

    while not rospy.is_shutdown():
        if time() - updatetime < 1 and ((state_x - statep_x < xMax/5 and state_y - statep_y < yMax/2) or not stash):
            error_x = target_x - state_x
            output_x = Kp_x*error_x + Ki_x*(error_x+errorp_x) + Kd_x*(error_x-errorp_x)
            output_x = outNeutral - output_x
        
            error_y = state_y - target_y
            output_y = Kp_y*error_y + Ki_y*(error_y+errorp_y) + Kd_y*(error_y-errorp_y)
            output_y = outNeutral - output_y
        
            errorp_x = error_x
            errorp_y = error_y

            if abs(error_x) < xMax/10 and abs(error_y) < yMax/6:
                shoot = 1
            else:
                shoot =0

        else:
            if time() - updatetime > 2:
                stash = []
            heatmap = np.zeros((int(xMax),int(yMax)), dtype=np.uint8)
            img.data=np.resize(np.ravel(heatmap.T, order='C'), int(xMax)*int(yMax)).astype(np.uint8).tolist())
            pubimg.publish(img)
            state_x = 0
            state_y = 0
            output_x = outNeutral
            output_y = outNeutral
            shoot = 0
            
        output = Joy()
        output.buttons = [outNeutral, outNeutral, output_x, output_y, shoot]

        pub.publish(output)
        rate.sleep()

if __name__ == '__main__':
    try:
        turret()
    except rospy.ROSInterruptException:
        pass
