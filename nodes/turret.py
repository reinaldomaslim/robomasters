#!/usr/bin/env python

## PID control for yaw and pitch of turret towards target from target input from webcam
## Subsribe to /roi topic to obtain position data
## Publish angular velocity of turret to arduino through /vel_cmd topic

import rospy
import numpy as np
from time import time
from sensor_msgs.msg import Joy, RegionOfInterest

#global variables
state_x = 0
state_y = 0
stash = []
updatetime = time()

#camera parameters
xMax = rospy.get_param('/usb_cam/image_width') / 10
yMax = rospy.get_param('/usb_cam/image_height') / 10

def callback(roi):
    global updatetime
    updatetime = time()

    #calculate center of roi
    center = [0, 0]
    center[0] = (roi.x_offset + roi.width/2) / 10
    center[1] = (roi.y_offset + roi.height/2) / 10

    global stash
    if len(stash)==10:
            del stash[0]
    stash.append(center)

    heatmap = np.zeros((xMax,yMax), dtype=np.uint8)
    for ctr in stash:
        heatmap[ctr[0], ctr[1]] += 1
    
    global state_x, state_y
    state_x, state_y = np.unravel_index(heatmap.argmax(), heatmap.shape) #only first occurrence returned

def turret():
    global state_x, state_y, updatetime
    pub = rospy.Publisher('/vel_cmd', Joy, queue_size=10)
    rospy.init_node('turret', anonymous=True)
    rospy.Subscriber('/roi', RegionOfInterest, callback)
    rate = rospy.Rate(10) # 10Hz

    # PID parameters
    outMin = 524
    outNeutral = 1024
    outMax = 1524
    # needs tuning again
    Kp = 1.6
    Ki = 0.001
    Kd = 0.7

    target_x = xMax/2
    target_y = yMax/2
    
    errorp_x = 0
    errorp_y = 0

    while not rospy.is_shutdown():
        if time() - updatetime < 3:
            error_x = target_x - state_x
            output_x = Kp*error_x + Ki*(error_x+errorp_x) + Kd*(error_x-errorp_x)
            output_x = outNeutral - output_x
        
            error_y = state_y - target_y
            output_y = Kp*error_y + Ki*(error_y+errorp_y) + Kd*(error_y-errorp_y)
            output_y = outNeutral - output_y
        
            errorp_x = error_x
            errorp_y = error_y

            if abs(error_x) < xMax/10 and abs(error_y) < yMax/6:
                shoot = 1
            else:
                shoot =0

        else:
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