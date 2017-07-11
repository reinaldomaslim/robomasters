#!/usr/bin/env python

## PID control for yaw and pitch of turret towards target from target input from webcam
## Subsribe to /yolo2_detections topic to obtain position data
## Publish angular velocity of turret to arduino through /cmd_vel topic

## Small buff autoshooting 

import rospy
import numpy as np
import matplotlib.pyplot as plt
from time import time
from sensor_msgs.msg import Joy
from yolo2.msg import Detection, ImageDetections

state_x = 0
state_y = 0
stash = []
updatetime = time()
im_size=[640, 480]

#heatmap to publish
img = Image()
img.header.frame_id = '/heatmap'
img.height = int(yMax)
img.width = int(xMax)
img.encoding = 'MONO8'
img.step = int(xMax)

def callback(msg):
    global updatetime
    updatetime = time()

    frame = np.zeros((int(im_size[0]/10),int(im_size[1]/10)), dtype=np.uint8)

    for detection in msg.detections:
        x1 = ( detection.x - detection.width/2 ) * im_size[0]/10
        y1 = ( detection.y - detection.height/2) * im_size[1]/10
        x2 = ( detection.x + detection.width/2 ) * im_size[0]/10
        y2 = ( detection.y + detection.height/2) * im_size[1]/10
        frame[int(y1):int(y2), int(x1):int(x2)] += detection.confidence

    global stash
    if len(stash)==5:
        del stash[0]
    stash.append(frame)
    heatmap = np.sum(stash)

    img.data=np.resize(heatmap, int(im_size[0]/10)*int(im_size[1]/10)).astype(np.uint8).tolist()
    pubimg.publish(img)

    global state_x, state_y
    state_x, state_y = np.unravel_index(heatmap.argmax(), heatmap.shape) #only first occurrence returned

def turret():

    rospy.init_node('small_buff', anonymous=True)
    
    global state_x, state_y, updatetime
    pub = rospy.Publisher('/cmd_vel', Joy, queue_size=10)
    rospy.Subscriber('/yolo2/detections', ImageDetections, callback)
    rate = rospy.Rate(10) # 10Hz

    xMax = int(im_size[0]/10)
    yMax = int(im_size[1]/10)
    outMin = 524
    outNeutral = 1024
    outMax = 1524

    # needs tuning again
    Kp = 1.2
    Ki = 0.01
    Kd = 0.9

    target_x = xMax/2
    target_y = yMax/2
    
    errorp_x = 0
    errorp_y = 0

    while not rospy.is_shutdown():
        if time() - updatetime < 1:
            error_x = target_x - state_x
            output_x = Kp*error_x + Ki*(error_x+errorp_x) + Kd*(error_x-errorp_x)
            output_x = outNeutral - output_x
        
            error_y = state_y - target_y
            output_y = Kp*error_y + Ki*(error_y+errorp_y) + Kd*(error_y-errorp_y)
            output_y = outNeutral - output_y
        
            errorp_x = error_x
            errorp_y = error_y

            print "state = ", state_x, state_y
            print "target = ", target_x, target_y
            print "error = ", error_x, error_y

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
