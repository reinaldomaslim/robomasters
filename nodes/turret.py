#!/usr/bin/env python

## PID control for yaw and pitch of turret towards target from target input from webcam
## Subsribe to /roi topic to obtain position data
## Publish angular velocity of turret to arduino through /cmd_vel topic

import rospy
import numpy as np
from time import time
from sensor_msgs.msg import Joy, RegionOfInterest, Image
# import matplotlib.pyplot as plt

#global variables
state_x = 0.0
state_y = 0.0
statep_x = state_x
statep_y = state_y
stash = []
updatetime = time()
updatetimep = time()
newDetection = False

#camera parameters
xMax = 1024 / 10.0
yMax = 576 / 10.0

def callback(msg):
    global updatetime, updatetimep, newDetection
    updatetimep = updatetime
    updatetime = time()
    if updatetime - updatetimep > 3:
        newDetection = True
    else:
        newDetection = False

    #calculate center and size of roi
    roi = [0.0, 0.0, 0.0] #center_x, center_y, size
    roi[0] = (msg.x_offset + msg.width/2) / 10.0
    roi[1] = (msg.y_offset + msg.height/2) / 10.0
    roi[2] = msg.width * msg.height / 10.0

    global stash
    # print "/roi = ", msg.x_offset, msg.y_offset

    if len(stash)==5:
            del stash[0]
    stash.append(roi)
    



    heatmap = np.zeros((xMax,yMax), dtype=np.uint8)
    for obj in stash:
        heatmap[int(obj[0]), int(obj[1])] += 0.1*obj[2]

    print heatmap.size
    msg=Image()
    msg.step=5814/102
    msg.height=int(xMax)
    msg.width=int(yMax)
    msg.data=np.resize(heatmap, 5814).astype(np.uint8).tolist()
    print np.amax(heatmap)
    msg.encoding='mono8'
    image_pub.publish(msg)

    # plt.figure(figsize=(int(xMax/10),int(yMax/10)))
    # plt.imshow(heatmap, cmap='hot')
    
    global state_x, state_y, statep_x, statep_y
    statep_x = state_x
    statep_y = state_y
    state_x, state_y = np.unravel_index(heatmap.argmax(), heatmap.shape) #only first occurrence returned




def turret():
    global state_x, state_y, statep_x, statep_y, updatetime, updatetimep, newDetection
    pub = rospy.Publisher('/cmd_vel', Joy, queue_size=10)

    global image_pub 
    image_pub = rospy.Publisher('/heatmap', Image, queue_size=1)

    rospy.init_node('turret', anonymous=True)
    rospy.Subscriber('/roi', RegionOfInterest, callback)


    rate = rospy.Rate(10) # 10Hz

    # PID parameters
    outMin = 524
    outNeutral = 1024
    outMax = 1524
    
    # needs tuning again
    Kp_x = 1.5
    Ki_x = 0.0
    Kd_x = 0.8

    Kp_y = 1.5
    Ki_y = 0
    Kd_y = 0.8

    target_x = xMax/2.0
    target_y = yMax/2.0
    
    errorp_x = 0
    errorp_y = 0

    while not rospy.is_shutdown():
        currtime = time()
        if currtime - updatetime < 1 and ((state_x - statep_x < xMax/5 and state_y - statep_y < yMax/2) or newDetection):
            error_x = target_x - state_x
            output_x = Kp_x*error_x + Ki_x*(error_x+errorp_x) + Kd_x*(error_x-errorp_x)
            output_x = outNeutral - output_x
        
            error_y = state_y - target_y
            output_y = Kp_y*error_y + Ki_y*(error_y+errorp_y) + Kd_y*(error_y-errorp_y)
            output_y = outNeutral - output_y

            print "p ", Kp_x*error_x , "    d ", Kd_x*(error_x-errorp_x)
        
            errorp_x = error_x
            errorp_y = error_y

            if abs(error_x) < xMax/10 and abs(error_y) < yMax/6:
                shoot = 1
            else:
                shoot =0

        else:
            print "diff = ", state_x - statep_x
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