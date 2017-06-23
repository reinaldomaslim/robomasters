#!/usr/bin/env python

## PID control for yaw and pitch of turret towards target from target input from webcam
## Subsribe to /front_cam/center topic to obtain state_x and state_y
## Publish angular velocity of turret to arduino through /vel_cmd topic

import rospy
from time import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

state_x = 0
state_y = 0
updatetime = time()

def callback(data):
    global updatetime
    updatetime = time()
    global state_x
    state_x = data.x
    global state_y
    state_y = data.y

def turret():
    global state_x
    global state_y
    global updatetime
    pub = rospy.Publisher('/vel_cmd', Joy, queue_size=10)
    rospy.init_node('turret', anonymous=True)
    rospy.Subscriber('/center', Vector3, callback)
    rate = rospy.Rate(10) # 10Hz

    # PID parameters
    xMax = 640
    yMax = 480
    # outMin = 364
    outMin = 524
    outNeutral = 1024
    # outMax = 1684
    outMax = 1524

    Kp = 0.51
    Ki = 0
    Kd = 0.6

    target_x = 320
    target_y = 240
    
    errorp_x = 0
    errorp_y = 0

    while not rospy.is_shutdown():
        if time() - updatetime < 0.15:
            error_x = target_x - state_x
            output_x = Kp*error_x + Ki*(error_x+errorp_x) + Kd*(error_x-errorp_x)
            output_x = outNeutral - output_x
        
            error_y = state_y - target_y
            output_y = Kp*error_y + Ki*(error_y+errorp_y) + Kd*(error_y-errorp_y)
            output_y = outNeutral - output_y
        
            errorp_x = error_x
            errorp_y = error_y

            if abs(error_x) < 100 and abs(error_y) < 100:
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