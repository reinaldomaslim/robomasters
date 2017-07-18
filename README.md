# robomasters
original work by team MECATRON Nanyang Technological University for Robomasters 2017 in Shenzhen

# Base Robot
1. dji_ros: serial to arduino to publish /cmd_vel
2. rplidar_ros: lidar node publishes /scan
3. usb_cam: camera node publishes /image_rect_color
4. tf_broadcaster: transformations of robot, publishes /odometry
5. detect_enemy: detects enemy by clustering based on /scan, publishes /enemy_yolo
6. camera_masking: masked out camera based on enemy's position, publishes /image_rect_color_masked
7. armor_detection: detects enemy's armor from masked image, publishes armor's /roi
8. mission_planner: performs dodging and shooting 
9. rviz
10. image_view 

setup NUC to launch 1-8 in robomasters/launch/base.launch during stratup. Put the base on center of arena and let 10-20s 
without moving it. turn on NUC and voila, u have a running base with autoshooting and passive dodging. For debug, use wifi 
and run 9-10. SSH to nuc@192.168.1.100 for remote wifi connection.

# Buff Shooting Infantry
1. uvc_cam: camera node to publish /image_raw
2. ros_yolo2: runs darknet tiny-yolo2 deep learning, weights and cfg file in /data. publishes roi in /yolo2/detections 
3. dji_ros: serial to arduino
4. buff: uses heatmap and pid to shoot, publishes /cmd_vel

# UAV Landing Assistant
1. uav_fast.py: detects circular landing site by Ransac, blinks led as direction indicator on 4 uav arms


