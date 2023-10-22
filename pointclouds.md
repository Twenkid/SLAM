# Pointclouds for SLAM and Navigation
### Research by Todor Arnaudov - Twenkid
#### 10.2023

### Meshes Optimized


![image](https://github.com/Twenkid/SLAM/assets/23367640/af590632-0288-4757-b6fe-e574dcd07939)

![image](https://github.com/Twenkid/SLAM/assets/23367640/8280299f-d37b-4bd1-bb5c-2ff6af3e70fb)

![image](https://github.com/Twenkid/SLAM/assets/23367640/8aed8083-e24d-4957-9b41-c3390a8cf446)


### Pointcloud RViz Directly from D435i

![image](https://github.com/Twenkid/SLAM/assets/23367640/0265d36e-77ef-4e70-8b32-3a64b3b842ca)

![image](https://github.com/Twenkid/SLAM/assets/23367640/752a4361-9645-4789-9ba9-290f620a18cb)

Realsense


ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true

/camera/depth/image_rect_raw


As thought initially, it seems possibly this could be a feasible solution for simple obstacle avoidance before implementing full-fledged navigation, as long as I start to be able to control the drone from a ROS node.

A possible workflow:

The drone would have current and target coordinates. (If there is a complex path, it should be broken down to pieces for which the following procedure would be applied).

The default direction is straingt towards the goal: the drone will calculate the angle from its frame of reference and rotate towards the goal.

It should avoid seeing near objects with some predefined distance (gradient) in a window in the center of the image, covering the dimensions of the drone.

The drone will rotate up to a certain amount of degrees to the right and/or to the left until seeing a clear view and moving forward for a while/a quant of motion.

Check if the goal coordinate is reached (no resolution of height etc. for now). If it is reached: Go to 7.

Go to 2.

Goal is reached. End.
