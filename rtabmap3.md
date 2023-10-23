# RTAB-map : SLAM, Navigation
## ROS2 Humble & ROS1 Netic
## Part III

#### Author: Todor Arnaudov - Twenkid

#### Researching Rtabmap and it's deployment and application in simulations, mapping and robots 10.2023

### 6. Ego Planner Swarm

Eren's suggestion, I'm installing it on the same VM VBox Ub.20 https://github.com/ZJU-FAST-Lab/ego-planner-swarm

Successfully built and run - with adding missing header files. ZJU-FAST-Lab/ego-planner-swarm#65

![image](https://github.com/Twenkid/SLAM/assets/23367640/3267a942-5699-4b1f-9ea0-8002db496a91)


https://github.com/HKUST-Aerial-Robotics/plan_utils/tree/master/multi_map_server/msg_gen/cpp/include/multi_map_server

The planner can work for a single drone by setting a param. to 0.

An earlier system: https://github.com/ZJU-FAST-Lab/ego-planner

They mention CUDA acceleration and Realsense driver, but it's a bit outdated: 2.30 (currently it's 2.54.1), for Ub.16.04 and 18.04.

### 7. Point Clouds

Introduction of working with point clouds for ROS1, ROS2 and in general. Clustering etc.

If not using navigation to explicit coordinates within a map, the navigation could be searching for free ways through the points, calculating/storing distances to the points from the drone etc. (There could be topics which give this data directly).

http://wiki.ros.org/pcl/Overview

###  8. Survey: Outdoor-navigation-in-ros-current-state-future-direction

https://discourse.ros.org/t/outdoor-navigation-in-ros-current-state-future-direction/33599

In brief, there aren't many popular out-of-the-box options for aerial 3D navigation. The author mention Rtabmap and a few others.

A very good resource/forum: ROS discourse. Perhaps a community of some of the best ROS experts.

Also: https://robotics.stackexchange.com/

Gazebo and ROS answers have moved there recently.

### 9. Physical Depth Camera for proper testing: D435i etc.

I may need to operate with the real D435i camera and record datasets and bags myself.

**A project: Nano Laptop Computer**

Components:

* Jetson Nano - present
* Enclosing the copmputer in a small box
  * Plastic boxes for food: fresh meat, chocolates box: an appropriate one found, suitable for an 11.6" laptop
* Power: External battery for phones, 2 ports: present - one port for the computer and another for the monitor
* Monitor: Small HDMI monitor, powered with 5 V: second battery port
  * ? Options: LVDS, EDI ...  MPI DSI ? - not supported; DSI supported for cameras; only HDMI support: LCD displays with a converter board from HDMI
  *  LVDS - older laptop moniotors; EDI ... Price: $40-50-100 with the board from Aliexpress: some items may require to contact the sellers for the exact quote
  *  What size is desirable? 10", 11" -- 7" is too small, 9" may be acceptable; 7" etc. usually are 1024x600 which is low; 1920x1080 is deseirable, depending on the price
  *  Can displays from scrap/old cheap tablets be used? Not sure yet, possibly different connectors etc. and may be proprietary
  *  Monitors from old laptops are possible as well -- sort out the interfaces and power
* Web cameras are supported through USB as well (tested with two Logitech C270 and a DIY microscope camera with a Gembird; not tested for exact supported high resolutions etc.)
* Keyboard and mouse: either USB, or a Bluetooth dongle + wireless ones
* Depth camera: D435i etc.
* This system can be used to "fly by hand" a "virtual drone" and record "low height flies" in the park, in corridors in buildings and anywhere and to use the data for testing different SLAM and Navigation systems and algorithms.

However it may happen to be too slow, 4 core Arm 1.5 GHz, now oudtaded version.

So far a full-fledged i5-3320M laptop was used to record point clouds.

With a Navigation/SLAM system working on the computer we can partially simulate the control of the drone as well: the system could be made to display the desired directions/trajectory adjustments and the human operator could act as the flight controller. So the operator may see whether the directions were correct or the drone heads to the objects.

### 10. Technical solutions for working on Ubuntu 18.04 and 22.04 and combining ROS2 and ROS1 for Jetson Nano

It'd be best if all can run in ROS2, however there is a lot of powerful software which is still only on ROS1 and nobody ports it.
 Are ROS1-ROS2 bridges possible? (Or feasible)

There are possible LXD containers for runing ROS2 Humble on Ubuntu 18.04, but it may not work.
ROS1 Noetic can be installed natively so it seems more reasonable.

Similarly if ROS1 is needed in a system with native ROS2, the ROS1 could be in a container.

Currently I worked with various VMs:

2 x WSL2s: Ubuntu 20.04 and 22.04
1 x VirtualBox: Xubuntu 20.04 (ROS1 Noetic, Rtabmap ROS1)
1 x HyperV: Ubuntu 22.04 (ROS2 Humble, Rtabmap ROS2)

### 11. Adding or exercising external control to Gazebo simulations from other ROS nodes such as the experimental package for obstacle avoidance

Depth camera simulated data is sent, a basic node captures the images and could process them (without a complete Navigation system).
A simulated drone or another vehicle should be driven to fly through complex paths and be controlled by a pilot, external remote control and then in autonomous node, so it can wander around the world with its own decisions.
So it should be remotely controlled with direct commands which are manually adjusting the parameters in a continuous way.

The ros pkg should be able to add commands to the simulation.

All these require:

* ros_gz bridge (done) - maps gazebo topics to ros ... gazebo topic -l ... ros2 topic list ... ros2 node list 
* mapping to proper topics, capturing them by the node, recognizing obstacles, measuring distance etc.

### 12. Installation of ROS1 on Jetson Nano and Docker for ROS2?

https://github.com/introlab/rtabmap_ros/

[introlab/rtabmap#427](https://github.com/introlab/rtabmap/issues/427)

https://github.com/introlab/rtabmap_ros/tree/master/docker

### 13. Issues with feeding data from a Gazebo simulation, it doesn't receive data for now

ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_args:="--delete_db_on_start" rgb_topic:=/front_camera depth_topic:=/rgbd_camera/depth_image depth:=true frame_id:=base_link approx_sync:=false
qos:=1 rviz:=true

### 14. Reverse engineering projects, exploring their topics and nodes, mapping their representations to Gazebo, converting point clouds to polygons, mapping drone's frame of reference and scale to the ones in the Gazebo simulation etc.

A bit of reverse engineering may be needed some of the projects, because they don't have a detailed documentation. Ego_planner_ projects are very good, but one has to sample the topics and nodes and read the code to see the details beyond the example which is run easily.

```
rostopic list
rostopic info
roslaunch rqt_image_view  ...

...

ros2 run rqt_image_view rqt_image_view /rgbd_camera/depth_image  

...

osboxes@osboxes:~$ rostopic info /voxel_cloud
Type: sensor_msgs/PointCloud2

Publishers: 
 * /points_xyzrgb (http://osboxes:40935/)

Subscribers: 
 * /rviz (http://osboxes:38937/)

osboxes@osboxes:~$ rostopic type /voxel_cloud 
sensor_msgs/PointCloud2

```

The scale/mapping of the simulations of these examples have to be converted into ours and we have to be able to map the drone's frame of reference, orientation etc. with the one in the simulations of the examples.

As Gazebo is used and the 3D data is usually given as Point Clouds, and there are also 2D grid maps and voxel 3D maps, approprite tools are required in order to convert these representations into polygons and models for Gazebo. One example: https://gazebosim.org/api/gazebo/4.0/pointcloud.html

Geometry may have to be spawned on the fly and eventually to pair it with textures (from the RGBD point cloud data) if it is to mirror what the vehicle or the UAV/drone actually sees.

For a demo just with a simulation, another solution could be to convert the models from the examples (such as ego_planner ones) to the format of Gazebo etc.

The 2D grid occupancy maps could be represented as high columns.

### 15. Successfully connecting D435i to Rtabmap on the laptop with Ubuntu 22.04

ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true

ros2 param set /camera/camera depth_module.emitter_enabled 0

ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

To do:

Calibrate IMU of D435i: Done.

https://dev.intelrealsense.com/docs/imu-calibration-tool-for-intel-realsense-depth-camera rs-imu-calibration.py

librealsense/wrappers ...

### 16. Nav2 Naigation package etc.

https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md

Rtabmap overall system structure: https://www.einfochips.com/blog/autonomous-navigation-of-amr-using-rtab-map-and-tof-camera/

Nav2 (ros_planning/navigation2)

It is for 2D occupancy grid / maps, but it can be used as RTBM creates an occupancy grid as well for the objects that touch the ground. If relying just on such a map, in a simple first attempt, if running through the Nav2 planned paths, the drone may only go around objects, by keeping the same height.

If we intercept the planning, the actual height of the objects located at the coordinates in the grid map could be verified from a PointCloud (PCL) from Rtabmap and if the object is lower the drone could overide the 2D occupancy map and go forward. Or there could be a command to go higher a bit, check the PCL again etc. This may require more complex planning sequence from our autonomous control.

Nav2: Concepts, topics, ...

```
map
map_server
costmap
save_map
load_map
map_server
map_saver
map_io

map_server map.yaml
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

# map_server_params.yaml
map_server:
    ros__parameters:
        yaml_filename: "map.yaml"

 map_server __params:=map_server_params.yaml

combined_params.yaml
map_server1:
    ros__parameters:
        yaml_filename: "some_map.yaml"

map_server2:
    ros__parameters:
        yaml_filename: "another_map.yaml"

 process_with_multiple_map_servers __params:=combined_params.yaml

 ```
 
### 17. Study d435i rtabmap example launch file and skyhub_server simulation, Gazebo .sdf description etc. and try to match them: rtabmap example requires stereo camera and it remaps from the infrared cameras of d435i (a laser projects a grid pattern)

https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_stereo.launch.py

```
  parameters=[{
          'frame_id':'camera_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info')]

Also a clue about how to map the camera frame of reference to the Gazebo sim.:

````
```
# The IMU frame is missing in TF tree, add it:
    Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
])
```

````

This example takes from the camera stereo images, it doesn't directly sample depth data and SLAM reconstructs it from the images

The next example uses depth:
https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_infra.launch.py

 $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true

 $ ros2 param set /camera/camera depth_module.emitter_enabled 0

 $ ros2 launch rtabmap_examples realsense_d435i_infra.launch.py
 remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/infra1/image_rect_raw'),
          ('rgb/camera_info', '/camera/infra1/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]
...
````

Continues in rtabmap4.md ...
