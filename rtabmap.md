# RTAB-map : SLAM, Navigation, ... ROS2

* Running an example with turtlebotsim3 - first in WSL2, then in a HyperV VM
* Mapping with Rtabmap with depth cameras:
* Running on native Ubuntu 22.04 - rtabmap seems preinstalled
* Running Rtabmap examples and connecting with the drone simulation's depth camera and other sensors 
* Trying out ROS1 version bags examples with ROS1 Noetic in VM VirtualBox
* Ego Planner Swarm (ROS1)
* A new fork, but still ROS1 (Noetic): 
https://github.com/ZJU-FAST-Lab/EGO-Planner-v2 

https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/main/swarm-playground/%5BREADME%5D_Brief_Documentation_for_Swarm_Playground.pdf

* See the parameters in the manual
* Point Clouds
* Survey: Outdoor-navigation-in-ros-current-state-future-direction
* Physical Depth Camera for proper testing: D435i etc.
* Technical solutions for working on Ubuntu 18.04 and 22.04 and combining ROS2 and ROS1?
* Adding or exercising external control to Gazebo simulations from other ROS nodes such as the experimental ros_obs_avd

* For embedded devices: Installation of ROS1 on Jetson Nano and Docker for ROS2?

* Issues with feeding data from the simulation, it doesn't receive data for now

* Reverse engineering projects, exploring their topics and nodes, mapping their representations to Gazebo, converting point clouds to polygons, mapping drone's frame of reference and scale to the ones in the Gazebo simulation etc.

* Successfully connecting D435i to Rtabmap on the laptop with Ubuntu 22.04

* Nav2 Naigation package etc.

* Study d435i rtabmap example launch file and skyhub_server simulation [X],

* Gazebo .sdf description [.+] etc. and try to match them: rtabmap example requires stereo camera and it remaps from the infrared cameras of d435i (a laser projects a grid pattern)

* --> Done, but there are things which I can't do yet as additional topics are required such as camera_info etc. I did what I could so far. [2.10.2023]


/opt/ros/humble/share/rtabmap_examples/launch/simulated_color.launch.py
ros2 launch rtabmap_examples simulated_color.launch.py
Issues with mapping base_link -- camera_link: probably fixed (below):

rgbd_odometry-1]    /front_camera_info
[rgbd_odometry-1] Warning: Invalid frame ID "camera_link" passed to canTransform argument target_frame - frame does not exist
[rgbd_odometry-1]          at line 93 in ./src/buffer_core.cpp
[rgbd_odometry-1] [ERROR] [1696280283.242013534] [rgbd_odometry]: Could not transform IMU msg from frame "base_link" to frame "camera_link", TF not available at time 1696280283.190770
[rtabmap_viz-3] 2023-10-02 23:58:03.280 (   6.782s) [        A46112C0]vtkOpenGLPolyDataMapper:
I probably fixed that:

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link', #camera_link
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True, #the default is true but Rtab doesn't receive the messages?
          'wait_imu_to_init':True}]
However:

[rtabmap-2]    /odom_info
[rtabmap_viz-3] [WARN] [1696281206.177541760] [rtabmap_viz]: rtabmap_viz: Did not receive data since 5 seconds! Make sure the input topics are published ("$ rostopic hz my_topic") and the timestamps in their header are set. If topics are coming from different computers, make sure the clocks of the computers are synchronized ("ntpdate"). If topics are not published at the same rate, you could increase "queue_size" parameter (current=10). 
[rtabmap_viz-3] rtabmap_viz subscribed to (approx sync):
[rtabmap_viz-3]    /odom \
[rtabmap_viz-3]    /front_camera \
[rtabmap_viz-3]    /rgbd_camera/depth_image \
[rtabmap_viz-3]    /front_camera_info \
[rtabmap_viz-3]    /odom_info
Maybe something from Gazebo and Mavros has to be set regarding: "the timestamps in their header are set."

front_camera and depth_image are there

odom is not exactly with that name

There are no _info topics.

```
ros2 topic list
/bottom_camera
/camera/accel/imu_info
/camera/accel/metadata
/camera/accel/sample
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/theora
/camera/color/metadata
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/theora
/camera/depth/metadata
/camera/extrinsics/depth_to_accel
/camera/extrinsics/depth_to_color
/camera/extrinsics/depth_to_gyro
/camera/extrinsics/depth_to_infra1
/camera/extrinsics/depth_to_infra2
/camera/gyro/imu_info
/camera/gyro/metadata
/camera/gyro/sample
/camera/imu
/camera/infra1/camera_info
/camera/infra1/image_rect_raw
/camera/infra1/image_rect_raw/compressed
/camera/infra1/image_rect_raw/compressedDepth
/camera/infra1/image_rect_raw/theora
/camera/infra1/metadata
/camera/infra2/camera_info
/camera/infra2/image_rect_raw
/camera/infra2/image_rect_raw/compressed
/camera/infra2/image_rect_raw/compressedDepth
/camera/infra2/image_rect_raw/theora
/camera/infra2/metadata
/detection_camera
/diagnostics
/events/read_split
/events/write_split
/front_camera
/mavros/adsb/send
/mavros/adsb/vehicle
/mavros/battery
/mavros/cam_imu_sync/cam_imu_stamp
/mavros/camera/image_captured
/mavros/cellular_status/status
/mavros/companion_process/status
/mavros/esc_status/info
/mavros/esc_status/status
/mavros/esc_telemetry/telemetry
/mavros/estimator_status
/mavros/extended_state
/mavros/fake_gps/mocap/pose
/mavros/geofence/fences
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gp_lp_offset
/mavros/global_position/gp_origin
/mavros/global_position/local
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/raw/satellites
/mavros/global_position/rel_alt
/mavros/global_position/set_gp_origin
/mavros/gps_input/gps_input
/mavros/gps_rtk/rtk_baseline
/mavros/gps_rtk/send_rtcm
/mavros/gpsstatus/gps1/raw
/mavros/gpsstatus/gps1/rtk
/mavros/gpsstatus/gps2/raw
/mavros/gpsstatus/gps2/rtk
/mavros/home_position/home
/mavros/home_position/set
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/diff_pressure
/mavros/imu/mag
/mavros/imu/static_pressure
/mavros/imu/temperature_baro
/mavros/imu/temperature_imu
/mavros/landing_target/lt_marker
/mavros/landing_target/pose
/mavros/landing_target/pose_in
/mavros/local_position/accel
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/local_position/pose_cov
/mavros/local_position/velocity_body
/mavros/local_position/velocity_body_cov
/mavros/local_position/velocity_local
/mavros/log_transfer/raw/log_data
/mavros/log_transfer/raw/log_entry
/mavros/mag_calibration/report
/mavros/mag_calibration/status
/mavros/manual_control/control
/mavros/manual_control/send
/mavros/mission/reached
/mavros/mission/waypoints
/mavros/mocap/pose
/mavros/mocap/tf
/mavros/mount_control/command
/mavros/mount_control/orientation
/mavros/mount_control/status
/mavros/nav_controller_output/output
/mavros/obstacle/send
/mavros/odometry/in
/mavros/odometry/out
/mavros/onboard_computer/status
/mavros/optical_flow/ground_distance
/mavros/optical_flow/raw/optical_flow
/mavros/optical_flow/raw/send
/mavros/param/event
/mavros/play_tune
/mavros/radio_status
/mavros/rallypoint/rallypoints
/mavros/rangefinder/rangefinder
/mavros/rangefinder_pub
/mavros/rangefinder_sub
/mavros/rc/in
/mavros/rc/out
/mavros/rc/override
/mavros/setpoint_accel/accel
/mavros/setpoint_attitude/cmd_vel
/mavros/setpoint_attitude/thrust
/mavros/setpoint_position/global
/mavros/setpoint_position/global_to_local
/mavros/setpoint_position/local
/mavros/setpoint_raw/attitude
/mavros/setpoint_raw/global
/mavros/setpoint_raw/local
/mavros/setpoint_raw/target_attitude
/mavros/setpoint_raw/target_global
/mavros/setpoint_raw/target_local
/mavros/setpoint_trajectory/desired
/mavros/setpoint_trajectory/local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/state
/mavros/statustext/recv
/mavros/statustext/send
/mavros/terrain/report
/mavros/time_reference
/mavros/timesync_status
/mavros/trajectory/desired
/mavros/trajectory/generated
/mavros/trajectory/path
/mavros/tunnel/in
/mavros/tunnel/out
/mavros/vfr_hud
/mavros/vision_pose/pose
/mavros/vision_pose/pose_cov
/mavros/vision_speed/speed_twist
/mavros/vision_speed/speed_twist_cov
/mavros/vision_speed/speed_vector
/mavros/wheel_odometry/distance
/mavros/wheel_odometry/odom
/mavros/wheel_odometry/rpm
/mavros/wind_estimation
/move_base_simple/goal
/parameter_events
/rgbd_camera/depth_image
/rosout
/tf
/tf_static
/uas1/mavlink_sink
/uas1/mavlink_source
```


Also it is not sure it the images from depth and color camera are synchronized completely: maybe the timestamp will solve it?

A proper SLAM needs synchronized data, the sample d435i launch files require completely synchronized messages.

In front_camera there is a stamp: ros2 topic echo /front_camera

header: stamp: sec: 1488 nanosec: 70000000 frame_id: iris_with_ardupilot/iris_with_standoffs/front_camera_link/camera height: 240 width: 320 encoding: rgb8 is_bigendian: 0 step: 960

Depth camera also has a stamp:
header: stamp: sec: 1551 nanosec: 232000000 frame_id: iris_with_ardupilot/iris_with_standoffs/rgbd_camera_link/rgbd_camera height: 240 width: 320 encoding: 32FC1 is_bigendian: 0 step: 1280 data:


Maybe /odom should be mapped to: /mavros/odometry/out  ?

Proper _info topics have to be added.

```
ros2 topic hz rgbd_camera/depth_image
2023-10-02 23:57:02.140 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7435: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:02.140 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7437: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:02.140 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7439: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:02.140 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7441: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:02.140 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7445: open_and_lock_file failed -> Function open_port_internal
average rate: 30.881
	min: 0.020s max: 0.059s std dev: 0.00819s window: 32
average rate: 30.888
	min: 0.020s max: 0.076s std dev: 0.00877s window: 63
average rate: 30.649
	min: 0.017s max: 0.076s std dev: 0.00901s window: 94
average rate: 30.210
	min: 0.017s max: 0.110s std dev: 0.01074s window: 123
average rate: 30.649
	min: 0.017s max: 0.110s std dev: 0.00999s window: 156
average rate: 30.338
	min: 0.017s max: 0.110s std dev: 0.01078s window: 185
^Cskyhub@skyhub-server:/opt/ros/humble/lib$ ros2 topic hz /front_camera 
2023-10-02 23:57:15.271 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7435: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:15.271 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7437: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:15.271 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7439: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:15.271 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7441: open_and_lock_file failed -> Function open_port_internal
2023-10-02 23:57:15.271 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7445: open_and_lock_file failed -> Function open_port_internal
average rate: 29.181
	min: 0.017s max: 0.092s std dev: 0.01366s window: 32
average rate: 29.745
	min: 0.017s max: 0.092s std dev: 0.01192s window: 63
average rate: 30.501
	min: 0.015s max: 0.092s std dev: 0.01051s window: 96
average rate: 30.480
	min: 0.015s max: 0.092s std dev: 0.00961s window: 127
Current version of the edited d435 launch file (simulated_color.launch.py):

/opt/ros/humble/share/rtabmap_examples/launch/simulated_color.launch.py
```


Recording the first small maps in the room [X]
Recording big maps outdoors: color, stereo, infrared and exploring them with Rtabmap. Exporting to point cloud. Recording ros2 bag with d435i for replay on the server. [X]
Opening ply with CloudCompare, MeshLab, Blender [X]
To do:

* Convert sample point clouds to polygon models for Gazebo; with CloudCompare: [X] -- exported with Rtab as meshes See: pointclouds.md Another path: https://gazebosim.org/api/gazebo/4.0/pointcloud.html

* Process point cloud with a dedicated ros2 node (use pcd structure etc.) []

* Cluster, recognize closest objects/boundaries; distances to objects, raycast ...

* Reference for adjusting coordinate frames with Gazebo simulation etc.: https://www.ros.org/reps/rep-0105.html

* "When defining coordinate frames with respect to a global reference like the earth: The default should be to align the x-axis east, y-axis north, and the z-axis up at the origin of the coordinate frame. If there is no other reference the default position of the z-axis should be zero at the height of the WGS84 ellipsoid.

```
/earth -- /map -- /odom -- /base_link
```

"Frame Authorities

The transform from odom to base_link is computed and broadcast by one of the odometry sources.

The transform from map to base_link is computed by a localization component. However, the localization component does not broadcast the transform from map to base_link. Instead, it first receives the transform from odom to base_link, and uses this information to broadcast the transform from map to odom.

The transform from earth to map is statically published and configured by the choice of map frame. If not specifically configured a fallback position is to use the initial position of the vehicle as the origin of the map frame. If the map is not georeferenced so as to support a simple static transform the localization module can follow the same procedure as for publishing the estimated offset from the map to the odom frame to publish the transform from earth to map frame."

... external motivators - a robot in an elevator [wind thrusts for the drone]

...the inertial odom frame should always remain continuous. ..

http://official-rtab-map-forum.206.s1.nabble.com/Filtering-rtabmap-localization-jumps-with-robot-localization-in-2D-td5931.html

http://docs.ros.org/en/noetic/api/robot_localization/html/index.html

https://answers.ros.org/question/390017/rtab_map-localization-tuning-goal-setting-questions/?answer=390366

About the goals in rtabmap:

"This is explained in this paper. https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/8/87/LabbeAURO2017.pdf

"Long-Term Online Multi-Session Graph-Based SPLAM with Memory Management" 
* Simultaneous Planinng, Localization and Mapping
* RTAB-Map's goal interface is an intermediary between the module sending the goal and move_base. The goal can be metric or a node in the graph (even a label "kitchen") which will be translated to a metric goal. The metric goal is attached to closest node in the graph, so if the graph is deformed because of a loop closure, the goal will be republished to move_base with the new location matching the optimized map. This is essential useful only in SLAM mode. For the localization mode (with default RGBD/OptimizeFromGraphEnd=false), you could send directly your metric pose to move_base, as the map is fixed and will never change."

Rtabmap forum: http://official-rtab-map-forum.206.s1.nabble.com/


1.RTABmap experiments in VMs
Requirements
ROS2 branch (it is under construction, the mature versions are ROS1) https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros

https://github.com/ROBOTIS-GIT/turtlebot3_simulations

Setting Up TurtleBot3 Simulation in ROS 2 Humble Hawksbill

https://medium.com/@nilutpolk/setting-up-turtlebot3-simulation-in-ros-2-humble-hawksbill-70a6fcdaf5de

Be careful with existing gazebo libraries etc.! How to check what's installed:

After rtabmap:
http://wiki.ros.org/RealSense

https://github.com/IntelRealSense/realsense-ros

Environment of the current experiments: WSL2: Ubuntu 22.04 in Windows 10
sudo apt list --installed
sudo apt list --installed | grep gazebo
sudo apt list --installed | grep ros 
etc.

ROS-specific:

ros2 pkg list
ros2 pkg list | grep turtle 

etc.:

turtlebot3
turtlebot3_bringup
turtlebot3_cartographer
turtlebot3_description
turtlebot3_example
turtlebot3_msgs
turtlebot3_navigation2
turtlebot3_node
turtlebot3_teleop
turtlesim

ros2 pkg list |grep keyb
keyboard_handler
teleop_twist_keyboard
... A good introduction and reference for ROS2 commands: https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS2-Basics.html

Gazebo11 installation if the repos are not set:
https://classic.gazebosim.org/tutorials?tut=install_ubuntu

Terminal_1
```
source /opt/ros/humble/setup.bash

sudo apt install gazebo11
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-cartographer 
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2 
sudo apt install ros-humble-nav2-bringup

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel

cd ~/turtlebot3_ws

colcon build 

source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# We don't need an empty world
# ros2 launch turtlebot3_gazebo empty_world.launch.py  
# model can be also waffle, waffle_pi
```

In Terminal 2 :
```
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard

Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit

currently:      linear velocity 0.22     angular velocity -2.84
currently:      linear velocity 0.22     angular velocity -2.84
```


Terminal 3

source /opt/ros/humble/setup.bash

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

It will run a Gazebo simulation with the environment and the object of the robot. Sample output:

```
[INFO] [launch]: All log files can be found below /home/tosh/.ros/log/2023-09-12-21-05-13-134490-DESKTOP-LQSDBBS-30895
[INFO] [launch]: Default logging verbosity is set to INFO
urdf_file_name : turtlebot3_waffle.urdf
urdf_file_name : turtlebot3_waffle.urdf
[INFO] [gzserver-1]: process started with pid [30899]
[INFO] [gzclient-2]: process started with pid [30901]
[INFO] [robot_state_publisher-3]: process started with pid [30903]
[INFO] [spawn_entity.py-4]: process started with pid [30905]
[robot_state_publisher-3] [INFO] [1694541916.030919889] [robot_state_publisher]: got segment base_footprint
[robot_state_publisher-3] [INFO] [1694541916.031072489] [robot_state_publisher]: got segment base_link
[robot_state_publisher-3] [INFO] [1694541916.031084489] [robot_state_publisher]: got segment base_scan
[robot_state_publisher-3] [INFO] [1694541916.031091389] [robot_state_publisher]: got segment camera_depth_frame
[robot_state_publisher-3] [INFO] [1694541916.031097489] [robot_state_publisher]: got segment camera_depth_optical_frame
[robot_state_publisher-3] [INFO] [1694541916.031103889] [robot_state_publisher]: got segment camera_link
[robot_state_publisher-3] [INFO] [1694541916.031109989] [robot_state_publisher]: got segment camera_rgb_frame
[robot_state_publisher-3] [INFO] [1694541916.031115989] [robot_state_publisher]: got segment camera_rgb_optical_frame
[robot_state_publisher-3] [INFO] [1694541916.031122189] [robot_state_publisher]: got segment caster_back_left_link
[robot_state_publisher-3] [INFO] [1694541916.031128189] [robot_state_publisher]: got segment caster_back_right_link
[robot_state_publisher-3] [INFO] [1694541916.031134089] [robot_state_publisher]: got segment imu_link
[robot_state_publisher-3] [INFO] [1694541916.031139889] [robot_state_publisher]: got segment wheel_left_link
[robot_state_publisher-3] [INFO] [1694541916.031145689] [robot_state_publisher]: got segment wheel_right_link
[spawn_entity.py-4] [INFO] [1694541916.929176289] [spawn_entity]: Spawn Entity started
[spawn_entity.py-4] [INFO] [1694541916.929658489] [spawn_entity]: Loading entity XML from file /home/tosh/ros2/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
[spawn_entity.py-4] [INFO] [1694541916.930671889] [spawn_entity]: Waiting for service /spawn_entity, timeout = 30
[spawn_entity.py-4] [INFO] [1694541916.931108789] [spawn_entity]: Waiting for service /spawn_entity
```

With the editor of Gazebo, left panel World -- Models -- added House ... Toolbar, The Cross-tool (Translate tool), next to the arrow, on the right) to Translate the turtle to another room which will allow to walk more.

