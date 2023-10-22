# RTAB-map : SLAM, Navigation
## ROS2 Humble & ROS1 Netic
## Part II

#### Author: Todor Arnaudov - Twenkid

#### Researching Rtabmap and it's deployment and application in simulations, mapping and robots 10.2023

3. Rtabmap on the server
Rtabmap seems already available.

What to do?
The call:
```
ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint subscribe_scan:=true depth:=false approx_sync:=true odom_topic:=/odom scan_topic:=/scan qos:=2 args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1" use_sim_time:=true rviz:=true
```
Has to be adjusted:

? -->
```
ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=true frame_id:=base_link subscribe_scan:=true depth:=true approx_sync:=true odom_topic:=/odom scan_topic:=/scan qos:=2 args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1" use_sim_time:=true rviz:=true
```

"frame_id" --> base_link (element from the drone model)

Drone model:

https://github.com/ID-Robots/skyhub_server/blob/main/ardupilot_gazebo_models/iris_with_standoffs/model.sdf

base_link
```
<sdf version='1.9'>
  <model name='iris_with_standoffs'>
    <pose>0 0 0.194923 0 0 0</pose>
    <link name='base_link'>
      <velocity_decay>
        <linear>0.0</linear>
        <angular>0.0</angular>
      </velocity_decay>
```

...

Let's run the command and see the nodes and topics. The drone simulation is running at the same time, but it doesn't have a depth camera included yet.
That's what we found: /rgbd_camera/depth_image

```
ros2 node list

WARNING: Be aware that are nodes in the graph that share an exact name, this can have unintended side effects.
/aruco_land
/dashboard
/demo_simulation
/drone_control
/drone_control
/drone_control
/drone_control_web_app
/global_position_service
/point_cloud_xyzrgb
/ros_gz_bridge
/ros_gz_bridge
/rqt_gui_cpp_node_666302
/rtabmap/rtabmap
/rtabmap/rtabmap_viz
/rtabmap/transform_listener_impl_5560d15643b0
/rtabmap/transform_listener_impl_55ce41abb3e0
/rviz
/transform_listener_impl_5555704faaf0
/web_video_server

```

We notice a warning about the nodes with the same time which are sampled separately by logic. In the future they may be separated with namespaces or different names

```
ros2 topic list

/bottom_camera
/camera/depth_registered/image_raw
/camera/rgb/camera_info
/camera/rgb/image_rect_color
/clicked_point
/clock
/detection_camera
/disparity
/front_camera
/goal_pose
/gps/fix
/imu/data
/initialpose
/mavros/global_position/global
/mavros/local_position/pose
/mavros/setpoint_position/global
/mavros/setpoint_raw/local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/state
/odom
/parameter_events
/rgbd_camera/depth_image
/rgbd_image_relay
/rosout
/rtabmap/cloud_ground
/rtabmap/cloud_map
/rtabmap/cloud_obstacles
/rtabmap/global_path
/rtabmap/global_path_nodes
/rtabmap/global_pose
/rtabmap/goal
/rtabmap/goal_node
/rtabmap/goal_out
/rtabmap/goal_reached
/rtabmap/grid_prob_map
/rtabmap/info
/rtabmap/initialpose
/rtabmap/labels
/rtabmap/landmarks
/rtabmap/local_grid_empty
/rtabmap/local_grid_ground
/rtabmap/local_grid_obstacle
/rtabmap/local_path
/rtabmap/local_path_nodes
/rtabmap/localization_pose
/rtabmap/map
/rtabmap/mapData
/rtabmap/mapGraph
/rtabmap/mapOdomCache
/rtabmap/mapPath
/rtabmap/map_updates
/rtabmap/octomap_binary
/rtabmap/octomap_empty_space
/rtabmap/octomap_full
/rtabmap/octomap_global_frontier_space
/rtabmap/octomap_grid
/rtabmap/octomap_ground
/rtabmap/octomap_obstacles
/rtabmap/octomap_occupied_space
/rtabmap/odom_local_map
/rtabmap/republish_node_data
/scan
/stereo_camera/left/camera_info
/stereo_camera/left/image_rect_color
/stereo_camera/right/camera_info
/stereo_camera/right/image_rect
/tf
/tf_static
/user_data_async
/voxel_cloud
skyhub@skyhub-serv
/rtabmap/cloud_ground

/rtabmap/cloud_map

/rtabmap/cloud_obstacles

/rtabmap/global_path

/rtabmap/global_path_nodes

/rtabmap/global_pose

/rtabmap/goal

/rtabmap/goal_node

/rtabmap/goal_out

/rtabmap/goal_reached

/rtabmap/grid_prob_map

/rtabmap/info

/rtabmap/initialpose

/rtabmap/labels

/rtabmap/landmarks

/rtabmap/local_grid_empty

/rtabmap/local_grid_ground

/rtabmap/local_grid_obstacle

/rtabmap/local_path

/rtabmap/local_path_nodes

/rtabmap/localization_pose

/rtabmap/map

/rtabmap/mapData

/rtabmap/mapGraph

/rtabmap/mapOdomCache

/rtabmap/mapPath

/rtabmap/map_updates

/rtabmap/octomap_binary

/rtabmap/octomap_empty_space

/rtabmap/octomap_full

/rtabmap/octomap_global_frontier_space

/rtabmap/octomap_grid

/rtabmap/octomap_ground

/rtabmap/octomap_obstacles

/rtabmap/octomap_occupied_space

/rtabmap/odom_local_map

/rtabmap/republish_node_data
```

....

Let's sample some of the topics:

```
ros2 topic info --verbose /rgbd_camera/depth_image
Type: sensor_msgs/msg/Image

Publisher count: 0

Subscription count: 1

Node name: rqt_gui_cpp_node_666302
Node namespace: /
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
GID: 01.0f.10.96.be.2a.65.c1.01.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

ros2 topic info --verbose /camera/depth_registered/image_raw

Type: sensor_msgs/msg/Image

Publisher count: 0

Subscription count: 3

Node name: rtabmap
Node namespace: /rtabmap
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
GID: 01.0f.10.96.e7.38.59.9d.01.00.00.00.00.00.7d.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: rtabmap_viz
Node namespace: /rtabmap
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
GID: 01.0f.10.96.e9.38.86.ee.01.00.00.00.00.00.3a.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: point_cloud_xyzrgb
Node namespace: /
Topic type: sensor_msgs/msg/Image
Endpoint type: SUBSCRIPTION
GID: 01.0f.10.96.ed.38.49.a1.01.00.00.00.00.00.16.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

ros2 topic info --verbose /rtabmap/goal

Type: geometry_msgs/msg/PoseStamped

Publisher count: 0

Subscription count: 1

Node name: rtabmap
Node namespace: /rtabmap
Topic type: geometry_msgs/msg/PoseStamped
Endpoint type: SUBSCRIPTION
GID: 01.0f.10.96.e7.38.59.9d.01.00.00.00.00.00.32.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite


ros2 topic info --verbose /rtabmap/map

Type: nav_msgs/msg/OccupancyGrid

Publisher count: 1

Node name: rtabmap
Node namespace: /rtabmap
Topic type: nav_msgs/msg/OccupancyGrid
Endpoint type: PUBLISHER
GID: 01.0f.10.96.e7.38.59.9d.01.00.00.00.00.00.13.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: TRANSIENT_LOCAL
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: rviz
Node namespace: /
Topic type: nav_msgs/msg/OccupancyGrid
Endpoint type: SUBSCRIPTION
GID: 01.0f.10.96.eb.38.92.37.01.00.00.00.00.00.2d.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
...

```

So far as far as I find the occupancy grid is 2D? i.e. it is not a voxel grid and is perhpas intended for ground vehicles.

https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html

````
```
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.
# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data

Compact Message Definition
std_msgs/Header header
nav_msgs/MapMetaData info
int8[] data
Probably we need the set of point cloud data, note that there's an "obstacle" topic and ground.
/rtabmap/cloud_ground
/rtabmap/cloud_map
/rtabmap/cloud_obstacles
ros2 topic info --verbose /rtabmap/cloud_ground
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: rtabmap
Node namespace: /rtabmap
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.10.96.e7.38.59.9d.01.00.00.00.00.00.17.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: TRANSIENT_LOCAL
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0

skyhub@skyhub-server:~/skyhub_server$ ros2 topic info --verbose /rtabmap/cloud_map
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: rtabmap
Node namespace: /rtabmap
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.10.96.e7.38.59.9d.01.00.00.00.00.00.15.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: TRANSIENT_LOCAL
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
skyhub@skyhub-server:~/skyhub_server$ ros2 topic info --verbose /rtabmap/cloud_obstacles
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: rtabmap
Node namespace: /rtabmap
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.10.96.e7.38.59.9d.01.00.00.00.00.00.16.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: TRANSIENT_LOCAL
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
sensor_msgs/msg/PointCloud2
https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html

# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```
````


### How to convert it into more readable format of coordinates?

https://github.com/ros2/common_interfaces/blob/master/sensor_msgs_py/sensor_msgs_py/point_cloud2.py#L61

read_points() --> numpy array

etc.

```
def read_points(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_cloud: bool = False) -> np.ndarray:
```

https://answers.ros.org/question/398326/pointcloud2-parse-to-xyz-array-in-ros2/

In C++

```
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    const size_t number_of_points = msg->height * msg->width;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "z");
    for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z)
    {
        double x = *iter_x;
        double y = *iter_y;
        double z = *iter_z;
    }
    
}
```

Also (but ROS1 Noetic): https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h

Another low level example including Python: https://medium.com/@tonyjacob_/pointcloud2-message-explained-853bd9907743

...

It may be reasonable to use C++ when developing the obstacle avoidance and navigation for the computer of the drone. Or see how many points we could process anyway.

...

ROS2 Point Cloud Clustering and Cylindrical Segmentation: https://www.youtube.com/watch?v=-lkDWwN6Vog

https://github.com/noshluk2/ros2_learners

Point cloud processing, clustering library (BSD, C++): https://pointclouds.org/

```
ros2 point_cloud_perception clustering
```

```
pcl_viewer voxel.pcd
```

Rtab also has a GUI for displaying different views of the collected data.
A side note:

Regarding processing the Depth image from D435, we may use realsense helper functions as demonstrated here:

https://github.com/IntelRealSense/realsense-ros/issues/1342#issuecomment-681172015


## 4. Running Rtabmap examples ...

Rtabmap examples ...

maybe expecting at: /camera/depth_registered/image_raw

Where is he rtabmap launch file (complex logic for various functions): /opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py

Documentation:

https://wiki.ros.org/rtabmap_slam

https://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot

...

An example to be run with D435i camera:

https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_stereo.launch.py

introlab/rtabmap_ros#866

````rtabmap requires depth images to have format mono 16 in mm, or 32 float in meters.```

Does the depth image from the simulated camera is in meters? I guess so. (it was 32FC1 from sensors_demo, but is it in meters? To do: check from a simulation)

An example of Autonomous Navigation of a rover which is going to several Goal points and back to home (in the example - it doesn't correctly recognize the home location)
It has both 3D map, while it is explained as for 2D but for similar navigation tasks for a drone which is flying at low height near objects, trees etc. it actually also may work with similar 2D occupancy map.

Rtabmap recognizes the ground plane, ground point cloud.

Various "costmap" params regarding the way the robot explores.

costmap_common_params.yaml: local_costmap ... base_local_planner_params.yaml:

TrajectoryPlannerROS: ...

https://wiki.ros.org/rtabmap_ros/Tutorials/StereoOutdoorNavigation

A Tutorial for ROS1 Noetic and D435i: (3/2022)

https://admantium.medium.com/ros-simultaneous-mapping-and-localization-with-rtabmap-e527e6a6716

The official forum of RTabmap:

http://official-rtab-map-forum.206.s1.nabble.com/

## 5. ROS1 Noetic rosbag examples

Trying them out in VBox Ub.20.04 and studying the dislay, indication, topics, parameters etc.

http://wiki.ros.org/rtabmap_ros

Useful commands: http://wiki.ros.org/rostopic

rostopic info /voxel_cloud
rostopic echo /voxel_cloud
rostopic find <msg-type>
..

RViz example - voxel clouds doesn't show, otherwise it works; it seems (and some discussions) possibly some minor version incompatibilities, some mention "//map" instead of "map" (but it doesn't help).

The RTM displays work as expected.

![image](https://github.com/Twenkid/SLAM/assets/23367640/c1a95831-c7cf-48e7-a222-53d054362472)

![image](https://github.com/Twenkid/SLAM/assets/23367640/0d1a532e-019c-42c8-b2e9-02b5f47d1756)

...

**Continues in Part 3: rtabmap3.md ...**
