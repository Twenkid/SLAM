https://docs.ros.org/en/api/rtabmap_msgs/html/msg/MapGraph.html

Header header
```
##
# /map to /odom transform
# Always identity when the graph is optimized from the latest pose.
##
geometry_msgs/Transform mapToOdom

# The poses
int32[] posesId
geometry_msgs/Pose[] poses

# The links
Link[] links
```

https://docs.ros.org/en/api/rtabmap_msgs/html/msg/MapData.html

```
std_msgs/Header header
rtabmap_msgs/MapGraph graph
rtabmap_msgs/NodeData[] nodes
```

https://docs.ros.org/en/api/geometry_msgs/html/msg/Transform.html

```
geometry_msgs/Transform mapToOdom

geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation


geometry_msgs/Pose pose
# request
string base_frame
---
# response
geometry_msgs/Pose pose
https://index.ros.org/p/ros_gz_bridge/#humble
ros_gz_bridge package from ros_ign repo
```

```mermaid
stateDiagram-v2
[*] --> Still
Still --> [*]
Still --> Moving
Moving --> Still
Moving --> Crash
Crash --> [*]
```

```mermaid
flowchart LR

A[Hard] -->|Text| B(Round)
B --> C{Decision}
C -->|One| D[Result 1]
C -->|Two| E[Result 2]
``
