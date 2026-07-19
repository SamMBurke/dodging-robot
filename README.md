# Dodging Robot Project

## Brief Summary
A ROS2 package for TurtleBot4 that uses 2D LiDAR to detect and track nearby
objects, and then plans a path to avoid them if it detects a potential collision.

**Pipeline:** raw LiDAR scan → Euclidean clustering → L-shape rectangle
fitting → Kalman-filter tracking with Hungarian data association → tracked
objects with position, velocity, and dimensions.

The robot starts stationary. Once it detects an object, it checks whether
the object is on a collision path and plans around it accordingly.

---

## How it works

```
/scan (sensor_msgs/LaserScan)
   │
   ▼
LidarObjectDetectorNode
   │  1. convert_to_cartesian()      polar LiDAR ranges -> 3D points
   │  2. get_euclidean_clusters()    group points into clusters (Open3D KD-tree)
   │  3. fit_rectangle()             L-shape rectangle fit per cluster
   ▼
/detected_objects (project_interfaces/DetectedROSObjectArray)
   │
   ▼
ObjectTrackerNode
   │  1. transform detection corners into a fixed frame (tf2)
   │  2. KalmanTracker.step()        predict all tracks, associate via
   │                                 Hungarian algorithm, update matches,
   │                                 spawn/prune tracks
   ▼
/tracked_objects (project_interfaces/TrackedObjectArray)
   │
   ▼
(path planner)
```

Detections have no memory — every scan produces a fresh, unlabeled batch of
rectangles. Tracks are the tracker's persistent belief that a specific
physical object exists, with a stable ID, a Kalman-filtered position and
velocity, and a smoothed heading/size, stitched together across many
frames.

---

## Repo structure

```
dodging-robot/
└── src/
    ├── project_interfaces/          # custom message definitions (ament_cmake)
    │   ├── msg/
    │   │   ├── DetectedROSObject.msg
    │   │   ├── DetectedROSObjectArray.msg
    │   │   ├── TrackedObject.msg
    │   │   └── TrackedObjectArray.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── project_pkg/                 # library code + nodes (ament_python)
        ├── project_pkg/
        │   ├── math.py                       # clustering + rectangle fitting
        │   ├── objects.py                    # DetectedObject dataclass
        │   ├── conversions.py                # ROS <-> numpy corner conversions
        │   ├── kalman_tracker.py             # KalmanFilter, Track, KalmanTracker
        │   ├── object_detector.py            # LidarObjectDetectorNode
        │   ├── object_tracker_node.py        # ObjectTrackerNode
        │   ├── fake_detection_publisher.py   # synthetic /detected_objects for testing
        │   └── tracked_objects_visualizer.py # /tracked_objects -> RViz2 markers
        ├── setup.py
        └── package.xml
```

---

## Dependencies

**ROS2 (apt):**
- `ros-jazzy-tf2-ros`
- `ros-jazzy-tf2-geometry-msgs`
- (`geometry_msgs`, `std_msgs`, `visualization_msgs`, `sensor_msgs` come with a standard ROS2 install)

**Python (pip, not resolvable via rosdep):**
- `open3d` — used for KD-tree Euclidean clustering
- `scipy` — used for the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`)

```bash
pip install open3d scipy --break-system-packages
```

`--break-system-packages` is required on Ubuntu 24 / Python 3.12 for a
system-wide `pip install`. These two packages aren't always in the rosdep
database, so `rosdep install` may not pick them up — install manually if so.

---

## Setup from scratch

```bash
# 1. clone / place the repo
cd ~/dodging-robot

# 2. install ROS-known dependencies
rosdep install --from-paths src --ignore-src -r -y

# 3. install the two pip-only dependencies (see above)
pip install open3d scipy --break-system-packages

# 4. build
colcon build --symlink-install

# 5. source the workspace (needed in every new terminal)
source install/setup.bash
```

`--symlink-install` lets edits to Python files take effect without a full
rebuild. You still need to rebuild after: adding/renaming a file, changing
a `.msg` definition, or editing `setup.py`'s `entry_points`.

**Sanity check the build worked:**
```bash
ros2 pkg list | grep project_interfaces   # should print "project_interfaces"
ros2 interface show project_interfaces/msg/TrackedObject   # should print the message definition
```

---

## Messages

**`DetectedROSObject`** — one raw detection, in the LiDAR's own (local) frame:
```
int32 id
geometry_msgs/Point center
float32 heading
float32 width
float32 length
geometry_msgs/Point[] corners
```

**`TrackedObject`** — one tracked object, in the fixed frame, with identity and velocity:
```
int32 id
geometry_msgs/Point center
geometry_msgs/Vector3 velocity
float32 heading
float32 length
float32 width
geometry_msgs/Point[] corners
int32 age
int32 hits
```

`*Array` variants wrap each of these with a `std_msgs/Header header` plus a list.

Import path note: generated messages live under `<package>.msg`, e.g.
`from project_interfaces.msg import TrackedObject` — not
`from project_interfaces import TrackedObject`.

---

## Nodes

| Node | Executable | Subscribes | Publishes | Purpose |
|---|---|---|---|---|
| `LidarObjectDetectorNode` | `lidar_object_detector_node` | `/scan` | `/detected_objects` | Cluster + rectangle-fit each scan |
| `ObjectTrackerNode` | `object_tracker_node` | `/detected_objects` | `/tracked_objects` | Transform to fixed frame, Kalman-track |
| `TrackedObjectsVisualizer` | `tracked_objects_visualizer` | `/tracked_objects` | `/tracked_objects_markers` | Draws boxes/IDs/velocity arrows for RViz2 |

Run any of them with:
```bash
ros2 run project_pkg <executable>
```
