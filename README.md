# Dodging Robot Project

## Brief Summary
A ROS2 package for TurtleBot4 that uses 2D LiDAR to detect and track nearby
objects, then reactively plans around them if they're on a collision path.

**Pipeline:** raw LiDAR scan → Euclidean clustering → L-shape rectangle
fitting → Kalman-filter tracking with Hungarian data association → tracked
objects with position, velocity, and dimensions → reactive obstacle
avoidance → velocity commands.

The robot starts stationary. Once it detects a moving object on a
predicted collision course, it computes an evasion point and drives
toward it; otherwise it holds position.

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
/tracked_objects (project_interfaces/TrackedObjectArray, in 'odom' frame)
   │
   ▼
ObstacleAvoidanceNode (oa_node)
   │  1. select_worst_case_object()  convert each tracked object into the
   │                                 robot's local frame, gate by current
   │                                 distance + predicted closest approach
   │  2. generate candidate evasion points, score by distance/turn/time
   │  3. lock onto a target with hysteresis (avoids target-switching jitter)
   │  4. compute a smoothed velocity command toward the locked point
   ▼
/cmd_vel (geometry_msgs/Twist)
```

Detections have no memory — every scan produces a fresh, unlabeled batch of
rectangles. Tracks are the tracker's persistent belief that a specific
physical object exists, with a stable ID, a Kalman-filtered position and
velocity, and a smoothed heading/size, stitched together across many
frames. The obstacle avoidance node only reacts to *moving* tracked
objects; stationary ones are ignored by design.

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
        │   ├── oa_algorithm.py               # obstacle avoidance math (frame transforms,
        │   │                                 #   CPA gating, candidate generation, cost
        │   │                                 #   function, hysteresis lock, velocity command)
        │   ├── oa_node.py                    # ObstacleAvoidanceNode
        │   └── tracked_objects_visualizer.py # /tracked_objects -> RViz2 markers
        ├── launch/
        │   └── dodging_robot.launch.py       # starts the full pipeline + RViz2
        ├── rviz/
        │   └── *.rviz                        # pre-configured RViz2 display layout
        ├── setup.py
        └── package.xml
```

---

## Dependencies

**ROS2 (apt):**
- `ros-jazzy-tf2-ros`
- `ros-jazzy-tf2-geometry-msgs`
- `ros-jazzy-rviz2` (only needed if not already present in your ROS2 install)
- (`geometry_msgs`, `std_msgs`, `visualization_msgs`, `sensor_msgs`, `nav_msgs` come with a standard ROS2 install)

**Python (pip, not resolvable via rosdep):**
- `open3d` — used for KD-tree Euclidean clustering
- `scipy` — used for the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`)

```bash
pip install open3d scipy --break-system-packages
```

`--break-system-packages` is required on Ubuntu 24 / Python 3.12 for a
system-wide `pip install`. These two packages aren't always in the rosdep
database, so `rosdep install` may not pick them up — install manually if so.

`oa_node` itself has no extra dependencies beyond the standard ROS2
packages above — it consumes `/tracked_objects` and `/odom` directly, with
no additional `tf2` lookups of its own.

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

`oa_node` doesn't introduce any new custom message types — it consumes
`TrackedObjectArray` and standard `nav_msgs/Odometry`, and publishes
standard `geometry_msgs/Twist` and `visualization_msgs/MarkerArray`.

---

## Nodes

| Node | Executable | Subscribes | Publishes | Purpose |
|---|---|---|---|---|
| `LidarObjectDetectorNode` | `lidar_object_detector_node` | `/scan` | `/detected_objects` | Cluster + rectangle-fit each scan |
| `ObjectTrackerNode` | `object_tracker_node` | `/detected_objects` | `/tracked_objects` | Transform to fixed frame, Kalman-track |
| `ObstacleAvoidanceNode` | `oa_node` | `/tracked_objects`, `/odom` | `/cmd_vel`, `/oa_debug_markers` | Reactive collision avoidance |
| `TrackedObjectsVisualizer` | `tracked_objects_visualizer` | `/tracked_objects` | `/tracked_objects_markers` | Draws boxes/IDs/velocity arrows for RViz2 |

---

## Obstacle avoidance

`oa_node` reacts to the single most urgent tracked object each control
cycle (10Hz). For every object in `/tracked_objects` it:

1. Converts the object's position/velocity from the fixed `odom` frame
   into the robot's local frame, using `robot_pose` from `/odom` directly
   (no separate `tf2` lookup needed, since `/odom` already reports the
   robot's pose in the same `odom` frame `/tracked_objects` uses).
2. Ignores anything stationary, and anything farther than
   `engagement_radius` — only objects currently nearby are considered at
   all, regardless of what their predicted trajectory looks like.
3. Runs a closest-point-of-approach (CPA) check against the remaining
   objects; the one with the smallest predicted approach distance is
   treated as this cycle's threat.
4. Generates a grid of candidate evasion points around the robot, filters
   out ones that are unreachable in time or too close to the obstacle,
   and scores the rest on distance/turn angle/time-to-reach.
5. Locks onto a target with hysteresis (a new candidate only replaces the
   current lock if it's meaningfully better), to avoid rapidly switching
   targets.
6. Computes a smoothed linear/angular velocity command toward the locked
   point and publishes it on `/cmd_vel`.

**Key parameters** (in `oa_node.py`'s `__init__`):

| Parameter | Meaning |
|---|---|
| `R` | Contact zone — predictive CPA threat threshold |
| `engagement_radius` | Objects farther than this (current distance, not predicted) are ignored entirely |
| `robot_radius` | Robot's own footprint, added to clearance margins |
| `v_max`, `omega_max` | Robot's velocity limits — confirm these against Create3's actual spec before trusting on hardware |
| `w_dist`, `w_angle`, `w_time` | Candidate scoring weights |
| `rho` | Hysteresis threshold for switching the locked target |

**Debug visualization:** `/oa_debug_markers` (`visualization_msgs/MarkerArray`)
shows the currently locked evasion point and a line from the robot to it —
add a MarkerArray display on this topic in RViz2 alongside
`/tracked_objects_markers` to see what the node is planning in real time.

---

## Running everything

The launch file starts the full pipeline (detector, tracker, obstacle
avoidance, visualizer) plus a pre-configured RViz2 window:

```bash
cd ~/dodging-robot
colcon build --symlink-install
source install/setup.bash
ros2 launch project_pkg dodging_robot.launch.py
```

Assumes TurtleBot4 bringup (LiDAR driver, `odom → base_link` TF, etc.) is
already running and reachable on the robot. Confirm before launching:
```bash
ros2 topic list      # should already show /scan, /odom, /tf from the robot
```