# MuJoCo Mobile Manipulator Simulation Demo - Design Spec

> Date: 2026-05-25
> Robot: AgileX Ranger + Dobot CR10 + AG95 Gripper
> Goal: Standalone MuJoCo demo with SLAM, navigation, whole-body control, object detection, pick-and-place

---

## 1. Overview

A self-contained Python demo that simulates the Ranger mobile manipulator in MuJoCo, demonstrating:

1. **3D LiDAR + RGB-D sensor simulation** in a home environment
2. **Mapping sweep**: robot navigates room boundary, builds occupancy grid + scene graph
3. **A* + VFH navigation** with obstacle avoidance
4. **Whole-body IK optimizer** (9-DOF: 3 base + 6 arm)
5. **Grounding-SAM2** object detection and 6D pose estimation
6. **Pick-and-place task** execution
7. **Dual visualization**: MuJoCo 3D viewer + Matplotlib 2D trajectory

### Architecture

```
mujoco_demo/
├── sim/              # MuJoCo engine: model loading, physics, sensor simulation
│   ├── engine.py     # MuJoCo wrapper (load URDF, step, render)
│   ├── sensors.py    # LiDAR + RGB-D camera simulation
│   └── env.py        # Home environment scene definition
├── slam/             # SLAM module
│   ├── occupancy.py  # 3D occupancy grid (log-odds)
│   ├── mapper.py     # Incremental mapping from LiDAR scans
│   └── scene_graph.py# Object pose graph
├── nav/              # Navigation
│   ├── astar.py      # A* global planner on occupancy grid
│   ├── vfh.py        # VFH+ local obstacle avoidance
│   └── planner.py    # Path following + waypoint tracking
├── control/          # Whole-body controller
│   ├── kinematics.py # Forward/inverse kinematics (base + arm)
│   ├── whole_body.py # QP-based whole-body IK optimizer
│   └── gripper.py    # Gripper open/close control
├── perception/       # Object detection
│   ├── detector.py   # Grounding-SAM2 wrapper
│   └── pose_est.py   # Depth-based 6D pose estimation
├── task/             # Task orchestration
│   ├── state_machine.py  # FSM: INIT→MAPPING→NAV→PICK→PLACE
│   └── task_parser.py    # NL task parsing (regex-based: "pick up X and place it on Y")
├── viz/              # Visualization
│   ├── mujoco_viz.py # MuJoCo 3D viewer with overlays
│   ├── matplotlib_viz.py # 2D trajectory + map plot
│   └── dashboard.py  # Dual-window dashboard
├── config.py         # All configuration constants
├── utils.py          # Math utilities, transforms
└── main.py           # Entry point
```

---

## 2. Robot Model

### 2.1 URDF Import

Source: `rangerboxcr10lidar_description/urdf/RangerCR10LiDAR.urdf`

MuJoCo loads URDF directly. Key handling:

| Component | URDF Joint Type | MuJoCo Type | Notes |
|-----------|----------------|-------------|-------|
| Ranger steering (x4) | revolute | hinge | Limits: [-1.57, 1.57] rad |
| Ranger wheels (x4) | continuous | hinge | No limits |
| CR10 joints 1-6 | revolute | hinge | See joint limits below |
| AG95 finger1 | revolute | hinge | Actuated, range [0, 0.6524] |
| AG95 finger2 + mimics | mimic | equality constraint | Sync via MuJoCo constraint |
| D435/D455/LiDAR | fixed | weld | Camera frames |
| Sensors | fixed | weld | No actuation |

**CR10 Joint Limits:**

| Joint | Lower (rad) | Upper (rad) |
|-------|------------|------------|
| joint1 | -3.92 | 0.94 |
| joint2 | -1.57 | 1.57 |
| joint3 | -2.86 | 2.86 |
| joint4 | -3.14 | 3.14 |
| joint5 | -3.14 | 3.14 |
| joint6 | -3.14 | 3.14 |

**AG95 Mimic Joint Fix:**

MuJoCo does not parse URDF `<mimic>` tags. After loading URDF, programmatically add MJCF equality constraints:

```xml
<equality>
  <joint joint1="gripper_finger1_joint" joint2="gripper_finger2_joint" polycoef="0 1 0 0 0"/>
  <joint joint1="gripper_finger1_joint" joint2="gripper_finger1_inner_knuckle_joint" polycoef="0 1.4946 0 0 0"/>
  <joint joint1="gripper_finger1_joint" joint2="gripper_finger2_inner_knuckle_joint" polycoef="0 1.4946 0 0 0"/>
  <!-- ... other mimic joints ... -->
</equality>
```

### 2.2 Mesh Files

All STL meshes from `rangerboxcr10lidar_description/meshes/` are loaded via URDF `package://` references. MuJoCo resolves these relative to the URDF location. If MuJoCo cannot resolve `package://`, convert to relative paths in a patched URDF copy.

### 2.3 Actuators

Add velocity-controlled actuators for all active joints:

```xml
<actuator>
  <!-- Ranger base: differential drive -->
  <velocity joint="fl_steering_wheel_joint" name="fl_steer"/>
  <velocity joint="fr_steering_joint" name="fr_steer"/>
  <velocity joint="rl_steering_wheel_joint" name="rl_steer"/>
  <velocity joint="rr_steering_wheel_joint" name="rr_steer"/>
  <velocity joint="fl_wheel_joint" name="fl_drive"/>
  <velocity joint="fr_wheel_joint" name="fr_drive"/>
  <velocity joint="rl_wheel_joint" name="rl_drive"/>
  <velocity joint="rr_wheel_joint" name="rr_drive"/>
  <!-- CR10 arm -->
  <position joint="cr10_joint1" name="arm_j1"/>
  <position joint="cr10_joint2" name="arm_j2"/>
  <position joint="cr10_joint3" name="arm_j3"/>
  <position joint="cr10_joint4" name="arm_j4"/>
  <position joint="cr10_joint5" name="arm_j5"/>
  <position joint="cr10_joint6" name="arm_j6"/>
  <!-- AG95 gripper -->
  <position joint="gripper_finger1_joint" name="gripper"/>
</actuator>
```

---

## 3. Simulation Environment

### 3.1 Home Scene

A simple room with furniture, created via MuJoCo XML:

```xml
<worldbody>
  <!-- Floor -->
  <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.85 1"/>
  <!-- Walls -->
  <geom type="box" pos="5 0 1" size="0.1 5 1" rgba="0.8 0.8 0.8 1"/>
  <geom type="box" pos="-5 0 1" size="0.1 5 1" rgba="0.8 0.8 0.8 1"/>
  <geom type="box" pos="0 5 1" size="5 0.1 1" rgba="0.8 0.8 0.8 1"/>
  <geom type="box" pos="0 -5 1" size="5 0.1 1" rgba="0.8 0.8 0.8 1"/>
  <!-- Table -->
  <body name="table" pos="2 1 0">
    <geom type="box" pos="0 0 0.4" size="0.6 0.4 0.02" rgba="0.6 0.4 0.2 1"/>
    <geom type="cylinder" pos="-0.5 -0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
    <geom type="cylinder" pos="0.5 -0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
    <geom type="cylinder" pos="-0.5 0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
    <geom type="cylinder" pos="0.5 0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
  </body>
  <!-- Shelf -->
  <body name="shelf" pos="-2 0 0">
    <geom type="box" pos="0 0 0.5" size="0.3 0.8 0.02" rgba="0.4 0.4 0.4 1"/>
    <geom type="box" pos="0 0 1.0" size="0.3 0.8 0.02" rgba="0.4 0.4 0.4 1"/>
  </body>
  <!-- Graspable objects -->
  <body name="red_cup" pos="2 1 0.45">
    <joint type="free" name="red_cup_joint"/>
    <geom type="cylinder" size="0.03 0.06" rgba="1 0 0 1" mass="0.1"/>
  </body>
  <body name="blue_cube" pos="2.2 0.8 0.45">
    <joint type="free" name="blue_cube_joint"/>
    <geom type="box" size="0.03 0.03 0.03" rgba="0 0 1 1" mass="0.15"/>
  </body>
</worldbody>
```

### 3.2 Camera Configuration

Two cameras in MuJoCo:

```xml
<camera name="d435_cam" pos="0.61 0 0.22" xyaxes="0 -1 0 0 0 1" fovy="57"/>
<camera name="d455_cam" pos="0 0 0" mode="fixed" />  <!-- Attached to gripper_base_link -->
```

- **D435**: Fixed to base_link, 57-degree FOV, 640x480
- **D455**: Fixed to gripper_base_link (follows arm), same resolution

---

## 4. Sensor Simulation

### 4.1 3D LiDAR (RoboSense 16-line)

**Parameters:**
- Lines: 16
- Vertical range: -15 to +15 degrees (2-degree spacing)
- Horizontal FOV: 360 degrees
- Range: 0.1 to 20 meters
- Scan rate: ~10 Hz (1 full rotation per 100ms)

**Implementation:**

```python
class LiDARSensor:
    def __init__(self, model, data):
        self.n_lines = 16
        self.v_angles = np.linspace(-15, 15, self.n_lines) * np.pi / 180
        self.h_resolution = 360  # rays per scan
        self.max_range = 20.0

    def scan(self, model, data):
        """Cast rays and return point cloud."""
        points = []
        lidar_pos = data.sensor('lidar_pos').data  # world frame
        lidar_mat = data.sensor('lidar_mat').data.reshape(3, 3)

        for v_angle in self.v_angles:
            for h_angle in np.linspace(0, 2*np.pi, self.h_resolution, endpoint=False):
                direction = self._ray_direction(v_angle, h_angle, lidar_mat)
                geom_id, distance = mj_ray(model, data, lidar_pos, direction)
                if distance < self.max_range:
                    hit_point = lidar_pos + direction * distance
                    points.append(hit_point)

        return np.array(points)  # Nx3
```

### 4.2 RGB-D Camera

**Parameters:**
- Resolution: 640 x 480
- FOV: 57 degrees (D435), 87 degrees (D455)
- Depth range: 0.1 to 10 meters

**Implementation:**

```python
class RGBDCamera:
    def __init__(self, cam_name, width=640, height=480):
        self.cam_name = cam_name
        self.width = width
        self.height = height

    def capture(self, model, data, scene, context):
        """Render RGB + depth from camera pose."""
        # Get camera pose
        cam_id = mj_name2id(model, mjOBJ_CAMERA, self.cam_name)
        cam_pos = model.cam_pos[cam_id]
        cam_mat = model.cam_mat0[cam_id].reshape(3, 3)

        # Set up rendering
        cam = mjvCamera()
        cam.type = mjCAMERA_FIXED
        cam.fixedcamid = cam_id

        # Render
        mjv_updateScene(model, data, opt, pert, cam, mjCAT_ALL, scene)
        mjr_render(viewport, scene, context)

        # Read pixels
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
        depth = np.zeros((height, width), dtype=np.float32)
        mjr_readPixels(rgb, depth, viewport, context)

        # Convert depth buffer to meters
        depth_meters = self._depth_to_meters(depth, model)

        return rgb, depth_meters, cam_pos, cam_mat
```

---

## 5. SLAM Module

### 5.1 Occupancy Grid

3D log-odds occupancy grid with 2D projection for navigation.

```python
class OccupancyGrid:
    def __init__(self, resolution=0.05, bounds=(-5, -5, 0, 5, 5, 3)):
        self.resolution = resolution  # 5cm
        self.bounds = bounds  # (x_min, y_min, z_min, x_max, y_max, z_max)
        self.grid = np.zeros(self._grid_shape())  # log-odds
        self.l_occ = 0.85   # log-odds for occupied
        self.l_free = -0.4   # log-odds for free

    def update(self, sensor_origin, points):
        """Update grid with new LiDAR observation."""
        for point in points:
            # Ray from origin to point: mark cells as free
            self._mark_ray_free(sensor_origin, point)
            # Mark hit cell as occupied
            self._mark_occupied(point)

    def get_2d_map(self, z_min=0.1, z_max=1.5):
        """Project 3D grid to 2D for navigation."""
        z_idx_min = int((z_min - self.bounds[2]) / self.resolution)
        z_idx_max = int((z_max - self.bounds[2]) / self.resolution)
        return np.max(self.grid[:, :, z_idx_min:z_idx_max], axis=2) > 0
```

### 5.2 Scene Graph

Tracks detected objects and their spatial relationships.

```python
class SceneGraphNode:
    id: str              # e.g., "red_cup_1"
    obj_type: str        # "graspable", "furniture", "obstacle"
    pose: np.ndarray     # 6D pose (x, y, z, roll, pitch, yaw)
    size: np.ndarray     # bounding box (w, h, d)
    confidence: float    # detection confidence
    parent: str          # e.g., "table_1" (if on table)

class SceneGraph:
    nodes: Dict[str, SceneGraphNode]
    edges: List[Tuple[str, str, str]]  # (from, to, relation)
```

### 5.3 Mapping Sweep

Pre-planned path around room boundary:

```python
def generate_sweep_waypoints(room_bounds, step=1.0):
    """Generate clockwise waypoints around room perimeter."""
    x_min, y_min, x_max, y_max = room_bounds
    waypoints = []
    # Bottom edge
    for x in np.arange(x_min + step, x_max, step):
        waypoints.append((x, y_min + step, 0))
    # Right edge
    for y in np.arange(y_min + step, y_max, step):
        waypoints.append((x_max - step, y, -np.pi/2))
    # Top edge
    for x in np.arange(x_max - step, x_min, -step):
        waypoints.append((x, y_max - step, np.pi))
    # Left edge
    for y in np.arange(y_max - step, y_min, -step):
        waypoints.append((x_min + step, y, np.pi/2))
    return waypoints
```

---

## 6. Navigation Module

### 6.1 A* Global Planner

Standard A* on the 2D occupancy grid.

```python
class AStarPlanner:
    def __init__(self, grid_2d, resolution=0.05):
        self.grid = grid_2d
        self.resolution = resolution

    def plan(self, start, goal):
        """Returns list of (x, y) waypoints."""
        # Convert world coords to grid coords
        start_grid = self._world_to_grid(start)
        goal_grid = self._world_to_grid(goal)

        # A* with 8-connected neighbors
        open_set = PriorityQueue()
        open_set.put((0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}

        while not open_set.empty():
            current = open_set.get()[1]
            if current == goal_grid:
                return self._reconstruct_path(came_from, current)

            for neighbor in self._get_neighbors(current):
                tentative_g = g_score[current] + self._distance(current, neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal_grid)
                    open_set.put((f_score, neighbor))

        return None  # No path found
```

### 6.2 VFH+ Local Avoidance

Polar histogram based obstacle avoidance.

```python
class VFHPlanner:
    def __init__(self, threshold=0.3, robot_radius=0.4):
        self.threshold = threshold
        self.robot_radius = robot_radius

    def compute_velocity(self, current_pos, current_heading, goal, lidar_scan):
        """Compute (v, omega) to follow global path while avoiding obstacles."""
        # Build polar histogram
        histogram = self._build_histogram(lidar_scan)

        # Find candidate valleys (gaps in histogram)
        valleys = self._find_valleys(histogram)

        # Select best direction (closest to goal direction)
        goal_direction = np.arctan2(goal[1]-current_pos[1], goal[0]-current_pos[0])
        best_direction = self._select_direction(valleys, goal_direction)

        # Compute velocity commands
        direction_error = best_direction - current_heading
        v = 0.3 * np.exp(-abs(direction_error))  # Slow down when turning
        omega = 2.0 * direction_error

        return v, omega
```

---

## 7. Whole-Body Controller

### 7.1 Kinematics

The robot has 9 DOF for manipulation: 3 base (x, y, theta) + 6 arm joints.

```python
class WholeBodyKinematics:
    def __init__(self, model, data):
        self.n_base = 3   # x, y, theta
        self.n_arm = 6    # cr10_joint1..6
        self.n_total = 9

    def forward_kinematics(self, q):
        """Compute end-effector pose from joint configuration."""
        # Base transform
        T_base = self._base_transform(q[:3])
        # Arm FK using MuJoCo
        T_ee = self._arm_fk(q[3:])
        return T_base @ T_ee

    def jacobian(self, q):
        """Compute 6x9 Jacobian (6D ee velocity, 9 joints)."""
        # Numerical Jacobian
        J = np.zeros((6, self.n_total))
        eps = 1e-6
        T0 = self.forward_kinematics(q)
        for i in range(self.n_total):
            q_plus = q.copy()
            q_plus[i] += eps
            T_plus = self.forward_kinematics(q_plus)
            J[:, i] = self._pose_diff(T_plus, T0) / eps
        return J
```

### 7.2 QP-Based Whole-Body IK

```python
class WholeBodyController:
    def __init__(self, kinematics, model, data):
        self.kin = kinematics
        self.model = model
        self.data = data
        self.qp_weight_tracking = 1.0
        self.qp_weight_regularization = 0.01
        self.qp_weight_collision = 0.5

    def compute_joint_velocities(self, q_current, target_pose, obstacles):
        """
        Solve QP:
            min  ||J*dq - dx||^2 + lambda1*||dq||^2 + lambda2*d_collision
            s.t. q_min <= q + dq <= q_max
                 dq_min <= dq <= dq_max
        """
        J = self.kin.jacobian(q_current)
        dx = self._pose_error(target_pose, self.kin.forward_kinematics(q_current))

        # Cost: tracking error
        H = J.T @ J + self.qp_weight_regularization * np.eye(9)
        f = -J.T @ dx

        # Collision avoidance term
        if obstacles:
            d_col, grad_col = self._collision_gradient(q_current, obstacles)
            if d_col < 0.5:  # Within safety margin
                H += self.qp_weight_collision * np.outer(grad_col, grad_col)
                f -= self.qp_weight_collision * d_col * grad_col

        # Constraints
        q_min, q_max = self._get_joint_limits()
        dq_max = self._get_velocity_limits()

        # Solve with scipy
        from scipy.optimize import minimize
        result = minimize(
            lambda dq: 0.5 * dq @ H @ dq + f @ dq,
            np.zeros(9),
            bounds=list(zip(-dq_max, dq_max)),
            method='SLSQP'
        )

        return result.x
```

### 7.3 Operating Modes

| Mode | Base | Arm | Use Case |
|------|------|-----|----------|
| NAVIGATION | Active (path tracking) | Locked (safe pose) | Moving to target |
| MANIPULATION | Locked (precise) | Active (IK control) | Pick/place |
| WHOLE_BODY | Active | Active | Coordinated motion |

---

## 8. Perception Module

### 8.1 Grounding-SAM2 Integration

```python
class ObjectDetector:
    def __init__(self, model_name="facebook/sam2-hiera-tiny"):
        # Grounding-SAM2 = Grounding DINO (detection) + SAM2 (segmentation)
        # Option 1: Use pipeline API from transformers
        # Option 2: Use separate Grounding DINO + SAM2
        from transformers import pipeline
        self.detector = pipeline("zero-shot-object-detection", model="IDEA-Research/grounding-dino-tiny")
        # SAM2 for segmentation refinement
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        sam2_model = build_sam2("sam2_hiera_tiny", checkpoint="sam2_hiera_tiny.pt")
        self.segmentor = SAM2ImagePredictor(sam2_model)

    def detect(self, rgb_image, text_prompt):
        """Detect objects matching text prompt."""
        inputs = self.processor(images=rgb_image, text=text_prompt, return_tensors="pt")
        outputs = self.model(**inputs)
        results = self.processor.post_process_grounded_object_detection(
            outputs, inputs["input_ids"],
            box_threshold=0.3,
            text_threshold=0.25,
            target_sizes=[rgb_image.shape[:2]]
        )
        return results[0]  # boxes, scores, labels
```

### 8.2 6D Pose Estimation

```python
class PoseEstimator:
    def __init__(self, camera_intrinsics):
        self.K = camera_intrinsics  # 3x3 intrinsic matrix

    def estimate(self, bbox, mask, depth_image, camera_pose):
        """Estimate 6D object pose from detection + depth."""
        # 1. Get median depth within mask
        depth_roi = depth_image[mask]
        depth_roi = depth_roi[depth_roi > 0]  # Filter invalid
        median_depth = np.median(depth_roi)

        # 2. Back-project to camera frame
        u, v = bbox.center
        x = (u - self.K[0,2]) * median_depth / self.K[0,0]
        y = (v - self.K[1,2]) * median_depth / self.K[1,1]
        z = median_depth
        point_cam = np.array([x, y, z, 1])

        # 3. Transform to world frame
        T_world_cam = camera_pose  # 4x4
        point_world = T_world_cam @ point_cam

        # 4. Estimate orientation (PCA on point cloud)
        orientation = self._estimate_orientation(mask, depth_image, camera_pose)

        return Pose6D(point_world[:3], orientation)
```

---

## 9. Task Orchestration

### 9.1 State Machine

```python
class TaskState(Enum):
    INIT = "init"
    MAPPING_SWEEP = "mapping_sweep"
    TASK_RECEIVED = "task_received"
    NAV_TO_OBJECT = "nav_to_object"
    APPROACH_OBJECT = "approach_object"
    PICK_OBJECT = "pick_object"
    NAV_TO_PLACE = "nav_to_place"
    PLACE_OBJECT = "place_object"
    COMPLETE = "complete"

class TaskStateMachine:
    def __init__(self, sim, slam, nav, control, perception, viz):
        self.state = TaskState.INIT
        self.sim = sim
        self.slam = slam
        self.nav = nav
        self.control = control
        self.perception = perception
        self.viz = viz

    def step(self):
        if self.state == TaskState.INIT:
            self._init_sim()
            self.state = TaskState.MAPPING_SWEEP

        elif self.state == TaskState.MAPPING_SWEEP:
            self._run_mapping_sweep()
            self.state = TaskState.TASK_RECEIVED

        elif self.state == TaskState.TASK_RECEIVED:
            task = self._parse_task("Pick up the red cup and place it on the shelf")
            self.target_object = task['object']
            self.target_location = task['location']
            self.state = TaskState.NAV_TO_OBJECT

        elif self.state == TaskState.NAV_TO_OBJECT:
            if self._navigate_to(self.target_object.pose[:2]):
                self.state = TaskState.APPROACH_OBJECT

        elif self.state == TaskState.APPROACH_OBJECT:
            if self._approach_and_grasp(self.target_object):
                self.state = TaskState.NAV_TO_PLACE

        elif self.state == TaskState.NAV_TO_PLACE:
            if self._navigate_to(self.target_location[:2]):
                self.state = TaskState.PLACE_OBJECT

        elif self.state == TaskState.PLACE_OBJECT:
            if self._place_object():
                self.state = TaskState.COMPLETE
```

---

## 10. Visualization

### 10.1 MuJoCo 3D Viewer

- Robot model with real-time joint animation
- LiDAR ray visualization (optional overlay)
- Planned path as line geometry
- Target markers
- Collision proximity visualization (color-coded)

### 10.2 Matplotlib 2D Dashboard

Real-time plot updated each simulation step:

```python
class MatplotlibDashboard:
    def __init__(self):
        self.fig, self.axes = plt.subplots(1, 2, figsize=(14, 6))

    def update(self, robot_pos, trajectory, grid_2d, path, objects):
        # Left: occupancy map + trajectory
        self.axes[0].clear()
        self.axes[0].imshow(grid_2d.T, origin='lower', cmap='binary')
        self.axes[0].plot(*zip(*trajectory), 'b-', linewidth=2)
        self.axes[0].plot(robot_pos[0], robot_pos[1], 'ro', markersize=8)
        for obj in objects:
            self.axes[0].plot(obj.x, obj.y, 'g^', markersize=10)
        if path:
            self.axes[0].plot(*zip(*path), 'r--', linewidth=1)
        self.axes[0].set_title('SLAM Map + Navigation')

        # Right: task progress
        self.axes[1].clear()
        self.axes[1].barh(range(len(self.states)), self.state_progress)
        self.axes[1].set_yticks(range(len(self.states)))
        self.axes[1].set_yticklabels(self.state_names)
        self.axes[1].set_title('Task Progress')

        plt.pause(0.01)
```

---

## 11. Configuration

```python
# config.py

# Simulation
SIM_TIMESTEP = 0.002  # 2ms physics step
SIM_DURATION = 300    # 5 min max

# LiDAR
LIDAR_N_LINES = 16
LIDAR_V_ANGLES = np.linspace(-15, 15, 16) * np.pi / 180
LIDAR_H_RAYS = 360
LIDAR_MAX_RANGE = 20.0
LIDAR_MOUNT_POS = [0.52588, 0, 0.16587]  # relative to base_link

# Camera
CAM_D435_WIDTH = 640
CAM_D435_HEIGHT = 480
CAM_D435_FOV = 57  # degrees
CAM_D455_WIDTH = 640
CAM_D455_HEIGHT = 480
CAM_D455_FOV = 87  # degrees

# SLAM
GRID_RESOLUTION = 0.05  # 5cm
GRID_BOUNDS = (-5, -5, 0, 5, 5, 3)  # x_min, y_min, z_min, x_max, y_max, z_max

# Navigation
NAV_LINEAR_SPEED = 0.3  # m/s
NAV_ANGULAR_SPEED = 0.5  # rad/s
NAV_GOAL_TOLERANCE = 0.2  # meters
NAV_OBSTACLE_DIST = 0.5  # safety margin

# Control
CONTROL_FREQUENCY = 100  # Hz
QP_WEIGHT_TRACKING = 1.0
QP_WEIGHT_REGULARIZATION = 0.01
QP_WEIGHT_COLLISION = 0.5

# Gripper
GRIPPER_OPEN = 0.6  # rad
GRIPPER_CLOSE = 0.1  # rad
GRIPPER_FORCE = 50  # N

# Task
TASK_OBJECTS = ["red cup", "blue cube", "green bottle"]
TASK_LOCATIONS = ["table", "shelf", "counter"]
```

---

## 12. Task Parser

Simple regex-based parser for task commands. No LLM dependency.

```python
import re

class TaskParser:
    PATTERNS = [
        # "pick up the red cup and place it on the shelf"
        r"pick up (?:the )?(.+?) and (?:place|put) (?:it|them) (?:on|in|at) (?:the )?(.+)",
        # "move the blue cube to the table"
        r"move (?:the )?(.+?) to (?:the )?(.+)",
        # "grasp the green bottle"
        r"(?:grasp|grab|pick up) (?:the )?(.+)",
    ]

    def parse(self, command: str) -> dict:
        command = command.lower().strip()
        for pattern in self.PATTERNS:
            match = re.match(pattern, command)
            if match:
                groups = match.groups()
                return {
                    'action': 'pick_and_place' if len(groups) == 2 else 'pick',
                    'object': groups[0].strip(),
                    'location': groups[1].strip() if len(groups) > 1 else None,
                }
        raise ValueError(f"Cannot parse task command: {command}")
```

---

## 13. Dependencies

```bash
pip install mujoco>=3.0 numpy scipy matplotlib opencv-python Pillow
pip install transformers torch torchvision  # Grounding DINO
pip install sam-2  # SAM2 segmentation (from https://github.com/facebookresearch/sam2)
```

MuJoCo version: >= 3.0 (native Python bindings, not mujoco-py)

---

## 14. Execution Flow

```
python main.py
```

1. MuJoCo window opens showing the home environment
2. Matplotlib dashboard opens alongside
3. Robot performs mapping sweep (~60 seconds)
4. Scene graph populated with detected objects
5. Task received: "Pick up the red cup and place it on the shelf"
6. Robot navigates to table, avoiding obstacles
7. Switches to D455 camera, detects cup precisely
8. Whole-body IK moves arm to pre-grasp pose
9. Gripper closes, cup attached
10. Navigates to shelf
11. Places cup on shelf
12. Task complete, trajectory saved

---

## 15. Risk Mitigation

| Risk | Mitigation |
|------|-----------|
| URDF mesh path resolution | Copy meshes to local dir, patch URDF paths |
| AG95 mimic joints in MuJoCo | Add equality constraints programmatically |
| Grounding-SAM2 GPU memory | Use tiny variant, fallback to simple detector |
| Real-time visualization lag | Decouple viz from physics (async rendering) |
| Whole-body IK convergence | Add null-space regularization, fallback to sequential |
| Camera rendering overhead | Render only when needed (not every physics step) |
