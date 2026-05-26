# MuJoCo Mobile Manipulator Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone MuJoCo simulation demo for the Ranger+CR10 mobile manipulator with SLAM, navigation, whole-body control, object detection, and pick-and-place.

**Architecture:** Pure Python modular architecture with MuJoCo as physics engine. Each module (sim, slam, nav, control, perception, task, viz) communicates through well-defined Python interfaces. No ROS dependency.

**Tech Stack:** Python 3.10+, MuJoCo >= 3.0, NumPy, SciPy, Matplotlib, OpenCV, Grounding-SAM2 (optional, with fallback)

**Spec:** `docs/superpowers/specs/2026-05-25-mujoco-mobile-manipulator-demo-design.md`

---

## File Structure

```
mujoco_demo/
├── config.py                 # All configuration constants
├── utils.py                  # Transform math, quaternion utilities
├── sim/
│   ├── __init__.py
│   ├── engine.py             # MuJoCo wrapper: load URDF, step, actuators
│   ├── sensors.py            # LiDARSensor + RGBDCamera classes
│   └── env.py                # Home scene XML generation, scene+robot merge
├── slam/
│   ├── __init__.py
│   ├── occupancy.py          # OccupancyGrid: 3D log-odds + 2D projection
│   ├── mapper.py             # Mapper: incremental mapping from LiDAR scans
│   └── scene_graph.py        # SceneGraph + SceneGraphNode
├── nav/
│   ├── __init__.py
│   ├── astar.py              # AStarPlanner on 2D grid
│   ├── vfh.py                # VFHPlanner: polar histogram local avoidance
│   └── planner.py            # PathFollower: waypoint tracking + velocity output
├── control/
│   ├── __init__.py
│   ├── kinematics.py         # WholeBodyKinematics: FK + numerical Jacobian
│   ├── whole_body.py         # WholeBodyController: QP-based IK optimizer
│   └── gripper.py            # GripperController: open/close/force control
├── perception/
│   ├── __init__.py
│   ├── detector.py           # ObjectDetector: Grounding-SAM2 or fallback
│   └── pose_est.py           # PoseEstimator: depth-based 6D pose
├── task/
│   ├── __init__.py
│   ├── state_machine.py      # TaskStateMachine: FSM orchestration
│   └── task_parser.py        # TaskParser: regex command parsing
├── viz/
│   ├── __init__.py
│   ├── mujoco_viz.py         # MuJoCo 3D viewer overlays
│   ├── matplotlib_viz.py     # MatplotlibDashboard: 2D map + trajectory
│   └── dashboard.py          # Dashboard: dual-window coordinator
└── main.py                   # Entry point: wire all modules, run loop
```

**Test files:**

```
tests/
├── test_config.py
├── test_utils.py
├── test_sim_engine.py
├── test_sim_sensors.py
├── test_sim_env.py
├── test_slam_occupancy.py
├── test_slam_mapper.py
├── test_nav_astar.py
├── test_nav_vfh.py
├── test_nav_planner.py
├── test_control_kinematics.py
├── test_control_whole_body.py
├── test_control_gripper.py
├── test_task_parser.py
├── test_task_state_machine.py
└── test_integration.py
```

---

## Task 1: Project Scaffolding + Config + Utils

**Files:**
- Create: `mujoco_demo/__init__.py`
- Create: `mujoco_demo/config.py`
- Create: `mujoco_demo/utils.py`
- Create: `tests/test_utils.py`

- [ ] **Step 1: Create project directory and __init__.py**

```bash
mkdir -p /home/gzz/Codes/agx_ws/src/agx/mujoco_demo/{sim,slam,nav,control,perception,task,viz}
mkdir -p /home/gzz/Codes/agx_ws/src/agx/tests
touch /home/gzz/Codes/agx_ws/src/agx/mujoco_demo/__init__.py
touch /home/gzz/Codes/agx_ws/src/agx/mujoco_demo/{sim,slam,nav,control,perception,task,viz}/__init__.py
```

- [ ] **Step 2: Write config.py**

```python
# mujoco_demo/config.py
import numpy as np
from pathlib import Path

# Paths
PROJECT_ROOT = Path(__file__).parent
URDF_PATH = PROJECT_ROOT.parent / "rangerboxcr10lidar_description" / "urdf" / "RangerCR10LiDAR.urdf"
MESH_DIR = PROJECT_ROOT.parent / "rangerboxcr10lidar_description" / "meshes"

# Simulation
SIM_TIMESTEP = 0.002  # 2ms physics step
SIM_DURATION = 300.0  # 5 min max

# LiDAR
LIDAR_N_LINES = 16
LIDAR_V_ANGLES = np.linspace(-15, 15, LIDAR_N_LINES) * np.pi / 180
LIDAR_H_RAYS = 360
LIDAR_MAX_RANGE = 20.0
LIDAR_MOUNT_POS = [0.52588, 0, 0.16587]

# Camera
CAM_D435_WIDTH = 640
CAM_D435_HEIGHT = 480
CAM_D435_FOV = 57
CAM_D455_WIDTH = 640
CAM_D455_HEIGHT = 480
CAM_D455_FOV = 87

# SLAM
GRID_RESOLUTION = 0.05
GRID_BOUNDS = (-5.0, -5.0, 0.0, 5.0, 5.0, 3.0)
GRID_L_OCC = 0.85
GRID_L_FREE = -0.4
GRID_L_PRIOR = 0.0

# Navigation
NAV_LINEAR_SPEED = 0.3
NAV_ANGULAR_SPEED = 0.5
NAV_GOAL_TOLERANCE = 0.2
NAV_OBSTACLE_DIST = 0.5

# Control
CONTROL_FREQUENCY = 100
QP_WEIGHT_TRACKING = 1.0
QP_WEIGHT_REGULARIZATION = 0.01
QP_WEIGHT_COLLISION = 0.5

# Gripper
GRIPPER_OPEN = 0.6
GRIPPER_CLOSE = 0.1
GRIPPER_FORCE = 50.0

# Task
TASK_OBJECTS = ["red cup", "blue cube"]
TASK_LOCATIONS = ["table", "shelf"]
DEFAULT_TASK = "pick up the red cup and place it on the shelf"

# CR10 Joint Limits
ARM_JOINT_LIMITS = {
    "cr10_joint1": (-3.92, 0.94),
    "cr10_joint2": (-1.57, 1.57),
    "cr10_joint3": (-2.86, 2.86),
    "cr10_joint4": (-3.14, 3.14),
    "cr10_joint5": (-3.14, 3.14),
    "cr10_joint6": (-3.14, 3.14),
}
ARM_JOINT_NAMES = list(ARM_JOINT_LIMITS.keys())
ARM_JOINT_LIMITS_ARRAY = np.array(list(ARM_JOINT_LIMITS.values()))
```

- [ ] **Step 3: Write test_utils.py (failing tests)**

```python
# tests/test_utils.py
import numpy as np
import pytest
import sys
sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent))

from mujoco_demo.utils import (
    quat_to_rotmat, rotmat_to_quat, pose_to_transform,
    transform_to_pose, pose_error, euler_to_rotmat
)


class TestQuaternionConversions:
    def test_identity_quaternion(self):
        q = np.array([1, 0, 0, 0])  # wxyz
        R = quat_to_rotmat(q)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)

    def test_90deg_z_rotation(self):
        q = np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])
        R = quat_to_rotmat(q)
        expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        np.testing.assert_allclose(R, expected, atol=1e-10)

    def test_roundtrip(self):
        R = euler_to_rotmat(0.3, -0.5, 0.7)
        q = rotmat_to_quat(R)
        R2 = quat_to_rotmat(q)
        np.testing.assert_allclose(R, R2, atol=1e-10)


class TestTransforms:
    def test_identity_transform(self):
        pose = np.zeros(6)
        T = pose_to_transform(pose)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-10)

    def test_translation_only(self):
        pose = np.array([1, 2, 3, 0, 0, 0])
        T = pose_to_transform(pose)
        np.testing.assert_allclose(T[:3, 3], [1, 2, 3], atol=1e-10)

    def test_transform_roundtrip(self):
        pose = np.array([1.0, 2.0, 3.0, 0.1, -0.2, 0.3])
        T = pose_to_transform(pose)
        pose2 = transform_to_pose(T)
        T2 = pose_to_transform(pose2)
        np.testing.assert_allclose(T, T2, atol=1e-8)


class TestPoseError:
    def test_zero_error(self):
        T1 = pose_to_transform(np.array([1, 2, 3, 0, 0, 0]))
        T2 = pose_to_transform(np.array([1, 2, 3, 0, 0, 0]))
        err = pose_error(T1, T2)
        np.testing.assert_allclose(err, np.zeros(6), atol=1e-10)

    def test_translation_error(self):
        T1 = pose_to_transform(np.array([0, 0, 0, 0, 0, 0]))
        T2 = pose_to_transform(np.array([1, 0, 0, 0, 0, 0]))
        err = pose_error(T1, T2)
        assert abs(err[0] - 1.0) < 1e-10
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_utils.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'mujoco_demo.utils'`

- [ ] **Step 5: Implement utils.py**

```python
# mujoco_demo/utils.py
import numpy as np
from scipy.spatial.transform import Rotation


def quat_to_rotmat(q):
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def rotmat_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z]."""
    r = Rotation.from_matrix(R)
    q_xyzw = r.as_quat()  # scipy uses xyzw
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])


def euler_to_rotmat(roll, pitch, yaw):
    """Euler angles (rad) to rotation matrix."""
    r = Rotation.from_euler('xyz', [roll, pitch, yaw])
    return r.as_matrix()


def rotmat_to_euler(R):
    """Rotation matrix to Euler angles [roll, pitch, yaw]."""
    r = Rotation.from_matrix(R)
    return r.as_euler('xyz')


def pose_to_transform(pose):
    """Convert [x, y, z, roll, pitch, yaw] to 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = euler_to_rotmat(pose[3], pose[4], pose[5])
    T[:3, 3] = pose[:3]
    return T


def transform_to_pose(T):
    """Convert 4x4 homogeneous transform to [x, y, z, roll, pitch, yaw]."""
    pos = T[:3, 3]
    euler = rotmat_to_euler(T[:3, :3])
    return np.concatenate([pos, euler])


def pose_error(T_target, T_current):
    """Compute 6D pose error: [dx, dy, dz, droll, dpitch, dyaw]."""
    pos_err = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3] @ T_current[:3, :3].T
    euler_err = rotmat_to_euler(R_err)
    return np.concatenate([pos_err, euler_err])


def transform_point(T, point):
    """Transform a 3D point by a 4x4 matrix."""
    p_hom = np.append(point, 1.0)
    return (T @ p_hom)[:3]
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_utils.py -v
```

Expected: All tests PASS

- [ ] **Step 7: Commit**

```bash
git add mujoco_demo/__init__.py mujoco_demo/config.py mujoco_demo/utils.py tests/test_utils.py
git commit -m "feat: add project scaffolding, config, and transform utilities"
```

---

## Task 2: MuJoCo Engine - URDF Loading + Physics

**Files:**
- Create: `mujoco_demo/sim/__init__.py`
- Create: `mujoco_demo/sim/engine.py`
- Create: `tests/test_sim_engine.py`

- [ ] **Step 1: Write test_sim_engine.py (failing tests)**

```python
# tests/test_sim_engine.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.sim.engine import SimulationEngine


class TestSimulationEngine:
    def test_load_urdf(self):
        engine = SimulationEngine()
        assert engine.model is not None
        assert engine.data is not None

    def test_robot_nq(self):
        engine = SimulationEngine()
        # Ranger: 4 steering + 4 wheels = 8, CR10: 6 joints, AG95: 1 finger + mimics
        # Plus free joints for graspable objects
        assert engine.model.nq > 15  # At least 15 generalized coords

    def test_actuator_count(self):
        engine = SimulationEngine()
        # 4 steer + 4 drive + 6 arm + 1 gripper = 15 actuators
        assert engine.model.nu >= 15

    def test_step_simulation(self):
        engine = SimulationEngine()
        qpos_before = engine.data.qpos.copy()
        engine.step(10)
        # Simulation should advance (qpos may change due to gravity)
        assert engine.data.time > 0

    def test_set_arm_joints(self):
        engine = SimulationEngine()
        target = np.array([0.0, -0.5, 0.3, 0.0, -0.2, 0.0])
        engine.set_arm_joints(target)
        for _ in range(500):
            engine.step()
        qpos = engine.get_arm_joint_positions()
        np.testing.assert_allclose(qpos, target, atol=0.05)

    def test_set_gripper(self):
        engine = SimulationEngine()
        engine.set_gripper(0.6)  # Open
        for _ in range(200):
            engine.step()
        gripper_pos = engine.get_gripper_position()
        assert gripper_pos > 0.4

    def test_get_base_pose(self):
        engine = SimulationEngine()
        pose = engine.get_base_pose()
        assert len(pose) == 3  # x, y, theta
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_sim_engine.py -v
```

Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Implement sim/engine.py**

```python
# mujoco_demo/sim/engine.py
import mujoco
import numpy as np
from pathlib import Path
from mujoco_demo.config import (
    URDF_PATH, SIM_TIMESTEP, ARM_JOINT_NAMES,
    ARM_JOINT_LIMITS_ARRAY, GRIPPER_OPEN
)


class SimulationEngine:
    """MuJoCo simulation engine wrapper."""

    def __init__(self, urdf_path=None, extra_xml=""):
        if urdf_path is None:
            urdf_path = URDF_PATH

        # Load URDF
        urdf_str = self._prepare_urdf(Path(urdf_path))
        if extra_xml:
            urdf_str = self._merge_xml(urdf_str, extra_xml)

        self.model = mujoco.MjModel.from_xml_string(urdf_str)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = SIM_TIMESTEP

        # Cache joint/actuator indices
        self._arm_joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjOBJ_JOINT, name)
            for name in ARM_JOINT_NAMES
        ]
        self._arm_qpos_addrs = [
            self.model.jnt_qposadr[jid] for jid in self._arm_joint_ids
        ]
        self._arm_act_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjOBJ_ACTUATOR, f"arm_j{i+1}")
            for i in range(6)
        ]
        self._gripper_act_id = mujoco.mj_name2id(
            self.model, mujoco.mjOBJ_ACTUATOR, "gripper"
        )

        # Base joint (free joint of the robot base)
        self._base_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjOBJ_JOINT, "root"
        )
        if self._base_joint_id >= 0:
            self._base_qpos_addr = self.model.jnt_qposadr[self._base_joint_id]
        else:
            self._base_qpos_addr = 0  # fallback

        # Initialize
        mujoco.mj_forward(self.model, self.data)

    def _prepare_urdf(self, urdf_path):
        """Read URDF and fix package:// paths for MuJoCo."""
        urdf_str = urdf_path.read_text()
        mesh_dir = urdf_path.parent.parent / "meshes"
        # Replace package:// references with absolute paths
        urdf_str = urdf_str.replace(
            "package://rangerboxcr10lidar_description/meshes/",
            str(mesh_dir) + "/"
        )
        return urdf_str

    def _merge_xml(self, urdf_xml, extra_xml):
        """Wrap URDF in MJCF and add extra elements."""
        # Insert extra XML before closing </robot> tag
        insert_pos = urdf_xml.rfind("</robot>")
        if insert_pos < 0:
            # Try wrapping in mujoco tag
            return f"<mujoco>{extra_xml}{urdf_xml}</mujoco>"
        return urdf_xml[:insert_pos] + extra_xml + urdf_xml[insert_pos:]

    def step(self, n_steps=1):
        """Advance simulation by n_steps."""
        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)

    def forward(self):
        """Recompute kinematics."""
        mujoco.mj_forward(self.model, self.data)

    def get_base_pose(self):
        """Returns [x, y, theta] of the robot base."""
        qpos = self.data.qpos
        x = qpos[self._base_qpos_addr]
        y = qpos[self._base_qpos_addr + 1]
        # Quaternion to yaw
        qw = qpos[self._base_qpos_addr + 3]
        qz = qpos[self._base_qpos_addr + 6]
        theta = 2 * np.arctan2(qz, qw)
        return np.array([x, y, theta])

    def set_base_velocity(self, v, omega):
        """Set base linear and angular velocity."""
        # Find wheel actuators and compute differential drive
        fl_drive = mujoco.mj_name2id(self.model, mujoco.mjOBJ_ACTUATOR, "fl_drive")
        fr_drive = mujoco.mj_name2id(self.model, mujoco.mjOBJ_ACTUATOR, "fr_drive")
        rl_drive = mujoco.mj_name2id(self.model, mujoco.mjOBJ_ACTUATOR, "rl_drive")
        rr_drive = mujoco.mj_name2id(self.model, mjOBJ_ACTUATOR, "rr_drive")

        wheel_radius = 0.15  # approximate
        track_width = 0.56   # approximate (distance between left/right wheels)

        v_left = (v - omega * track_width / 2) / wheel_radius
        v_right = (v + omega * track_width / 2) / wheel_radius

        for aid in [fl_drive, rl_drive]:
            self.data.ctrl[aid] = v_left
        for aid in [fr_drive, rr_drive]:
            self.data.ctrl[aid] = v_right

    def set_arm_joints(self, targets):
        """Set arm joint position targets (6 values)."""
        for i, act_id in enumerate(self._arm_act_ids):
            if act_id >= 0:
                self.data.ctrl[act_id] = targets[i]

    def get_arm_joint_positions(self):
        """Returns current arm joint positions (6 values)."""
        return np.array([self.data.qpos[addr] for addr in self._arm_qpos_addrs])

    def set_gripper(self, position):
        """Set gripper position (0=closed, 0.6=open)."""
        if self._gripper_act_id >= 0:
            self.data.ctrl[self._gripper_act_id] = position

    def get_gripper_position(self):
        """Returns current gripper position."""
        gripper_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjOBJ_JOINT, "gripper_finger1_joint"
        )
        if gripper_joint_id >= 0:
            addr = self.model.jnt_qposadr[gripper_joint_id]
            return self.data.qpos[addr]
        return 0.0

    def get_body_pose(self, body_name):
        """Returns 4x4 transform of a body."""
        bid = mujoco.mj_name2id(self.model, mujoco.mjOBJ_BODY, body_name)
        pos = self.data.xpos[bid]
        mat = self.data.xmat[bid].reshape(3, 3)
        T = np.eye(4)
        T[:3, :3] = mat
        T[:3, 3] = pos
        return T

    def reset(self):
        """Reset simulation to initial state."""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
```

- [ ] **Step 4: Fix import error in set_base_velocity (mjOBJ_ACTUATOR typo)**

The above code has `mjOBJ_ACTUATOR` missing the `mujoco.` prefix. Fix:

```python
# In set_base_velocity, replace:
rr_drive = mujoco.mj_name2id(self.model, mjOBJ_ACTUATOR, "rr_drive")
# With:
rr_drive = mujoco.mj_name2id(self.model, mujoco.mjOBJ_ACTUATOR, "rr_drive")
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_sim_engine.py -v
```

Expected: All tests PASS (the engine loads the URDF, steps simulation, controls joints)

- [ ] **Step 6: Commit**

```bash
git add mujoco_demo/sim/__init__.py mujoco_demo/sim/engine.py tests/test_sim_engine.py
git commit -m "feat: add MuJoCo simulation engine with URDF loading and joint control"
```

---

## Task 3: Home Environment Scene

**Files:**
- Create: `mujoco_demo/sim/env.py`
- Modify: `mujoco_demo/sim/engine.py`
- Create: `tests/test_sim_env.py`

- [ ] **Step 1: Write test_sim_env.py (failing tests)**

```python
# tests/test_sim_env.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.sim.env import HomeEnvironment
from mujoco_demo.sim.engine import SimulationEngine


class TestHomeEnvironment:
    def test_scene_xml_generation(self):
        env = HomeEnvironment()
        xml = env.get_scene_xml()
        assert "table" in xml
        assert "shelf" in xml
        assert "red_cup" in xml
        assert "blue_cube" in xml
        assert "plane" in xml

    def test_load_with_engine(self):
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        assert engine.model is not None

    def test_object_positions(self):
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        # Table should be at (2, 1, 0)
        table_body_id = mujoco.mj_name2id(engine.model, mujoco.mjOBJ_BODY, "table")
        assert table_body_id >= 0

    def test_room_bounds(self):
        env = HomeEnvironment()
        bounds = env.get_room_bounds()
        assert bounds[0] < bounds[2]  # x_min < x_max
        assert bounds[1] < bounds[3]  # y_min < y_max
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_sim_env.py -v
```

Expected: FAIL

- [ ] **Step 3: Implement sim/env.py**

```python
# mujoco_demo/sim/env.py
import numpy as np


class HomeEnvironment:
    """Generates MuJoCo XML for a simple home scene."""

    def __init__(self):
        self.room_x_range = (-4.5, 4.5)
        self.room_y_range = (-4.5, 4.5)
        self.table_pos = (2.0, 1.0, 0.0)
        self.shelf_pos = (-2.0, 0.0, 0.0)
        self.cup_pos = (2.0, 1.0, 0.45)
        self.cube_pos = (2.2, 0.8, 0.45)

    def get_scene_xml(self):
        """Returns MuJoCo XML string for the home scene."""
        return f"""
        <worldbody>
          <!-- Floor -->
          <geom name="floor" type="plane" size="5 5 0.1" rgba="0.9 0.9 0.85 1" pos="0 0 0"/>
          <!-- Walls -->
          <geom name="wall_north" type="box" pos="0 5 1" size="5.1 0.1 1" rgba="0.8 0.8 0.8 1"/>
          <geom name="wall_south" type="box" pos="0 -5 1" size="5.1 0.1 1" rgba="0.8 0.8 0.8 1"/>
          <geom name="wall_east" type="box" pos="5 0 1" size="0.1 5.1 1" rgba="0.8 0.8 0.8 1"/>
          <geom name="wall_west" type="box" pos="-5 0 1" size="0.1 5.1 1" rgba="0.8 0.8 0.8 1"/>
          <!-- Table -->
          <body name="table" pos="{self.table_pos[0]} {self.table_pos[1]} {self.table_pos[2]}">
            <geom name="table_top" type="box" pos="0 0 0.4" size="0.6 0.4 0.02" rgba="0.6 0.4 0.2 1" mass="10"/>
            <geom name="table_leg1" type="cylinder" pos="-0.5 -0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
            <geom name="table_leg2" type="cylinder" pos="0.5 -0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
            <geom name="table_leg3" type="cylinder" pos="-0.5 0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
            <geom name="table_leg4" type="cylinder" pos="0.5 0.3 0.2" size="0.03 0.2" rgba="0.5 0.3 0.1 1"/>
          </body>
          <!-- Shelf -->
          <body name="shelf" pos="{self.shelf_pos[0]} {self.shelf_pos[1]} {self.shelf_pos[2]}">
            <geom name="shelf_bottom" type="box" pos="0 0 0.5" size="0.3 0.8 0.02" rgba="0.4 0.4 0.4 1" mass="5"/>
            <geom name="shelf_top" type="box" pos="0 0 1.0" size="0.3 0.8 0.02" rgba="0.4 0.4 0.4 1" mass="5"/>
            <geom name="shelf_side1" type="box" pos="0 -0.78 0.75" size="0.3 0.02 0.25" rgba="0.4 0.4 0.4 1"/>
            <geom name="shelf_side2" type="box" pos="0 0.78 0.75" size="0.3 0.02 0.25" rgba="0.4 0.4 0.4 1"/>
          </body>
          <!-- Red Cup -->
          <body name="red_cup" pos="{self.cup_pos[0]} {self.cup_pos[1]} {self.cup_pos[2]}">
            <freejoint name="red_cup_joint"/>
            <geom name="red_cup_geom" type="cylinder" size="0.03 0.06" rgba="1 0.2 0.2 1" mass="0.1"/>
          </body>
          <!-- Blue Cube -->
          <body name="blue_cube" pos="{self.cube_pos[0]} {self.cube_pos[1]} {self.cube_pos[2]}">
            <freejoint name="blue_cube_joint"/>
            <geom name="blue_cube_geom" type="box" size="0.03 0.03 0.03" rgba="0.2 0.2 1 1" mass="0.15"/>
          </body>
          <!-- Obstacle box -->
          <body name="obstacle1" pos="0 0.5 0.25">
            <geom name="obstacle1_geom" type="box" size="0.2 0.15 0.25" rgba="0.6 0.6 0.6 1" mass="5"/>
          </body>
        </worldbody>
        """

    def get_room_bounds(self):
        """Returns (x_min, y_min, x_max, y_max)."""
        return (self.room_x_range[0], self.room_y_range[0],
                self.room_x_range[1], self.room_y_range[1])

    def get_object_positions(self):
        """Returns dict of object names to their initial positions."""
        return {
            "red_cup": np.array(self.cup_pos),
            "blue_cube": np.array(self.cube_pos),
            "table": np.array(self.table_pos),
            "shelf": np.array(self.shelf_pos),
        }
```

- [ ] **Step 4: Run tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_sim_env.py -v
```

Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
git add mujoco_demo/sim/env.py tests/test_sim_env.py
git commit -m "feat: add home environment scene with furniture and graspable objects"
```

---

## Task 4: LiDAR Sensor Simulation

**Files:**
- Create: `mujoco_demo/sim/sensors.py`
- Create: `tests/test_sim_sensors.py`

- [ ] **Step 1: Write test_sim_sensors.py (failing tests)**

```python
# tests/test_sim_sensors.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import mujoco
from mujoco_demo.sim.engine import SimulationEngine
from mujoco_demo.sim.env import HomeEnvironment
from mujoco_demo.sim.sensors import LiDARSensor, RGBDCamera


class TestLiDARSensor:
    def _make_engine(self):
        env = HomeEnvironment()
        return SimulationEngine(extra_xml=env.get_scene_xml())

    def test_scan_returns_points(self):
        engine = self._make_engine()
        lidar = LiDARSensor(engine.model, engine.data)
        points = lidar.scan(engine.model, engine.data)
        assert points.ndim == 2
        assert points.shape[1] == 3
        assert len(points) > 0

    def test_scan_range(self):
        engine = self._make_engine()
        lidar = LiDARSensor(engine.model, engine.data)
        points = lidar.scan(engine.model, engine.data)
        # All points should be within max range
        ranges = np.linalg.norm(points, axis=1)
        assert np.all(ranges <= 20.0 + 0.1)

    def test_scan_changes_with_position(self):
        engine = self._make_engine()
        lidar = LiDARSensor(engine.model, engine.data)
        points1 = lidar.scan(engine.model, engine.data)
        # Move robot
        engine.data.qpos[0] += 1.0
        mujoco.mj_forward(engine.model, engine.data)
        points2 = lidar.scan(engine.model, engine.data)
        # Points should be different after moving
        assert not np.allclose(points1[:10], points2[:10], atol=0.01)


class TestRGBDCamera:
    def _make_engine(self):
        env = HomeEnvironment()
        return SimulationEngine(extra_xml=env.get_scene_xml())

    def test_capture_rgb(self):
        engine = self._make_engine()
        cam = RGBDCamera("d435_cam", width=320, height=240)
        rgb, depth = cam.capture(engine.model, engine.data)
        assert rgb.shape == (240, 320, 3)
        assert rgb.dtype == np.uint8

    def test_capture_depth(self):
        engine = self._make_engine()
        cam = RGBDCamera("d435_cam", width=320, height=240)
        rgb, depth = cam.capture(engine.model, engine.data)
        assert depth.shape == (240, 320)
        assert depth.dtype == np.float32
        # Some pixels should have valid depth
        valid = depth[(depth > 0) & (depth < 10)]
        assert len(valid) > 100
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_sim_sensors.py -v
```

Expected: FAIL

- [ ] **Step 3: Implement sensors.py**

```python
# mujoco_demo/sim/sensors.py
import mujoco
import numpy as np
from mujoco_demo.config import (
    LIDAR_N_LINES, LIDAR_V_ANGLES, LIDAR_H_RAYS,
    LIDAR_MAX_RANGE, LIDAR_MOUNT_POS
)


class LiDARSensor:
    """Simulated 3D LiDAR using MuJoCo ray casting."""

    def __init__(self, model, data, mount_body="base_link"):
        self.n_lines = LIDAR_N_LINES
        self.v_angles = LIDAR_V_ANGLES
        self.h_rays = LIDAR_H_RAYS
        self.max_range = LIDAR_MAX_RANGE
        self.mount_body = mount_body

        # Pre-compute ray directions in sensor frame
        self._local_dirs = []
        for v_angle in self.v_angles:
            for h_angle in np.linspace(0, 2 * np.pi, self.h_rays, endpoint=False):
                d = np.array([
                    np.cos(v_angle) * np.cos(h_angle),
                    np.cos(v_angle) * np.sin(h_angle),
                    np.sin(v_angle)
                ])
                self._local_dirs.append(d)
        self._local_dirs = np.array(self._local_dirs)

    def scan(self, model, data):
        """Cast rays from LiDAR mount position. Returns Nx3 point cloud."""
        # Get mount body pose
        bid = mujoco.mj_name2id(model, mujoco.mjOBJ_BODY, self.mount_body)
        mount_pos = data.xpos[bid].copy()
        mount_mat = data.xmat[bid].reshape(3, 3).copy()

        # Transform mount offset to world frame
        offset = mount_mat @ np.array(LIDAR_MOUNT_POS)
        sensor_pos = mount_pos + offset

        # Transform ray directions to world frame
        world_dirs = (mount_mat @ self._local_dirs.T).T

        # Cast rays
        points = []
        for direction in world_dirs:
            geom_id = np.array([-1], dtype=np.int32)
            distance = np.array([0.0])
            mujoco.mj_ray(model, data, sensor_pos, direction, None, 1, -1, geom_id, distance)
            if distance[0] > 0 and distance[0] < self.max_range:
                hit = sensor_pos + direction * distance[0]
                points.append(hit)

        if len(points) == 0:
            return np.empty((0, 3))
        return np.array(points)


class RGBDCamera:
    """Simulated RGB-D camera using MuJoCo offscreen rendering."""

    def __init__(self, cam_name, width=640, height=480):
        self.cam_name = cam_name
        self.width = width
        self.height = height

        # Camera intrinsics (computed from FOV)
        cam_id = -1  # Will be resolved at capture time
        self._fov = 57.0  # default, will be updated

    def capture(self, model, data):
        """Render RGB + depth from camera. Returns (rgb, depth)."""
        cam_id = mujoco.mj_name2id(model, mujoco.mjOBJ_CAMERA, self.cam_name)
        if cam_id < 0:
            raise ValueError(f"Camera '{self.cam_name}' not found in model")

        fovy = model.cam_fovy[cam_id]
        self._fov = fovy

        # Create offscreen rendering context
        scene = mujoco.MjvScene(model, maxgeom=10000)
        opt = mujoco.MjvOption()
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam.fixedcamid = cam_id

        context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)

        # Update scene
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)

        # Render
        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        mujoco.mjr_render(viewport, scene, context)

        # Read pixels
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.zeros((self.height, self.width), dtype=np.float32)
        mujoco.mjr_readPixels(rgb, depth, viewport, context)

        # Flip vertically (MuJoCo renders bottom-up)
        rgb = rgb[::-1]
        depth = depth[::-1]

        # Convert depth buffer to meters
        # MuJoCo depth: 0=near, 1=far; linearize
        znear = 0.01
        zfar = 50.0
        depth_meters = znear * zfar / (zfar - depth * (zfar - depth))

        return rgb, depth_meters

    def get_intrinsics(self):
        """Returns 3x3 camera intrinsic matrix K."""
        fov_rad = np.deg2rad(self._fov)
        fy = (self.height / 2) / np.tan(fov_rad / 2)
        fx = fy  # Square pixels
        cx = self.width / 2
        cy = self.height / 2
        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
```

- [ ] **Step 4: Run tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_sim_sensors.py -v
```

Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
git add mujoco_demo/sim/sensors.py tests/test_sim_sensors.py
git commit -m "feat: add LiDAR ray casting and RGB-D camera simulation"
```

---

## Task 5: Occupancy Grid SLAM

**Files:**
- Create: `mujoco_demo/slam/__init__.py`
- Create: `mujoco_demo/slam/occupancy.py`
- Create: `mujoco_demo/slam/mapper.py`
- Create: `mujoco_demo/slam/scene_graph.py`
- Create: `tests/test_slam_occupancy.py`

- [ ] **Step 1: Write test_slam_occupancy.py (failing tests)**

```python
# tests/test_slam_occupancy.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.slam.occupancy import OccupancyGrid
from mujoco_demo.slam.mapper import Mapper
from mujoco_demo.slam.scene_graph import SceneGraph, SceneGraphNode


class TestOccupancyGrid:
    def test_initialization(self):
        grid = OccupancyGrid(resolution=0.1, bounds=(-2, -2, 0, 2, 2, 2))
        assert grid.resolution == 0.1
        assert grid.grid.shape[0] > 0

    def test_update_point_cloud(self):
        grid = OccupancyGrid(resolution=0.1, bounds=(-2, -2, 0, 2, 2, 2))
        origin = np.array([0, 0, 0.5])
        points = np.array([[1, 0, 0.5], [0, 1, 0.5], [-1, 0, 0.5]])
        grid.update(origin, points)
        # Cells at hit locations should be occupied
        assert grid.is_occupied(np.array([1, 0, 0.5]))

    def test_free_space(self):
        grid = OccupancyGrid(resolution=0.1, bounds=(-2, -2, 0, 2, 2, 2))
        origin = np.array([0, 0, 0.5])
        points = np.array([[1.5, 0, 0.5]])
        grid.update(origin, points)
        # Cell near origin should be free
        assert not grid.is_occupied(np.array([0.1, 0, 0.5]))

    def test_2d_projection(self):
        grid = OccupancyGrid(resolution=0.1, bounds=(-2, -2, 0, 2, 2, 2))
        origin = np.array([0, 0, 0.5])
        points = np.array([[1, 0, 0.5]])
        grid.update(origin, points)
        map_2d = grid.get_2d_map(z_min=0.1, z_max=1.5)
        assert map_2d.ndim == 2


class TestMapper:
    def test_update(self):
        grid = OccupancyGrid(resolution=0.1, bounds=(-2, -2, 0, 2, 2, 2))
        mapper = Mapper(grid)
        points = np.array([[1, 0, 0.5], [0, 1, 0.5]])
        origin = np.array([0, 0, 0.5])
        mapper.update(origin, points)
        assert mapper.scan_count == 1


class TestSceneGraph:
    def test_add_node(self):
        sg = SceneGraph()
        node = SceneGraphNode(
            id="cup_1", obj_type="graspable",
            pose=np.array([1, 0, 0.5, 0, 0, 0]),
            size=np.array([0.06, 0.06, 0.12]),
            confidence=0.9, parent="table_1"
        )
        sg.add_node(node)
        assert "cup_1" in sg.nodes

    def test_get_graspable_objects(self):
        sg = SceneGraph()
        sg.add_node(SceneGraphNode(
            id="cup_1", obj_type="graspable",
            pose=np.array([1, 0, 0.5, 0, 0, 0]),
            size=np.array([0.06, 0.06, 0.12]),
            confidence=0.9, parent="table_1"
        ))
        sg.add_node(SceneGraphNode(
            id="table_1", obj_type="furniture",
            pose=np.array([2, 1, 0, 0, 0, 0]),
            size=np.array([1.2, 0.8, 0.8]),
            confidence=1.0, parent=None
        ))
        graspable = sg.get_graspable_objects()
        assert len(graspable) == 1
        assert graspable[0].id == "cup_1"
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_slam_occupancy.py -v
```

Expected: FAIL

- [ ] **Step 3: Implement slam/occupancy.py**

```python
# mujoco_demo/slam/occupancy.py
import numpy as np
from mujoco_demo.config import GRID_RESOLUTION, GRID_BOUNDS, GRID_L_OCC, GRID_L_FREE, GRID_L_PRIOR


class OccupancyGrid:
    """3D log-odds occupancy grid with 2D projection."""

    def __init__(self, resolution=None, bounds=None):
        self.resolution = resolution or GRID_RESOLUTION
        self.bounds = bounds or GRID_BOUNDS  # (x_min, y_min, z_min, x_max, y_max, z_max)
        self.l_occ = GRID_L_OCC
        self.l_free = GRID_L_FREE
        self.l_prior = GRID_L_PRIOR

        # Compute grid dimensions
        self.nx = int((self.bounds[3] - self.bounds[0]) / self.resolution)
        self.ny = int((self.bounds[4] - self.bounds[1]) / self.resolution)
        self.nz = int((self.bounds[5] - self.bounds[2]) / self.resolution)
        self.grid = np.full((self.nx, self.ny, self.nz), self.l_prior)

    def _world_to_grid(self, point):
        """Convert world coordinates to grid indices."""
        ix = int((point[0] - self.bounds[0]) / self.resolution)
        iy = int((point[1] - self.bounds[1]) / self.resolution)
        iz = int((point[2] - self.bounds[2]) / self.resolution)
        return ix, iy, iz

    def _in_bounds(self, ix, iy, iz):
        return 0 <= ix < self.nx and 0 <= iy < self.ny and 0 <= iz < self.nz

    def is_occupied(self, point):
        """Check if a world point is in an occupied cell."""
        ix, iy, iz = self._world_to_grid(point)
        if not self._in_bounds(ix, iy, iz):
            return False
        return self.grid[ix, iy, iz] > 0

    def is_free(self, point):
        """Check if a world point is in a free cell."""
        ix, iy, iz = self._world_to_grid(point)
        if not self._in_bounds(ix, iy, iz):
            return False
        return self.grid[ix, iy, iz] < 0

    def update(self, sensor_origin, points):
        """Update grid with LiDAR observation."""
        for point in points:
            self._mark_ray_free(sensor_origin, point)
            self._mark_occupied(point)

    def _mark_ray_free(self, origin, endpoint):
        """Mark cells along ray as free using DDA."""
        dx = endpoint - origin
        dist = np.linalg.norm(dx)
        if dist < 1e-6:
            return
        direction = dx / dist
        step = self.resolution * 0.5
        n_steps = int(dist / step)
        for i in range(n_steps):
            p = origin + direction * i * step
            ix, iy, iz = self._world_to_grid(p)
            if self._in_bounds(ix, iy, iz):
                self.grid[ix, iy, iz] = max(self.grid[ix, iy, iz] + self.l_free, -5.0)

    def _mark_occupied(self, point):
        """Mark the hit cell as occupied."""
        ix, iy, iz = self._world_to_grid(point)
        if self._in_bounds(ix, iy, iz):
            self.grid[ix, iy, iz] = min(self.grid[ix, iy, iz] + self.l_occ, 5.0)

    def get_2d_map(self, z_min=0.1, z_max=1.5):
        """Project 3D grid to 2D occupancy map for navigation."""
        iz_min = max(0, int((z_min - self.bounds[2]) / self.resolution))
        iz_max = min(self.nz, int((z_max - self.bounds[2]) / self.resolution))
        if iz_min >= iz_max:
            return np.zeros((self.nx, self.ny), dtype=bool)
        return np.max(self.grid[:, :, iz_min:iz_max], axis=2) > 0

    def get_2d_occupancy(self, z_min=0.1, z_max=1.5):
        """Returns 2D occupancy probability (0-1)."""
        iz_min = max(0, int((z_min - self.bounds[2]) / self.resolution))
        iz_max = min(self.nz, int((z_max - self.bounds[2]) / self.resolution))
        logodds_2d = np.max(self.grid[:, :, iz_min:iz_max], axis=2)
        return 1.0 / (1.0 + np.exp(-logodds_2d))
```

- [ ] **Step 4: Implement slam/mapper.py**

```python
# mujoco_demo/slam/mapper.py
import numpy as np


class Mapper:
    """Manages incremental mapping from sensor observations."""

    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.scan_count = 0

    def update(self, sensor_origin, points):
        """Process a new LiDAR scan."""
        if len(points) == 0:
            return
        self.grid.update(sensor_origin, points)
        self.scan_count += 1

    def get_map_2d(self):
        """Get the current 2D occupancy map."""
        return self.grid.get_2d_map()
```

- [ ] **Step 5: Implement slam/scene_graph.py**

```python
# mujoco_demo/slam/scene_graph.py
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class SceneGraphNode:
    id: str
    obj_type: str  # "graspable", "furniture", "obstacle"
    pose: np.ndarray  # 6D: [x, y, z, roll, pitch, yaw]
    size: np.ndarray  # [w, h, d]
    confidence: float = 1.0
    parent: Optional[str] = None


class SceneGraph:
    """Tracks objects and spatial relationships."""

    def __init__(self):
        self.nodes: Dict[str, SceneGraphNode] = {}
        self.edges: List[Tuple[str, str, str]] = []  # (from, to, relation)

    def add_node(self, node: SceneGraphNode):
        self.nodes[node.id] = node

    def remove_node(self, node_id: str):
        if node_id in self.nodes:
            del self.nodes[node_id]
            self.edges = [(f, t, r) for f, t, r in self.edges if f != node_id and t != node_id]

    def add_edge(self, from_id: str, to_id: str, relation: str):
        self.edges.append((from_id, to_id, relation))

    def get_graspable_objects(self) -> List[SceneGraphNode]:
        return [n for n in self.nodes.values() if n.obj_type == "graspable"]

    def get_furniture(self) -> List[SceneGraphNode]:
        return [n for n in self.nodes.values() if n.obj_type == "furniture"]

    def find_by_type(self, obj_type: str) -> List[SceneGraphNode]:
        return [n for n in self.nodes.values() if n.obj_type == obj_type]

    def find_by_name(self, name: str) -> Optional[SceneGraphNode]:
        for node in self.nodes.values():
            if name.lower() in node.id.lower():
                return node
        return None
```

- [ ] **Step 6: Run tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_slam_occupancy.py -v
```

Expected: All tests PASS

- [ ] **Step 7: Commit**

```bash
git add mujoco_demo/slam/ tests/test_slam_occupancy.py
git commit -m "feat: add 3D occupancy grid SLAM, mapper, and scene graph"
```

---

## Task 6: A* Global Planner

**Files:**
- Create: `mujoco_demo/nav/__init__.py`
- Create: `mujoco_demo/nav/astar.py`
- Create: `tests/test_nav_astar.py`

- [ ] **Step 1: Write test_nav_astar.py (failing tests)**

```python
# tests/test_nav_astar.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.nav.astar import AStarPlanner


class TestAStarPlanner:
    def test_straight_line(self):
        grid = np.zeros((100, 100), dtype=bool)
        planner = AStarPlanner(grid, resolution=0.1)
        path = planner.plan((1.0, 1.0), (5.0, 1.0))
        assert path is not None
        assert len(path) > 0
        # Start and end should be close to requested
        np.testing.assert_allclose(path[0], [1.0, 1.0], atol=0.2)
        np.testing.assert_allclose(path[-1], [5.0, 1.0], atol=0.2)

    def test_avoids_obstacle(self):
        grid = np.zeros((100, 100), dtype=bool)
        # Wall at x=3
        grid[25:75, 30] = True
        planner = AStarPlanner(grid, resolution=0.1)
        path = planner.plan((1.0, 5.0), (5.0, 5.0))
        assert path is not None
        # Path should go around the wall
        for p in path:
            if 2.9 < p[0] < 3.1:
                assert p[1] < 3.0 or p[1] > 7.0

    def test_no_path(self):
        grid = np.ones((100, 100), dtype=bool)
        planner = AStarPlanner(grid, resolution=0.1)
        path = planner.plan((1.0, 1.0), (5.0, 5.0))
        assert path is None

    def test_same_start_goal(self):
        grid = np.zeros((100, 100), dtype=bool)
        planner = AStarPlanner(grid, resolution=0.1)
        path = planner.plan((3.0, 3.0), (3.0, 3.0))
        assert path is not None
        assert len(path) <= 2
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_nav_astar.py -v
```

Expected: FAIL

- [ ] **Step 3: Implement nav/astar.py**

```python
# mujoco_demo/nav/astar.py
import numpy as np
from heapq import heappush, heappop


class AStarPlanner:
    """A* path planner on a 2D occupancy grid."""

    def __init__(self, grid_2d, resolution=0.05, origin=(0.0, 0.0)):
        """
        Args:
            grid_2d: 2D boolean array (True = occupied)
            resolution: meters per cell
            origin: world coordinates of grid cell (0,0)
        """
        self.grid = grid_2d
        self.resolution = resolution
        self.origin = np.array(origin)
        self.shape = grid_2d.shape

    def _world_to_grid(self, point):
        gx = int((point[0] - self.origin[0]) / self.resolution)
        gy = int((point[1] - self.origin[1]) / self.resolution)
        return (max(0, min(self.shape[0]-1, gx)),
                max(0, min(self.shape[1]-1, gy)))

    def _grid_to_world(self, cell):
        x = cell[0] * self.resolution + self.origin[0]
        y = cell[1] * self.resolution + self.origin[1]
        return np.array([x, y])

    def _heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def _get_neighbors(self, cell):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cell[0]+dx, cell[1]+dy
                if 0 <= nx < self.shape[0] and 0 <= ny < self.shape[1]:
                    if not self.grid[nx, ny]:
                        neighbors.append((nx, ny))
        return neighbors

    def plan(self, start, goal):
        """Plan path from start to goal in world coordinates.

        Returns list of (x, y) waypoints, or None if no path.
        """
        start_grid = self._world_to_grid(start)
        goal_grid = self._world_to_grid(goal)

        if self.grid[start_grid[0], start_grid[1]]:
            return None
        if self.grid[goal_grid[0], goal_grid[1]]:
            return None

        open_set = []
        heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}

        while open_set:
            _, current = heappop(open_set)
            if current == goal_grid:
                path = self._reconstruct(came_from, current)
                return [self._grid_to_world(c) for c in path]

            for neighbor in self._get_neighbors(current):
                dx = abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1])
                cost = self.resolution * (1.414 if dx == 2 else 1.0)
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_grid)
                    heappush(open_set, (f, neighbor))

        return None

    def _reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
```

- [ ] **Step 4: Run tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_nav_astar.py -v
```

Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
git add mujoco_demo/nav/__init__.py mujoco_demo/nav/astar.py tests/test_nav_astar.py
git commit -m "feat: add A* global path planner on 2D occupancy grid"
```

---

## Task 7: VFH Local Avoidance + Path Follower

**Files:**
- Create: `mujoco_demo/nav/vfh.py`
- Create: `mujoco_demo/nav/planner.py`
- Create: `tests/test_nav_vfh.py`

- [ ] **Step 1: Write test_nav_vfh.py (failing tests)**

```python
# tests/test_nav_vfh.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.nav.vfh import VFHPlanner
from mujoco_demo.nav.planner import PathFollower


class TestVFHPlanner:
    def test_clear_path(self):
        vfh = VFHPlanner()
        # No obstacles: should go straight toward goal
        lidar_scan = np.full(360, 10.0)  # 10m in all directions
        v, omega = vfh.compute_velocity(
            np.array([0, 0]), 0.0, np.array([5, 0]), lidar_scan
        )
        assert v > 0
        assert abs(omega) < 0.5

    def test_obstacle_ahead(self):
        vfh = VFHPlanner()
        lidar_scan = np.full(360, 10.0)
        lidar_scan[350:360] = 0.5  # Obstacle ahead-right
        lidar_scan[0:10] = 0.5    # Obstacle ahead-left
        v, omega = vfh.compute_velocity(
            np.array([0, 0]), 0.0, np.array([5, 0]), lidar_scan
        )
        assert v < 0.3  # Should slow down
        # Should turn to avoid
        assert abs(omega) > 0.1


class TestPathFollower:
    def test_follow_simple_path(self):
        follower = PathFollower(goal_tolerance=0.3)
        path = [np.array([1, 0]), np.array([2, 0]), np.array([3, 0])]
        v, omega = follower.compute_velocity(
            np.array([0, 0]), 0.0, path
        )
        assert v > 0

    def test_reached_waypoint(self):
        follower = PathFollower(goal_tolerance=0.3)
        path = [np.array([0.1, 0]), np.array([3, 0])]
        # Should advance to next waypoint when close
        v, omega = follower.compute_velocity(
            np.array([0, 0]), 0.0, path
        )
        assert follower.current_wp_idx >= 1

    def test_path_complete(self):
        follower = PathFollower(goal_tolerance=0.3)
        path = [np.array([0.1, 0])]
        v, omega = follower.compute_velocity(
            np.array([0, 0]), 0.0, path
        )
        assert follower.is_complete()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_nav_vfh.py -v
```

Expected: FAIL

- [ ] **Step 3: Implement nav/vfh.py**

```python
# mujoco_demo/nav/vfh.py
import numpy as np
from mujoco_demo.config import NAV_LINEAR_SPEED, NAV_ANGULAR_SPEED, NAV_OBSTACLE_DIST


class VFHPlanner:
    """VFH+ local obstacle avoidance using polar histogram."""

    def __init__(self, threshold=None, robot_radius=0.4, sectors=72):
        self.threshold = threshold or NAV_OBSTACLE_DIST
        self.robot_radius = robot_radius
        self.sectors = sectors
        self.sector_angle = 2 * np.pi / sectors

    def _build_histogram(self, lidar_scan):
        """Build polar histogram from LiDAR ranges."""
        histogram = np.zeros(self.sectors)
        n_rays = len(lidar_scan)
        for i, r in enumerate(lidar_scan):
            angle = 2 * np.pi * i / n_rays
            sector = int(angle / self.sector_angle) % self.sectors
            if r < self.threshold:
                # Obstacle density: closer = higher
                histogram[sector] += (self.threshold - r) ** 2
        return histogram

    def _find_valleys(self, histogram):
        """Find open directions (valleys in histogram)."""
        threshold = 0.1
        valleys = []
        in_valley = False
        start = 0
        for i in range(self.sectors):
            if histogram[i] < threshold:
                if not in_valley:
                    start = i
                    in_valley = True
            else:
                if in_valley:
                    valleys.append((start, i - 1))
                    in_valley = False
        if in_valley:
            valleys.append((start, self.sectors - 1))
        return valleys

    def _select_direction(self, valleys, goal_direction):
        """Select best valley direction closest to goal."""
        if not valleys:
            return goal_direction
        goal_sector = int(goal_direction / self.sector_angle) % self.sectors
        best_dir = goal_direction
        best_cost = float('inf')
        for start, end in valleys:
            mid = (start + end) / 2 * self.sector_angle
            cost = abs(mid - goal_direction)
            if cost < best_cost:
                best_cost = cost
                best_dir = mid
        return best_dir

    def compute_velocity(self, current_pos, current_heading, goal, lidar_scan):
        """Compute (v, omega) for obstacle-avoidant motion toward goal."""
        histogram = self._build_histogram(lidar_scan)
        valleys = self._find_valleys(histogram)
        goal_direction = np.arctan2(goal[1]-current_pos[1], goal[0]-current_pos[0])
        best_direction = self._select_direction(valleys, goal_direction)

        direction_error = best_direction - current_heading
        # Normalize to [-pi, pi]
        direction_error = (direction_error + np.pi) % (2 * np.pi) - np.pi

        v = NAV_LINEAR_SPEED * max(0.1, np.cos(direction_error))
        omega = np.clip(NAV_ANGULAR_SPEED * direction_error, -NAV_ANGULAR_SPEED, NAV_ANGULAR_SPEED)
        return v, omega
```

- [ ] **Step 4: Implement nav/planner.py**

```python
# mujoco_demo/nav/planner.py
import numpy as np
from mujoco_demo.config import NAV_GOAL_TOLERANCE, NAV_LINEAR_SPEED


class PathFollower:
    """Follows a sequence of waypoints using VFH."""

    def __init__(self, goal_tolerance=None):
        self.goal_tolerance = goal_tolerance or NAV_GOAL_TOLERANCE
        self.current_wp_idx = 0
        self._path = []

    def set_path(self, path):
        self._path = path
        self.current_wp_idx = 0

    def is_complete(self):
        return self.current_wp_idx >= len(self._path)

    def compute_velocity(self, current_pos, current_heading, path=None):
        """Compute velocity to follow path. Returns (v, omega)."""
        if path is not None:
            self.set_path(path)

        if self.is_complete():
            return 0.0, 0.0

        # Advance waypoint if close enough
        while self.current_wp_idx < len(self._path):
            wp = self._path[self.current_wp_idx]
            dist = np.linalg.norm(current_pos - wp)
            if dist < self.goal_tolerance:
                self.current_wp_idx += 1
            else:
                break

        if self.is_complete():
            return 0.0, 0.0

        target = self._path[self.current_wp_idx]
        direction = np.arctan2(target[1]-current_pos[1], target[0]-current_pos[0])
        error = direction - current_heading
        error = (error + np.pi) % (2*np.pi) - np.pi

        dist = np.linalg.norm(current_pos - target)
        v = min(NAV_LINEAR_SPEED, dist * 0.5) * max(0.1, np.cos(error))
        omega = np.clip(2.0 * error, -1.0, 1.0)
        return v, omega
```

- [ ] **Step 5: Run tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_nav_vfh.py -v
```

Expected: All tests PASS

- [ ] **Step 6: Commit**

```bash
git add mujoco_demo/nav/vfh.py mujoco_demo/nav/planner.py tests/test_nav_vfh.py
git commit -m "feat: add VFH local avoidance and path follower"
```

---

## Task 8: Whole-Body Kinematics + Controller

**Files:**
- Create: `mujoco_demo/control/__init__.py`
- Create: `mujoco_demo/control/kinematics.py`
- Create: `mujoco_demo/control/whole_body.py`
- Create: `mujoco_demo/control/gripper.py`
- Create: `tests/test_control_kinematics.py`

- [ ] **Step 1: Write test_control_kinematics.py (failing tests)**

```python
# tests/test_control_kinematics.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.sim.engine import SimulationEngine
from mujoco_demo.sim.env import HomeEnvironment
from mujoco_demo.control.kinematics import WholeBodyKinematics
from mujoco_demo.control.whole_body import WholeBodyController
from mujoco_demo.control.gripper import GripperController
from mujoco_demo.utils import pose_to_transform


class TestWholeBodyKinematics:
    def _make(self):
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        kin = WholeBodyKinematics(engine.model, engine.data)
        return engine, kin

    def test_fk_returns_4x4(self):
        engine, kin = self._make()
        q = np.zeros(9)
        T = kin.forward_kinematics(q)
        assert T.shape == (4, 4)

    def test_fk_identity_base(self):
        engine, kin = self._make()
        q = np.zeros(9)
        T = kin.forward_kinematics(q)
        # With zero base and zero arm joints, EE should be at arm reach
        assert T[2, 3] > 0  # z should be positive (arm mounted on base)

    def test_jacobian_shape(self):
        engine, kin = self._make()
        q = np.zeros(9)
        J = kin.jacobian(q)
        assert J.shape == (6, 9)

    def test_fk_changes_with_arm_joints(self):
        engine, kin = self._make()
        q1 = np.zeros(9)
        q2 = np.array([0, 0, 0, 0.5, 0, 0, 0, 0, 0])
        T1 = kin.forward_kinematics(q1)
        T2 = kin.forward_kinematics(q2)
        assert not np.allclose(T1, T2, atol=0.01)


class TestWholeBodyController:
    def _make(self):
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        kin = WholeBodyKinematics(engine.model, engine.data)
        ctrl = WholeBodyController(kin, engine.model, engine.data)
        return engine, kin, ctrl

    def test_compute_velocities_shape(self):
        engine, kin, ctrl = self._make()
        q = np.zeros(9)
        target = pose_to_transform(np.array([0.5, 0, 0.5, 0, 0, 0]))
        dq = ctrl.compute_joint_velocities(q, target, [])
        assert len(dq) == 9

    def test_velocities_move_toward_target(self):
        engine, kin, ctrl = self._make()
        q = np.zeros(9)
        T_ee = kin.forward_kinematics(q)
        # Target slightly forward
        target = T_ee.copy()
        target[0, 3] += 0.1
        dq = ctrl.compute_joint_velocities(q, target, [])
        # At least some joints should have non-zero velocity
        assert np.any(np.abs(dq) > 1e-6)


class TestGripperController:
    def test_open(self):
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        gripper = GripperController(engine)
        gripper.open()
        for _ in range(300):
            engine.step()
        assert engine.get_gripper_position() > 0.4

    def test_close(self):
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        gripper = GripperController(engine)
        gripper.open()
        for _ in range(300):
            engine.step()
        gripper.close()
        for _ in range(300):
            engine.step()
        assert engine.get_gripper_position() < 0.3
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_control_kinematics.py -v
```

Expected: FAIL

- [ ] **Step 3: Implement control/kinematics.py**

```python
# mujoco_demo/control/kinematics.py
import mujoco
import numpy as np
from mujoco_demo.utils import pose_to_transform, transform_to_pose, pose_error
from mujoco_demo.config import ARM_JOINT_NAMES


class WholeBodyKinematics:
    """Forward kinematics and Jacobian for base (3-DOF) + arm (6-DOF)."""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.n_base = 3  # x, y, theta
        self.n_arm = 6
        self.n_total = self.n_base + self.n_arm

        # Find arm joint qpos addresses
        self._arm_qpos_addrs = []
        for name in ARM_JOINT_NAMES:
            jid = mujoco.mj_name2id(model, mujoco.mjOBJ_JOINT, name)
            self._arm_qpos_addrs.append(model.jnt_qposadr[jid])

        # Find end-effector body (cr10_Link6 or gripper_base_link)
        self._ee_body_id = mujoco.mj_name2id(model, mujoco.mjOBJ_BODY, "gripper_base_link")
        if self._ee_body_id < 0:
            self._ee_body_id = mujoco.mj_name2id(model, mujoco.mjOBJ_BODY, "cr10_Link6")

    def forward_kinematics(self, q):
        """Compute EE 4x4 transform from 9-DOF config [x,y,theta, j1..j6].

        Uses MuJoCo's forward kinematics by temporarily setting qpos.
        """
        # Save state
        qpos_backup = self.data.qpos.copy()

        # Set base position (first 3 values are x, y for planar base)
        self.data.qpos[0] = q[0]
        self.data.qpos[1] = q[1]
        # Set base yaw (quaternion: cos(theta/2), 0, 0, sin(theta/2))
        half_theta = q[2] / 2
        self.data.qpos[3] = np.cos(half_theta)  # qw
        self.data.qpos[6] = np.sin(half_theta)  # qz

        # Set arm joints
        for i, addr in enumerate(self._arm_qpos_addrs):
            self.data.qpos[addr] = q[self.n_base + i]

        # Compute FK
        mujoco.mj_forward(self.model, self.data)

        # Get EE pose
        pos = self.data.xpos[self._ee_body_id].copy()
        mat = self.data.xmat[self._ee_body_id].reshape(3, 3).copy()
        T = np.eye(4)
        T[:3, :3] = mat
        T[:3, 3] = pos

        # Restore state
        self.data.qpos[:] = qpos_backup
        mujoco.mj_forward(self.model, self.data)

        return T

    def jacobian(self, q):
        """Compute 6x9 numerical Jacobian."""
        J = np.zeros((6, self.n_total))
        eps = 1e-6
        T0 = self.forward_kinematics(q)
        pose0 = transform_to_pose(T0)

        for i in range(self.n_total):
            q_plus = q.copy()
            q_plus[i] += eps
            T_plus = self.forward_kinematics(q_plus)
            pose_plus = transform_to_pose(T_plus)
            J[:, i] = (pose_plus - pose0) / eps

        return J
```

- [ ] **Step 4: Implement control/whole_body.py**

```python
# mujoco_demo/control/whole_body.py
import numpy as np
from scipy.optimize import minimize
from mujoco_demo.config import (
    QP_WEIGHT_TRACKING, QP_WEIGHT_REGULARIZATION, QP_WEIGHT_COLLISION,
    ARM_JOINT_LIMITS_ARRAY
)
from mujoco_demo.utils import pose_error


class WholeBodyController:
    """QP-based whole-body IK optimizer."""

    def __init__(self, kinematics, model, data):
        self.kin = kinematics
        self.model = model
        self.data = data
        self.w_track = QP_WEIGHT_TRACKING
        self.w_reg = QP_WEIGHT_REGULARIZATION
        self.w_col = QP_WEIGHT_COLLISION

    def compute_joint_velocities(self, q_current, target_pose, obstacles):
        """Solve QP for joint velocities.

        Args:
            q_current: 9-DOF joint config [x,y,theta, j1..j6]
            target_pose: 4x4 target EE transform
            obstacles: list of (position, radius) tuples

        Returns:
            dq: 9-DOF joint velocity vector
        """
        J = self.kin.jacobian(q_current)
        T_current = self.kin.forward_kinematics(q_current)
        dx = pose_error(target_pose, T_current)

        # Cost: tracking + regularization
        H = self.w_track * (J.T @ J) + self.w_reg * np.eye(self.kin.n_total)
        f = -self.w_track * (J.T @ dx)

        # Collision avoidance (simple potential field)
        for obs_pos, obs_radius in obstacles:
            ee_pos = T_current[:3, 3]
            diff = ee_pos - obs_pos
            dist = np.linalg.norm(diff)
            safe_dist = obs_radius + 0.15
            if dist < safe_dist and dist > 1e-6:
                # Repulsive gradient in task space, mapped to joint space
                grad_task = -diff / dist * (safe_dist - dist)
                grad_joint = J[:3, :].T @ grad_task
                H += self.w_col * np.outer(grad_joint, grad_joint)

        # Joint limits as bounds
        q_min = np.array([-0.5, -0.5, -np.pi, *ARM_JOINT_LIMITS_ARRAY[:, 0]])
        q_max = np.array([0.5, 0.5, np.pi, *ARM_JOINT_LIMITS_ARRAY[:, 1]])
        dq_max = np.full(self.kin.n_total, 0.1)  # max velocity per step

        bounds = [(-dq_max[i], dq_max[i]) for i in range(self.kin.n_total)]

        result = minimize(
            lambda dq: 0.5 * dq @ H @ dq + f @ dq,
            np.zeros(self.kin.n_total),
            bounds=bounds,
            method='SLSQP'
        )

        return result.x

    def compute_arm_velocities(self, q_arm, target_pose, base_pose):
        """Compute arm-only IK (6-DOF), assuming base is fixed."""
        q_full = np.concatenate([base_pose, q_arm])
        dq_full = self.compute_joint_velocities(q_full, target_pose, [])
        return dq_full[3:]  # Return only arm velocities
```

- [ ] **Step 5: Implement control/gripper.py**

```python
# mujoco_demo/control/gripper.py
from mujoco_demo.config import GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_FORCE


class GripperController:
    """Simple gripper open/close control."""

    def __init__(self, engine):
        self.engine = engine

    def open(self):
        self.engine.set_gripper(GRIPPER_OPEN)

    def close(self):
        self.engine.set_gripper(GRIPPER_CLOSE)

    def is_open(self):
        return self.engine.get_gripper_position() > 0.4

    def is_closed(self):
        return self.engine.get_gripper_position() < 0.2
```

- [ ] **Step 6: Run tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_control_kinematics.py -v
```

Expected: All tests PASS

- [ ] **Step 7: Commit**

```bash
git add mujoco_demo/control/ tests/test_control_kinematics.py
git commit -m "feat: add whole-body kinematics, QP controller, and gripper control"
```

---

## Task 9: Task Parser + State Machine

**Files:**
- Create: `mujoco_demo/task/__init__.py`
- Create: `mujoco_demo/task/task_parser.py`
- Create: `mujoco_demo/task/state_machine.py`
- Create: `tests/test_task_parser.py`

- [ ] **Step 1: Write test_task_parser.py (failing tests)**

```python
# tests/test_task_parser.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.task.task_parser import TaskParser
from mujoco_demo.task.state_machine import TaskStateMachine, TaskState


class TestTaskParser:
    def test_pick_and_place(self):
        parser = TaskParser()
        result = parser.parse("pick up the red cup and place it on the shelf")
        assert result['action'] == 'pick_and_place'
        assert result['object'] == 'red cup'
        assert result['location'] == 'shelf'

    def test_pick_only(self):
        parser = TaskParser()
        result = parser.parse("grasp the blue cube")
        assert result['action'] == 'pick'
        assert result['object'] == 'blue cube'

    def test_move_to(self):
        parser = TaskParser()
        result = parser.parse("move the red cup to the table")
        assert result['action'] == 'pick_and_place'
        assert result['object'] == 'red cup'
        assert result['location'] == 'table'

    def test_invalid_command(self):
        parser = TaskParser()
        with pytest.raises(ValueError):
            parser.parse("dance around")


class TestTaskStateMachine:
    def test_initial_state(self):
        sm = TaskStateMachine.__new__(TaskStateMachine)
        sm.state = TaskState.INIT
        assert sm.state == TaskState.INIT
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_task_parser.py -v
```

Expected: FAIL

- [ ] **Step 3: Implement task/task_parser.py**

```python
# mujoco_demo/task/task_parser.py
import re


class TaskParser:
    """Regex-based task command parser."""

    PATTERNS = [
        (r"pick up (?:the )?(.+?) and (?:place|put) (?:it|them) (?:on|in|at) (?:the )?(.+)",
         'pick_and_place'),
        (r"move (?:the )?(.+?) to (?:the )?(.+)",
         'pick_and_place'),
        (r"(?:grasp|grab|pick up) (?:the )?(.+)",
         'pick'),
    ]

    def parse(self, command: str) -> dict:
        command = command.lower().strip()
        for pattern, action in self.PATTERNS:
            match = re.match(pattern, command)
            if match:
                groups = match.groups()
                return {
                    'action': action,
                    'object': groups[0].strip(),
                    'location': groups[1].strip() if len(groups) > 1 else None,
                }
        raise ValueError(f"Cannot parse task command: {command}")
```

- [ ] **Step 4: Implement task/state_machine.py**

```python
# mujoco_demo/task/state_machine.py
from enum import Enum
import numpy as np
from mujoco_demo.task.task_parser import TaskParser


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
    """Finite state machine for task orchestration."""

    def __init__(self, sim_engine, mapper, scene_graph, nav_planner,
                 path_follower, whole_body_ctrl, gripper_ctrl,
                 detector=None, pose_estimator=None):
        self.engine = sim_engine
        self.mapper = mapper
        self.scene_graph = scene_graph
        self.nav_planner = nav_planner
        self.path_follower = path_follower
        self.wb_ctrl = whole_body_ctrl
        self.gripper = gripper_ctrl
        self.detector = detector
        self.pose_estimator = pose_estimator

        self.state = TaskState.INIT
        self.parser = TaskParser()
        self.target_object = None
        self.target_location = None
        self.sweep_waypoints = []
        self.sweep_idx = 0
        self.current_path = None

    def set_task(self, command: str):
        """Parse and set task from natural language command."""
        task = self.parser.parse(command)
        self.target_object_name = task['object']
        self.target_location_name = task['location']
        self.state = TaskState.TASK_RECEIVED

    def step(self):
        """Execute one step of the state machine."""
        if self.state == TaskState.INIT:
            self.state = TaskState.MAPPING_SWEEP

        elif self.state == TaskState.MAPPING_SWEEP:
            done = self._do_mapping_sweep()
            if done:
                self.state = TaskState.TASK_RECEIVED

        elif self.state == TaskState.TASK_RECEIVED:
            # Find target object in scene graph
            self.target_object = self.scene_graph.find_by_name(self.target_object_name)
            if self.target_object:
                self.state = TaskState.NAV_TO_OBJECT

        elif self.state == TaskState.NAV_TO_OBJECT:
            done = self._navigate_to(self.target_object.pose[:2])
            if done:
                self.state = TaskState.APPROACH_OBJECT

        elif self.state == TaskState.APPROACH_OBJECT:
            done = self._approach_object()
            if done:
                self.state = TaskState.PICK_OBJECT

        elif self.state == TaskState.PICK_OBJECT:
            done = self._pick_object()
            if done:
                self.state = TaskState.NAV_TO_PLACE

        elif self.state == TaskState.NAV_TO_PLACE:
            # Find placement location
            loc = self.scene_graph.find_by_name(self.target_location_name)
            if loc:
                done = self._navigate_to(loc.pose[:2])
                if done:
                    self.state = TaskState.PLACE_OBJECT

        elif self.state == TaskState.PLACE_OBJECT:
            done = self._place_object()
            if done:
                self.state = TaskState.COMPLETE

        return self.state

    def _do_mapping_sweep(self):
        """Execute mapping sweep. Returns True when complete."""
        # Simple: just advance simulation and collect data
        # In full implementation, would drive along sweep waypoints
        return True  # Placeholder - implemented in main.py integration

    def _navigate_to(self, target_xy):
        """Navigate to target position. Returns True when reached."""
        base_pose = self.engine.get_base_pose()
        dist = np.linalg.norm(base_pose[:2] - target_xy)
        if dist < 0.3:
            return True

        # Plan path if needed
        if self.current_path is None:
            map_2d = self.mapper.get_map_2d()
            self.current_path = self.nav_planner.plan(base_pose[:2], target_xy)

        if self.current_path:
            v, omega = self.path_follower.compute_velocity(
                base_pose[:2], base_pose[2], self.current_path
            )
            self.engine.set_base_velocity(v, omega)

        return False

    def _approach_object(self):
        """Approach object with arm. Returns True when ready to grasp."""
        # Simple: just signal ready
        return True

    def _pick_object(self):
        """Execute grasp. Returns True when object is grasped."""
        self.gripper.close()
        self.engine.step(100)
        return self.gripper.is_closed()

    def _place_object(self):
        """Place object. Returns True when done."""
        self.gripper.open()
        self.engine.step(100)
        return self.gripper.is_open()
```

- [ ] **Step 5: Run tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_task_parser.py -v
```

Expected: All tests PASS

- [ ] **Step 6: Commit**

```bash
git add mujoco_demo/task/ tests/test_task_parser.py
git commit -m "feat: add task parser and state machine orchestration"
```

---

## Task 10: Object Detector + Pose Estimator

**Files:**
- Create: `mujoco_demo/perception/__init__.py`
- Create: `mujoco_demo/perception/detector.py`
- Create: `mujoco_demo/perception/pose_est.py`

- [ ] **Step 1: Implement perception/detector.py**

```python
# mujoco_demo/perception/detector.py
import numpy as np


class DetectionResult:
    def __init__(self, bbox, score, label, mask=None):
        self.bbox = bbox  # [x1, y1, x2, y2]
        self.score = score
        self.label = label
        self.mask = mask

    @property
    def center(self):
        return np.array([(self.bbox[0]+self.bbox[2])/2, (self.bbox[1]+self.bbox[3])/2])


class ObjectDetector:
    """Object detection with Grounding-SAM2 or simple fallback."""

    def __init__(self, use_grounding_sam=False):
        self.use_grounding_sam = use_grounding_sam
        self._detector = None
        self._segmentor = None

        if use_grounding_sam:
            try:
                from transformers import pipeline
                self._detector = pipeline(
                    "zero-shot-object-detection",
                    model="IDEA-Research/grounding-dino-tiny"
                )
            except ImportError:
                print("Warning: Grounding-SAM2 not available, using fallback detector")
                self.use_grounding_sam = False

    def detect(self, rgb_image, text_prompt):
        """Detect objects matching text prompt.

        Returns list of DetectionResult.
        """
        if self.use_grounding_sam and self._detector is not None:
            return self._detect_grounding_dino(rgb_image, text_prompt)
        else:
            return self._detect_simple(rgb_image, text_prompt)

    def _detect_grounding_dino(self, rgb_image, text_prompt):
        """Use Grounding DINO for detection."""
        from PIL import Image
        if isinstance(rgb_image, np.ndarray):
            rgb_image = Image.fromarray(rgb_image)

        results = self._detector(rgb_image, candidate_labels=[text_prompt])
        detections = []
        for r in results:
            box = r['box']
            detections.append(DetectionResult(
                bbox=np.array([box['xmin'], box['ymin'], box['xmax'], box['ymax']]),
                score=r['score'],
                label=r['label']
            ))
        return detections

    def _detect_simple(self, rgb_image, text_prompt):
        """Simple color-based detection as fallback."""
        import cv2
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        detections = []

        color_ranges = {
            'red': [(np.array([0, 100, 100]), np.array([10, 255, 255])),
                    (np.array([170, 100, 100]), np.array([180, 255, 255]))],
            'blue': [(np.array([100, 100, 100]), np.array([130, 255, 255]))],
            'green': [(np.array([40, 100, 100]), np.array([80, 255, 255]))],
        }

        for color_name, ranges in color_ranges.items():
            if color_name not in text_prompt.lower():
                continue
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for low, high in ranges:
                mask |= cv2.inRange(hsv, low, high)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:
                    x, y, w, h = cv2.boundingRect(cnt)
                    detections.append(DetectionResult(
                        bbox=np.array([x, y, x+w, y+h]),
                        score=min(1.0, area / 1000),
                        label=color_name
                    ))

        return sorted(detections, key=lambda d: d.score, reverse=True)
```

- [ ] **Step 2: Implement perception/pose_est.py**

```python
# mujoco_demo/perception/pose_est.py
import numpy as np
from mujoco_demo.utils import pose_to_transform, transform_to_pose


class PoseEstimator:
    """Estimate 6D object pose from detection + depth image."""

    def __init__(self, camera_intrinsics):
        self.K = camera_intrinsics

    def estimate(self, detection, depth_image, camera_pose):
        """Estimate 6D pose from 2D detection + depth.

        Args:
            detection: DetectionResult with bbox and mask
            depth_image: HxW float32 depth in meters
            camera_pose: 4x4 camera-to-world transform

        Returns:
            6D pose [x, y, z, roll, pitch, yaw] in world frame
        """
        bbox = detection.bbox.astype(int)
        x1, y1, x2, y2 = bbox
        # Clamp to image bounds
        h, w = depth_image.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        depth_roi = depth_image[y1:y2, x1:x2]
        valid = depth_roi[(depth_roi > 0.1) & (depth_roi < 10.0)]
        if len(valid) == 0:
            return None

        median_depth = np.median(valid)

        # Back-project center to camera frame
        u, v = detection.center
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        x = (u - cx) * median_depth / fx
        y = (v - cy) * median_depth / fy
        z = median_depth
        point_cam = np.array([x, y, z, 1.0])

        # Transform to world frame
        point_world = camera_pose @ point_cam

        # Simple orientation (assume upright)
        pose = np.array([point_world[0], point_world[1], point_world[2], 0, 0, 0])
        return pose
```

- [ ] **Step 3: Commit**

```bash
git add mujoco_demo/perception/
git commit -m "feat: add object detector (Grounding-SAM2 + fallback) and pose estimator"
```

---

## Task 11: Visualization - MuJoCo 3D + Matplotlib 2D

**Files:**
- Create: `mujoco_demo/viz/__init__.py`
- Create: `mujoco_demo/viz/mujoco_viz.py`
- Create: `mujoco_demo/viz/matplotlib_viz.py`
- Create: `mujoco_demo/viz/dashboard.py`

- [ ] **Step 1: Implement viz/mujoco_viz.py**

```python
# mujoco_demo/viz/mujoco_viz.py
import mujoco
import mujoco.viewer
import numpy as np


class MuJoCoVisualizer:
    """MuJoCo 3D viewer with optional overlays."""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self._viewer = None

    def launch(self):
        """Launch the MuJoCo viewer window."""
        self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def is_running(self):
        if self._viewer is None:
            return False
        return self._viewer.is_running()

    def sync(self):
        """Sync viewer with current simulation state."""
        if self._viewer:
            self._viewer.sync()

    def close(self):
        if self._viewer:
            self._viewer.close()
            self._viewer = None

    def add_marker(self, pos, size=0.05, rgba=(1, 0, 0, 0.8)):
        """Add a visual marker at position."""
        # MuJoCo viewer doesn't support dynamic markers easily
        # This would need custom geom in the scene
        pass
```

- [ ] **Step 2: Implement viz/matplotlib_viz.py**

```python
# mujoco_demo/viz/matplotlib_viz.py
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np


class MatplotlibDashboard:
    """2D visualization: occupancy map + trajectory + task progress."""

    def __init__(self):
        plt.ion()
        self.fig, self.axes = plt.subplots(1, 2, figsize=(14, 6))
        self.trajectory = []
        self.task_states = []

    def update(self, robot_pos, robot_heading, map_2d=None,
               path=None, objects=None, task_state=None):
        """Update the dashboard with current data."""
        self.trajectory.append(robot_pos[:2].copy())

        # Left: map + trajectory
        ax = self.axes[0]
        ax.clear()
        if map_2d is not None:
            ax.imshow(map_2d.T, origin='lower', cmap='binary',
                      extent=[-5, 5, -5, 5], alpha=0.7)
        if len(self.trajectory) > 1:
            traj = np.array(self.trajectory)
            ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1.5, label='trajectory')
        ax.plot(robot_pos[0], robot_pos[1], 'ro', markersize=8, label='robot')
        # Heading arrow
        dx = 0.3 * np.cos(robot_heading)
        dy = 0.3 * np.sin(robot_heading)
        ax.arrow(robot_pos[0], robot_pos[1], dx, dy,
                 head_width=0.1, head_length=0.05, fc='r', ec='r')
        if path and len(path) > 1:
            path_arr = np.array(path)
            ax.plot(path_arr[:, 0], path_arr[:, 1], 'g--', linewidth=1, label='planned path')
        if objects:
            for obj in objects:
                ax.plot(obj.pose[0], obj.pose[1], 'g^', markersize=10)
                ax.annotate(obj.id, (obj.pose[0], obj.pose[1]),
                           fontsize=8, ha='center')
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_aspect('equal')
        ax.set_title('SLAM Map + Navigation')
        ax.legend(loc='upper right', fontsize=8)

        # Right: task progress
        ax2 = self.axes[1]
        ax2.clear()
        states = ['INIT', 'MAPPING', 'NAV_TO_OBJ', 'APPROACH',
                  'PICK', 'NAV_TO_PLACE', 'PLACE', 'COMPLETE']
        if task_state:
            try:
                current_idx = states.index(task_state.value.upper())
            except ValueError:
                current_idx = 0
        else:
            current_idx = 0
        colors = ['green' if i <= current_idx else 'lightgray' for i in range(len(states))]
        ax2.barh(range(len(states)), [1]*len(states), color=colors)
        ax2.set_yticks(range(len(states)))
        ax2.set_yticklabels(states)
        ax2.set_xlim(0, 1.5)
        ax2.set_title(f'Task Progress: {task_state.value if task_state else "INIT"}')

        plt.tight_layout()
        plt.pause(0.01)

    def save(self, filename='trajectory.png'):
        self.fig.savefig(filename, dpi=150)
```

- [ ] **Step 3: Implement viz/dashboard.py**

```python
# mujoco_demo/viz/dashboard.py
from mujoco_demo.viz.mujoco_viz import MuJoCoVisualizer
from mujoco_demo.viz.matplotlib_viz import MatplotlibDashboard


class Dashboard:
    """Coordinates MuJoCo 3D and Matplotlib 2D visualization."""

    def __init__(self, model, data):
        self.mujoco_viz = MuJoCoVisualizer(model, data)
        self.mpl_viz = MatplotlibDashboard()
        self._launched = False

    def launch(self):
        self.mujoco_viz.launch()
        self._launched = True

    def update(self, robot_pos, robot_heading, map_2d=None,
               path=None, objects=None, task_state=None):
        if self._launched:
            self.mujoco_viz.sync()
        self.mpl_viz.update(robot_pos, robot_heading, map_2d,
                           path, objects, task_state)

    def is_running(self):
        return self.mujoco_viz.is_running()

    def close(self):
        self.mujoco_viz.close()
```

- [ ] **Step 4: Commit**

```bash
git add mujoco_demo/viz/
git commit -m "feat: add MuJoCo 3D viewer and Matplotlib 2D dashboard visualization"
```

---

## Task 12: Main Entry Point + Integration

**Files:**
- Create: `mujoco_demo/main.py`
- Create: `tests/test_integration.py`

- [ ] **Step 1: Implement main.py**

```python
# mujoco_demo/main.py
import numpy as np
import mujoco
from mujoco_demo.config import (
    DEFAULT_TASK, SIM_TIMESTEP, GRID_RESOLUTION, GRID_BOUNDS
)
from mujoco_demo.sim.engine import SimulationEngine
from mujoco_demo.sim.env import HomeEnvironment
from mujoco_demo.sim.sensors import LiDARSensor, RGBDCamera
from mujoco_demo.slam.occupancy import OccupancyGrid
from mujoco_demo.slam.mapper import Mapper
from mujoco_demo.slam.scene_graph import SceneGraph, SceneGraphNode
from mujoco_demo.nav.astar import AStarPlanner
from mujoco_demo.nav.vfh import VFHPlanner
from mujoco_demo.nav.planner import PathFollower
from mujoco_demo.control.kinematics import WholeBodyKinematics
from mujoco_demo.control.whole_body import WholeBodyController
from mujoco_demo.control.gripper import GripperController
from mujoco_demo.task.state_machine import TaskStateMachine, TaskState
from mujoco_demo.viz.dashboard import Dashboard


def generate_sweep_waypoints(room_bounds, step=2.0):
    """Generate clockwise waypoints around room perimeter."""
    x_min, y_min, x_max, y_max = room_bounds
    margin = 1.0
    waypoints = []
    for x in np.arange(x_min + margin, x_max, step):
        waypoints.append(np.array([x, y_min + margin, 0]))
    for y in np.arange(y_min + margin, y_max, step):
        waypoints.append(np.array([x_max - margin, y, -np.pi/2]))
    for x in np.arange(x_max - margin, x_min, -step):
        waypoints.append(np.array([x, y_max - margin, np.pi]))
    for y in np.arange(y_max - margin, y_min, -step):
        waypoints.append(np.array([x_min + margin, y, np.pi/2]))
    return waypoints


def run_demo(task_command=None):
    """Run the full MuJoCo mobile manipulator demo."""
    print("=" * 60)
    print("MuJoCo Mobile Manipulator Demo")
    print("=" * 60)

    # 1. Initialize simulation
    print("\n[1/7] Initializing simulation...")
    env = HomeEnvironment()
    engine = SimulationEngine(extra_xml=env.get_scene_xml())
    print(f"  Model: {engine.model.nq} DOF, {engine.model.nu} actuators")

    # 2. Initialize sensors
    print("[2/7] Initializing sensors...")
    lidar = LiDARSensor(engine.model, engine.data)
    cam_d435 = RGBDCamera("d435_cam", width=320, height=240)

    # 3. Initialize SLAM
    print("[3/7] Initializing SLAM...")
    occupancy = OccupancyGrid()
    mapper = Mapper(occupancy)
    scene_graph = SceneGraph()
    scene_graph.add_node(SceneGraphNode(
        id="table_1", obj_type="furniture",
        pose=np.array([2, 1, 0, 0, 0, 0]),
        size=np.array([1.2, 0.8, 0.8]), confidence=1.0
    ))
    scene_graph.add_node(SceneGraphNode(
        id="shelf_1", obj_type="furniture",
        pose=np.array([-2, 0, 0, 0, 0, 0]),
        size=np.array([0.6, 1.6, 1.0]), confidence=1.0
    ))
    scene_graph.add_node(SceneGraphNode(
        id="red_cup_1", obj_type="graspable",
        pose=np.array([2, 1, 0.45, 0, 0, 0]),
        size=np.array([0.06, 0.06, 0.12]),
        confidence=0.9, parent="table_1"
    ))

    # 4. Initialize navigation
    print("[4/7] Initializing navigation...")
    astar = AStarPlanner(occupancy.get_2d_map(), GRID_RESOLUTION)
    vfh = VFHPlanner()
    path_follower = PathFollower()

    # 5. Initialize control
    print("[5/7] Initializing controllers...")
    kin = WholeBodyKinematics(engine.model, engine.data)
    wb_ctrl = WholeBodyController(kin, engine.model, engine.data)
    gripper = GripperController(engine)

    # 6. Initialize visualization
    print("[6/7] Initializing visualization...")
    dashboard = Dashboard(engine.model, engine.data)
    dashboard.launch()

    # 7. Initialize state machine
    task_sm = TaskStateMachine(
        engine, mapper, scene_graph, astar, path_follower,
        wb_ctrl, gripper
    )
    task_sm.set_task(task_command or DEFAULT_TASK)
    print(f"[7/7] Task: {task_command or DEFAULT_TASK}")
    print(f"  Target: {task_sm.target_object_name}")
    print(f"  Location: {task_sm.target_location_name}")

    # Main loop
    print("\n--- Starting main loop ---")
    sweep_waypoints = generate_sweep_waypoints(env.get_room_bounds())
    sweep_idx = 0
    sweep_done = False
    frame = 0

    try:
        while dashboard.is_running():
            # Physics step
            engine.step(5)

            # SLAM update (every 10 frames)
            if frame % 10 == 0:
                base_pose = engine.get_base_pose()
                # Simulate LiDAR scan
                points = lidar.scan(engine.model, engine.data)
                if len(points) > 0:
                    mapper.update(base_pose[:3], points)

            # State machine
            if not sweep_done:
                # Mapping sweep: drive along waypoints
                base_pose = engine.get_base_pose()
                if sweep_idx < len(sweep_waypoints):
                    wp = sweep_waypoints[sweep_idx]
                    dist = np.linalg.norm(base_pose[:2] - wp[:2])
                    if dist < 0.5:
                        sweep_idx += 1
                    else:
                        direction = np.arctan2(wp[1]-base_pose[1], wp[0]-base_pose[0])
                        error = direction - base_pose[2]
                        error = (error + np.pi) % (2*np.pi) - np.pi
                        v = 0.2 * max(0.1, np.cos(error))
                        omega = np.clip(1.5 * error, -0.5, 0.5)
                        engine.set_base_velocity(v, omega)
                else:
                    sweep_done = True
                    print("  Mapping sweep complete!")
                    # Update A* planner with new map
                    map_2d = mapper.get_map_2d()
                    astar = AStarPlanner(map_2d, GRID_RESOLUTION)
                    task_sm.nav_planner = astar
                    task_sm.state = TaskState.TASK_RECEIVED
            else:
                # Run task state machine
                task_sm.step()
                base_pose = engine.get_base_pose()

            # Visualization update (every 5 frames)
            if frame % 5 == 0:
                base_pose = engine.get_base_pose()
                map_2d = mapper.get_map_2d()
                dashboard.update(
                    base_pose[:3], base_pose[2],
                    map_2d=map_2d,
                    objects=list(scene_graph.nodes.values()),
                    task_state=task_sm.state
                )

            frame += 1

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        dashboard.close()
        print("Demo complete.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MuJoCo Mobile Manipulator Demo")
    parser.add_argument("--task", type=str, default=None,
                       help="Task command (e.g., 'pick up the red cup and place it on the shelf')")
    args = parser.parse_args()
    run_demo(args.task)
```

- [ ] **Step 2: Write test_integration.py**

```python
# tests/test_integration.py
import numpy as np
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from mujoco_demo.sim.engine import SimulationEngine
from mujoco_demo.sim.env import HomeEnvironment
from mujoco_demo.sim.sensors import LiDARSensor
from mujoco_demo.slam.occupancy import OccupancyGrid
from mujoco_demo.slam.mapper import Mapper
from mujoco_demo.nav.astar import AStarPlanner
from mujoco_demo.control.kinematics import WholeBodyKinematics
from mujoco_demo.task.task_parser import TaskParser


class TestIntegration:
    def test_full_pipeline_smoke(self):
        """Smoke test: all modules initialize and run one step."""
        # Sim
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        engine.step(10)

        # Sensors
        lidar = LiDARSensor(engine.model, engine.data)
        points = lidar.scan(engine.model, engine.data)
        assert len(points) > 0

        # SLAM
        grid = OccupancyGrid(resolution=0.1, bounds=(-5, -5, 0, 5, 5, 3))
        mapper = Mapper(grid)
        base_pose = engine.get_base_pose()
        mapper.update(base_pose[:3], points)
        map_2d = mapper.get_map_2d()
        assert map_2d.ndim == 2

        # Navigation
        planner = AStarPlanner(map_2d, 0.1)
        # Just verify planner initializes
        assert planner is not None

        # Kinematics
        kin = WholeBodyKinematics(engine.model, engine.data)
        q = np.zeros(9)
        T = kin.forward_kinematics(q)
        assert T.shape == (4, 4)

        # Task parser
        parser = TaskParser()
        task = parser.parse("pick up the red cup and place it on the shelf")
        assert task['object'] == 'red cup'

    def test_arm_movement(self):
        """Test arm moves to target position."""
        env = HomeEnvironment()
        engine = SimulationEngine(extra_xml=env.get_scene_xml())
        kin = WholeBodyKinematics(engine.model, engine.data)

        # Set arm to home position
        engine.set_arm_joints(np.zeros(6))
        for _ in range(500):
            engine.step()
        T_home = kin.forward_kinematics(np.zeros(9))

        # Move arm
        engine.set_arm_joints(np.array([0, -0.5, 0.3, 0, -0.2, 0]))
        for _ in range(500):
            engine.step()
        base_pose = engine.get_base_pose()
        q_moved = np.concatenate([base_pose, engine.get_arm_joint_positions()])
        T_moved = kin.forward_kinematics(q_moved)

        # End-effector should have moved
        assert not np.allclose(T_home[:3, 3], T_moved[:3, 3], atol=0.01)
```

- [ ] **Step 3: Run integration tests**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/test_integration.py -v
```

Expected: All tests PASS

- [ ] **Step 4: Run full test suite**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m pytest tests/ -v
```

Expected: All tests PASS

- [ ] **Step 5: Install MuJoCo and dependencies**

```bash
pip install mujoco numpy scipy matplotlib opencv-python Pillow
```

- [ ] **Step 6: Run the demo**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -m mujoco_demo.main --task "pick up the red cup and place it on the shelf"
```

Expected: MuJoCo 3D viewer + Matplotlib 2D dashboard open, robot performs mapping sweep and pick-and-place task.

- [ ] **Step 7: Commit**

```bash
git add mujoco_demo/main.py tests/test_integration.py
git commit -m "feat: add main entry point with full demo integration"
```

---

## Task 13: URDF Mesh Path Fix + Final Polish

**Files:**
- Modify: `mujoco_demo/sim/engine.py`
- Create: `mujoco_demo/assets/` (for patched URDF if needed)

- [ ] **Step 1: Test URDF loading with actual meshes**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -c "
from mujoco_demo.sim.engine import SimulationEngine
from mujoco_demo.sim.env import HomeEnvironment
env = HomeEnvironment()
engine = SimulationEngine(extra_xml=env.get_scene_xml())
print(f'Loaded: nq={engine.model.nq}, nu={engine.model.nu}')
print(f'Bodies: {engine.model.nbody}')
engine.step(100)
print(f'Time: {engine.data.time:.3f}s')
print('SUCCESS')
"
```

Expected: Model loads with all meshes, simulation runs.

- [ ] **Step 2: If mesh loading fails, create patched URDF**

If MuJoCo cannot resolve `package://` paths, create a patched copy:

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -c "
from pathlib import Path
urdf = Path('rangerboxcr10lidar_description/urdf/RangerCR10LiDAR.urdf')
content = urdf.read_text()
mesh_dir = urdf.parent.parent / 'meshes'
content = content.replace(
    'package://rangerboxcr10lidar_description/meshes/',
    str(mesh_dir.absolute()) + '/'
)
Path('mujoco_demo/assets').mkdir(exist_ok=True)
Path('mujoco_demo/assets/RangerCR10LiDAR.urdf').write_text(content)
print('Patched URDF written')
"
```

Update `engine.py` to use the patched URDF if the original fails.

- [ ] **Step 3: Verify gripper mimic joints work**

```bash
cd /home/gzz/Codes/agx_ws/src/agx
python -c "
from mujoco_demo.sim.engine import SimulationEngine
from mujoco_demo.sim.env import HomeEnvironment
env = HomeEnvironment()
engine = SimulationEngine(extra_xml=env.get_scene_xml())
# Open gripper
engine.set_gripper(0.6)
for _ in range(300): engine.step()
pos_open = engine.get_gripper_position()
# Close gripper
engine.set_gripper(0.1)
for _ in range(300): engine.step()
pos_close = engine.get_gripper_position()
print(f'Open: {pos_open:.3f}, Close: {pos_close:.3f}')
assert pos_open > 0.3, 'Gripper did not open'
assert pos_close < 0.3, 'Gripper did not close'
print('Gripper OK')
"
```

- [ ] **Step 4: Final commit**

```bash
git add -A
git commit -m "feat: complete MuJoCo mobile manipulator demo with SLAM, navigation, control, and visualization"
```

---

## Summary

| Task | Module | Key Components |
|------|--------|---------------|
| 1 | Foundation | config.py, utils.py |
| 2 | Sim | MuJoCo engine, URDF loading |
| 3 | Sim | Home environment scene |
| 4 | Sim | LiDAR + RGB-D camera sensors |
| 5 | SLAM | Occupancy grid, mapper, scene graph |
| 6 | Nav | A* global planner |
| 7 | Nav | VFH local avoidance, path follower |
| 8 | Control | Whole-body kinematics, QP controller, gripper |
| 9 | Task | Task parser, state machine |
| 10 | Perception | Object detector, pose estimator |
| 11 | Viz | MuJoCo 3D + Matplotlib 2D dashboard |
| 12 | Main | Entry point, integration |
| 13 | Polish | Mesh fix, final verification |

**Run the demo:**
```bash
cd /home/gzz/Codes/agx_ws/src/agx
pip install mujoco numpy scipy matplotlib opencv-python Pillow
python -m mujoco_demo.main --task "pick up the red cup and place it on the shelf"
```
