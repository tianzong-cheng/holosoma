# IRMV V3 Retargeting Support — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add IRMV V3 (21-DOF humanoid) retargeting support with LAFAN data format.

**Architecture:** Create model files under `models/irmv_v3/`, build MuJoCo XML from the existing URDF (with virtual toe/EE links + foot spheres), then register the robot in three config files (`robot.py`, `data_type.py`, `data_conversion.py`).

**Tech Stack:** Python, MuJoCo XML, LAFAN motion format, tyro config system

---

### Task 1: Scaffold model directory and copy meshes + URDF

**Files:**
- Create: `src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/meshes/` (22 STL files)
- Create: `src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.urdf`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/meshes
```

- [ ] **Step 2: Copy all 22 STL mesh files**

```bash
cp /home/tianzong/Workspace/UgradCapstone/robot_description/irmv/irmv-humanoid-v3-description/meshes/*.STL \
   src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/meshes/
```

Expected: 22 files copied (pelvis, torso_link, left/right hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll, left/right shoulder_pitch, shoulder_roll, shoulder_yaw, elbow)

- [ ] **Step 3: Verify file count**

```bash
ls src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/meshes/*.STL | wc -l
```

Expected: 22

- [ ] **Step 4: Copy URDF and fix mesh paths**

Read the source URDF:
```bash
wc -l /home/tianzong/Workspace/UgradCapstone/robot_description/irmv/irmv-humanoid-v3-description/urdf/irmv_v3.urdf
```

Use Python to copy the URDF, replacing `package://irmv_v3/meshes/` with `meshes/`:

```python
import re
src = "/home/tianzong/Workspace/UgradCapstone/robot_description/irmv/irmv-humanoid-v3-description/urdf/irmv_v3.urdf"
dst = "src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.urdf"
with open(src) as f:
    content = f.read()
content = content.replace("package://irmv_v3/meshes/", "meshes/")
with open(dst, "w") as f:
    f.write(content)
```

Run: `python -c "..."`  

- [ ] **Step 5: Verify URDF was copied**

```bash
head -5 src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.urdf
```

Expected: `<robot name="irmv_v3">` with mesh paths using `meshes/` not `package://`

- [ ] **Step 6: Commit**

```bash
git add src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/
git commit -m "feat: add IRMV V3 model files (URDF + meshes)"
```

---

### Task 2: Add IRMV V3 robot config to `config_types/robot.py`

**Files:**
- Modify: `src/holosoma_retargeting/holosoma_retargeting/config_types/robot.py:18-21` (_ROBOT_DEFAULTS dict)
- Modify: `src/holosoma_retargeting/holosoma_retargeting/config_types/robot.py:128-157` (_foot_sticking_links method)

- [ ] **Step 1: Add `_ROBOT_DEFAULTS` entry**

In `config_types/robot.py`, add after the `"t1"` entry on line 20:

```python
"irmv_v3": {"robot_dof": 21, "robot_height": 1.32, "object_name": "ground"},
```

- [ ] **Step 2: Add foot sticking links case**

In the `_foot_sticking_links` method, add before the `raise ValueError` (before line 157):

```python
if self.robot_type == "irmv_v3":
    return [
        "left_foot_sphere_1_link",
        "right_foot_sphere_1_link",
        "left_foot_sphere_2_link",
        "right_foot_sphere_2_link",
        "left_foot_sphere_3_link",
        "right_foot_sphere_3_link",
        "left_foot_sphere_4_link",
        "right_foot_sphere_4_link",
    ]
```

- [ ] **Step 3: Verify config loads**

```bash
source scripts/source_isaacsim_setup.sh && python -c "
from holosoma_retargeting.config_types.robot import RobotConfig
rc = RobotConfig(robot_type='irmv_v3')
print('DOF:', rc.ROBOT_DOF)
print('Height:', rc.ROBOT_HEIGHT)
print('Name:', rc.ROBOT_NAME)
print('URDF:', rc.ROBOT_URDF_FILE)
print('Foot links:', rc.FOOT_STICKING_LINKS)
"
```

Expected output:
```
DOF: 21
Height: 1.32
Name: irmv_v3_21dof
URDF: models/irmv_v3/irmv_v3_21dof.urdf
Foot links: ['left_foot_sphere_1_link', 'right_foot_sphere_1_link', ...]
```

- [ ] **Step 4: Verify invalid robot type still raises**

```bash
python -c "
from holosoma_retargeting.config_types.robot import RobotConfig
try:
    rc = RobotConfig(robot_type='nonexistent')
except ValueError as e:
    print('OK:', 'irmv_v3' in str(e))
"
```

Expected: `OK: True`

- [ ] **Step 5: Commit**

```bash
git add src/holosoma_retargeting/holosoma_retargeting/config_types/robot.py
git commit -m "feat: register IRMV V3 robot in config_types/robot.py"
```

---

### Task 3: Add joint mapping for LAFAN in `config_types/data_type.py`

**Files:**
- Modify: `src/holosoma_retargeting/holosoma_retargeting/config_types/data_type.py:177-297` (JOINTS_MAPPINGS dict)

- [ ] **Step 1: Add LAFAN→irmv_v3 mapping**

Add after the `("lafan", "t1")` entry (after line 211):

```python
("lafan", "irmv_v3"): {
    "Spine1": "pelvis",
    "LeftUpLeg": "left_hip_pitch_link",
    "LeftLeg": "left_knee_link",
    "LeftFoot": "left_ankle_pitch_link",
    "LeftToeBase": "left_toe_link",
    "RightUpLeg": "right_hip_pitch_link",
    "RightLeg": "right_knee_link",
    "RightFoot": "right_ankle_pitch_link",
    "RightToeBase": "right_toe_link",
    "LeftArm": "left_shoulder_roll_link",
    "LeftForeArm": "left_elbow_link",
    "LeftHand": "left_ee_link",
    "RightArm": "right_shoulder_roll_link",
    "RightForeArm": "right_elbow_link",
    "RightHand": "right_ee_link",
},
```

- [ ] **Step 2: Verify joint mapping resolves**

```bash
source scripts/source_isaacsim_setup.sh && python -c "
from holosoma_retargeting.config_types.data_type import MotionDataConfig
mc = MotionDataConfig(data_format='lafan', robot_type='irmv_v3')
print(mc.resolved_joints_mapping)
"
```

Expected: dict with 15 entries mapping LAFAN joints to IRMV V3 links.

- [ ] **Step 3: Verify missing mapping raises error**

```bash
python -c "
from holosoma_retargeting.config_types.data_type import MotionDataConfig
try:
    mc = MotionDataConfig(data_format='smplh', robot_type='irmv_v3')
    print(mc.resolved_joints_mapping)
except ValueError as e:
    print('OK: no mapping found for smplh+irmv_v3')
"
```

Expected: `OK: no mapping found for smplh+irmv_v3`

- [ ] **Step 4: Commit**

```bash
git add src/holosoma_retargeting/holosoma_retargeting/config_types/data_type.py
git commit -m "feat: add LAFAN joint mapping for IRMV V3"
```

---

### Task 4: Add joint names in `config_types/data_conversion.py`

**Files:**
- Modify: `src/holosoma_retargeting/holosoma_retargeting/config_types/data_conversion.py:10-42` (_ROBOT_JOINT_NAMES_DEFAULT dict)

- [ ] **Step 1: Add IRMV V3 joint name list**

Add after the `"g1"` entry (after line 41):

```python
"irmv_v3": [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "lefT_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
],
```

Note: The left hip yaw joint is spelled `lefT_hip_yaw_joint` (capital T) — matches the URDF exactly.

- [ ] **Step 2: Verify joint name list**

```bash
source scripts/source_isaacsim_setup.sh && python -c "
from holosoma_retargeting.config_types.data_conversion import DataConversionConfig
dc = DataConversionConfig(input_file='dummy.npz', robot='irmv_v3')
print(len(dc.JOINT_NAMES), 'joints')
print(dc.JOINT_NAMES[2])  # should be lefT_hip_yaw_joint
"
```

Expected:
```
21 joints
lefT_hip_yaw_joint
```

- [ ] **Step 3: Commit**

```bash
git add src/holosoma_retargeting/holosoma_retargeting/config_types/data_conversion.py
git commit -m "feat: add IRMV V3 joint names for data conversion"
```

---

### Task 5: Create MJCF XML model with virtual links

**Files:**
- Create: `src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.xml`

This is the main task — converting the URDF kinematic tree to MuJoCo XML format and adding virtual links (foot spheres, toes, arm end-effectors).

- [ ] **Step 1: Write the MJCF XML file**

Write the complete file `src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.xml` with the content shown below:

```xml
<mujoco model="irmv_v3_21dof">
  <compiler angle="radian" meshdir="meshes/"/>

  <asset>
    <mesh name="pelvis" content_type="model/stl" file="pelvis.STL"/>
    <mesh name="torso_link" content_type="model/stl" file="torso_link.STL"/>
    <mesh name="left_hip_pitch_link" content_type="model/stl" file="left_hip_pitch_link.STL"/>
    <mesh name="left_hip_roll_link" content_type="model/stl" file="left_hip_roll_link.STL"/>
    <mesh name="left_hip_yaw_link" content_type="model/stl" file="left_hip_yaw_link.STL"/>
    <mesh name="left_knee_link" content_type="model/stl" file="left_knee_link.STL"/>
    <mesh name="left_ankle_pitch_link" content_type="model/stl" file="left_ankle_pitch_link.STL"/>
    <mesh name="left_ankle_roll_link" content_type="model/stl" file="left_ankle_roll_link.STL"/>
    <mesh name="right_hip_pitch_link" content_type="model/stl" file="right_hip_pitch_link.STL"/>
    <mesh name="right_hip_roll_link" content_type="model/stl" file="right_hip_roll_link.STL"/>
    <mesh name="right_hip_yaw_link" content_type="model/stl" file="right_hip_yaw_link.STL"/>
    <mesh name="right_knee_link" content_type="model/stl" file="right_knee_link.STL"/>
    <mesh name="right_ankle_pitch_link" content_type="model/stl" file="right_ankle_pitch_link.STL"/>
    <mesh name="right_ankle_roll_link" content_type="model/stl" file="right_ankle_roll_link.STL"/>
    <mesh name="left_shoulder_pitch_link" content_type="model/stl" file="left_shoulder_pitch_link.STL"/>
    <mesh name="left_shoulder_roll_link" content_type="model/stl" file="left_shoulder_roll_link.STL"/>
    <mesh name="left_shoulder_yaw_link" content_type="model/stl" file="left_shoulder_yaw_link.STL"/>
    <mesh name="left_elbow_link" content_type="model/stl" file="left_elbow_link.STL"/>
    <mesh name="right_shoulder_pitch_link" content_type="model/stl" file="right_shoulder_pitch_link.STL"/>
    <mesh name="right_shoulder_roll_link" content_type="model/stl" file="right_shoulder_roll_link.STL"/>
    <mesh name="right_shoulder_yaw_link" content_type="model/stl" file="right_shoulder_yaw_link.STL"/>
    <mesh name="right_elbow_link" content_type="model/stl" file="right_elbow_link.STL"/>
  </asset>

  <worldbody>
    <geom name="ground" type="plane" size="10 10 0.1" pos="0 0 0" material="MatPlane"/>
    <light pos="0 0 1000" castshadow="true"/>

    <body name="pelvis">
      <inertial pos="0.00022355094 -3.798e-06 -0.032590765" mass="2.8716" diaginertia="0.0058066 0.00371554 0.00508252"/>
      <freejoint/>
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.2 0.2 0.2 1" mesh="pelvis"/>
      <geom name="pelvis" type="mesh" rgba="0.2 0.2 0.2 1" mesh="pelvis"/>

      <!-- LEFT LEG -->
      <body name="left_hip_pitch_link" pos="0 0.084 -0.048497">
        <inertial pos="0.013196 0.020825 -0.070145" mass="1.4898" diaginertia="0.00209208 0.00166555 0.00138175"/>
        <joint name="left_hip_pitch_joint" pos="0 0 0" axis="0 0.86603 -0.5" range="-3.14 3.14" actuatorfrcrange="-50 50"/>
        <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_hip_pitch_link"/>
        <geom name="left_hip_pitch_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_hip_pitch_link"/>
        <body name="left_hip_roll_link" pos="0 0.021 -0.071503">
          <inertial pos="0.0007051 1.548e-05 -0.090644" mass="1.3446" diaginertia="0.0014351 0.0013322 0.0016851"/>
          <joint name="left_hip_roll_joint" pos="0 0 0" axis="1 0 0" range="-1.0 3.14" actuatorfrcrange="-50 50"/>
          <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_hip_roll_link"/>
          <geom name="left_hip_roll_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_hip_roll_link"/>
          <body name="left_hip_yaw_link" pos="0 0 -0.080622">
            <inertial pos="0.00099516 -0.00571933 -0.157640" mass="1.9735" diaginertia="0.00402315 0.0047646 0.00231147"/>
            <joint name="lefT_hip_yaw_joint" pos="0 0 0" axis="0 0 1" range="-1.15 1.15" actuatorfrcrange="-50 50"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_hip_yaw_link"/>
            <geom name="left_hip_yaw_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_hip_yaw_link"/>
            <body name="left_knee_link" pos="0 0 -0.21">
              <inertial pos="-0.01109 0.0021963 -0.13238" mass="1.7957" diaginertia="0.0056257 0.00519812 0.00104611"/>
              <joint name="left_knee_joint" pos="0 0 0" axis="0 1 0" range="-1.0 3.14" actuatorfrcrange="-50 50"/>
              <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_knee_link"/>
              <geom name="left_knee_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_knee_link"/>
              <body name="left_ankle_pitch_link" pos="0 0 -0.32">
                <inertial pos="-2.24e-10 3.14e-10 -0.00686" mass="0.09967" diaginertia="6.6126e-06 9.7901e-06 9.9138e-06"/>
                <joint name="left_ankle_pitch_joint" pos="0 0 0" axis="0 1 0" range="-0.77 0.77"/>
                <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_ankle_pitch_link"/>
                <geom name="left_ankle_pitch_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_ankle_pitch_link"/>
                <body name="left_ankle_roll_link" pos="0 0 -0.012">
                  <inertial pos="0.03853 4.81e-09 -0.016612" mass="1.3601" diaginertia="0.0005894 0.0044946 0.00505498"/>
                  <joint name="left_ankle_roll_joint" pos="0 0 0" axis="1 0 0" range="-0.22 0.22"/>
                  <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.2 0.2 0.2 1" mesh="left_ankle_roll_link"/>
                  <geom name="left_ankle_roll_link" type="mesh" rgba="0.2 0.2 0.2 1" mesh="left_ankle_roll_link"/>
                  <!-- Foot contact spheres -->
                  <body name="left_foot_sphere_1_link" pos="-0.05 0.025 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="left_foot_sphere_1_link" type="sphere" size="0.005"/>
                  </body>
                  <body name="left_foot_sphere_2_link" pos="-0.05 -0.025 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="left_foot_sphere_2_link" type="sphere" size="0.005"/>
                  </body>
                  <body name="left_foot_sphere_3_link" pos="0.12 0.03 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="left_foot_sphere_3_link" type="sphere" size="0.005"/>
                  </body>
                  <body name="left_foot_sphere_4_link" pos="0.12 -0.03 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="left_foot_sphere_4_link" type="sphere" size="0.005"/>
                  </body>
                  <!-- Virtual toe link -->
                  <body name="left_toe_link" pos="0.14 0 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="left_toe_link" type="sphere" size="0.005"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>

      <!-- RIGHT LEG (mirrored from left) -->
      <body name="right_hip_pitch_link" pos="0 -0.084 -0.048497">
        <inertial pos="0.013196 -0.020806 -0.070145" mass="1.4898" diaginertia="0.00209209 0.00166556 0.00138175"/>
        <joint name="right_hip_pitch_joint" pos="0 0 0" axis="0 0.86603 0.5" range="-3.14 3.14" actuatorfrcrange="-50 50"/>
        <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_hip_pitch_link"/>
        <geom name="right_hip_pitch_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_hip_pitch_link"/>
        <body name="right_hip_roll_link" pos="0 -0.021 -0.071503">
          <inertial pos="0.00070549 6.37e-06 -0.090644" mass="1.3446" diaginertia="0.0014351 0.0013322 0.0016851"/>
          <joint name="right_hip_roll_joint" pos="0 0 0" axis="1 0 0" range="-3.14 1.0" actuatorfrcrange="-50 50"/>
          <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_hip_roll_link"/>
          <geom name="right_hip_roll_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_hip_roll_link"/>
          <body name="right_hip_yaw_link" pos="0 0 -0.080622">
            <inertial pos="0.00100888 0.00576995 -0.157638" mass="1.9735" diaginertia="0.00402315 0.0047646 0.00231148"/>
            <joint name="right_hip_yaw_joint" pos="0 0 0" axis="0 0 1" range="-1.15 1.15" actuatorfrcrange="-50 50"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_hip_yaw_link"/>
            <geom name="right_hip_yaw_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_hip_yaw_link"/>
            <body name="right_knee_link" pos="0 0 -0.21">
              <inertial pos="-0.01109 -0.002516 -0.13238" mass="1.7957" diaginertia="0.0056257 0.00519812 0.00104611"/>
              <joint name="right_knee_joint" pos="0 0 0" axis="0 1 0" range="-1.0 3.14" actuatorfrcrange="-50 50"/>
              <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_knee_link"/>
              <geom name="right_knee_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_knee_link"/>
              <body name="right_ankle_pitch_link" pos="0 0 -0.32">
                <inertial pos="-2.24e-10 3.14e-10 -0.00686" mass="0.09967" diaginertia="6.6126e-06 9.7901e-06 9.9138e-06"/>
                <joint name="right_ankle_pitch_joint" pos="0 0 0" axis="0 1 0" range="-0.77 0.77"/>
                <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_ankle_pitch_link"/>
                <geom name="right_ankle_pitch_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_ankle_pitch_link"/>
                <body name="right_ankle_roll_link" pos="0 0 -0.012">
                  <inertial pos="0.03853 4.81e-09 -0.016612" mass="1.3601" diaginertia="0.0005894 0.0044946 0.00505498"/>
                  <joint name="right_ankle_roll_joint" pos="0 0 0" axis="1 0 0" range="-0.22 0.22"/>
                  <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.2 0.2 0.2 1" mesh="right_ankle_roll_link"/>
                  <geom name="right_ankle_roll_link" type="mesh" rgba="0.2 0.2 0.2 1" mesh="right_ankle_roll_link"/>
                  <!-- Foot contact spheres -->
                  <body name="right_foot_sphere_1_link" pos="-0.05 0.025 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="right_foot_sphere_1_link" type="sphere" size="0.005"/>
                  </body>
                  <body name="right_foot_sphere_2_link" pos="-0.05 -0.025 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="right_foot_sphere_2_link" type="sphere" size="0.005"/>
                  </body>
                  <body name="right_foot_sphere_3_link" pos="0.12 0.03 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="right_foot_sphere_3_link" type="sphere" size="0.005"/>
                  </body>
                  <body name="right_foot_sphere_4_link" pos="0.12 -0.03 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="right_foot_sphere_4_link" type="sphere" size="0.005"/>
                  </body>
                  <!-- Virtual toe link -->
                  <body name="right_toe_link" pos="0.14 0 -0.03">
                    <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                    <geom type="sphere" size="0.005" rgba="0.2 0.2 0.2 1" contype="0" conaffinity="0" density="0" group="1"/>
                    <geom name="right_toe_link" type="sphere" size="0.005"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>

      <!-- WAIST & TORSO -->
      <body name="torso_link" pos="0 0 0.3">
        <inertial pos="4.68e-05 -1.29e-05 -0.067646" mass="7.3668" diaginertia="0.021922 0.021646 0.01512"/>
        <joint name="waist_yaw_joint" pos="0 0 0" axis="0 0 1" range="-3.14 3.14" actuatorfrcrange="-50 50"/>
        <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="torso_link"/>
        <geom name="torso_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="torso_link"/>

        <!-- LEFT ARM -->
        <body name="left_shoulder_pitch_link" pos="0 0.115 0">
          <inertial pos="-0.017031 0.04949 -0.0001267" mass="0.62913" diaginertia="0.00044347 0.00032046 0.00029486"/>
          <joint name="left_shoulder_pitch_joint" pos="0 0 0" axis="0 1 0" range="-3.14 3.14" actuatorfrcrange="-10 10"/>
          <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_shoulder_pitch_link"/>
          <geom name="left_shoulder_pitch_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_shoulder_pitch_link"/>
          <body name="left_shoulder_roll_link" pos="0 0.05 0">
            <inertial pos="0.0013403 -1.33e-05 -0.11572" mass="0.653" diaginertia="0.00039643 0.00038836 0.00046829"/>
            <joint name="left_shoulder_roll_joint" pos="0 0 0" axis="1 0 0" range="-1.57 3.14" actuatorfrcrange="-10 10"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_shoulder_roll_link"/>
            <geom name="left_shoulder_roll_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_shoulder_roll_link"/>
            <body name="left_shoulder_yaw_link" pos="0 0 -0.15">
              <inertial pos="7.42e-05 0.017031 -0.049381" mass="0.62913" diaginertia="0.00029805 0.00044347 0.00031727"/>
              <joint name="left_shoulder_yaw_joint" pos="0 0 0" axis="0 0 1" range="-3.14 3.14" actuatorfrcrange="-10 10"/>
              <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_shoulder_yaw_link"/>
              <geom name="left_shoulder_yaw_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_shoulder_yaw_link"/>
              <body name="left_elbow_link" pos="0 0 -0.05">
                <inertial pos="0.12358 1.39e-08 8.43e-08" mass="0.20775" diaginertia="0.00017062 0.00078906 0.00076895"/>
                <joint name="left_elbow_joint" pos="0 0 0" axis="0 1 0" range="-1.57 2.0" actuatorfrcrange="-10 10"/>
                <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="left_elbow_link"/>
                <geom name="left_elbow_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="left_elbow_link"/>
                <!-- Virtual end-effector -->
                <body name="left_ee_link" pos="0.10 0 0">
                  <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                  <geom type="sphere" size="0.01" rgba="0.2 0.5 0.8 1" contype="0" conaffinity="0" density="0" group="1"/>
                  <geom name="left_ee_link" type="sphere" size="0.01"/>
                </body>
              </body>
            </body>
          </body>
        </body>

        <!-- RIGHT ARM -->
        <body name="right_shoulder_pitch_link" pos="0 -0.115 0">
          <inertial pos="-0.017031 -0.04949 -0.0001267" mass="0.62913" diaginertia="0.00044347 0.00032046 0.00029486"/>
          <joint name="right_shoulder_pitch_joint" pos="0 0 0" axis="0 1 0" range="-3.14 3.14" actuatorfrcrange="-10 10"/>
          <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_shoulder_pitch_link"/>
          <geom name="right_shoulder_pitch_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_shoulder_pitch_link"/>
          <body name="right_shoulder_roll_link" pos="0 -0.05 0">
            <inertial pos="0.0013403 1.12e-05 -0.11572" mass="0.653" diaginertia="0.00039643 0.00038836 0.00046829"/>
            <joint name="right_shoulder_roll_joint" pos="0 0 0" axis="1 0 0" range="-3.14 1.57" actuatorfrcrange="-10 10"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_shoulder_roll_link"/>
            <geom name="right_shoulder_roll_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_shoulder_roll_link"/>
            <body name="right_shoulder_yaw_link" pos="0 0.00020894 -0.15">
              <inertial pos="0.00030713 -0.017239 -0.04937" mass="0.62913" diaginertia="0.00029805 0.00044347 0.00031726"/>
              <joint name="right_shoulder_yaw_joint" pos="0 0 0" axis="0 0 1" range="-3.14 3.14" actuatorfrcrange="-10 10"/>
              <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_shoulder_yaw_link"/>
              <geom name="right_shoulder_yaw_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_shoulder_yaw_link"/>
              <body name="right_elbow_link" pos="0 0 -0.05">
                <inertial pos="0.12358 -0.00020893 7.81e-08" mass="0.20775" diaginertia="0.00017062 0.00078906 0.00076895"/>
                <joint name="right_elbow_joint" pos="0 0 0" axis="0 1 0" range="-1.57 2.0" actuatorfrcrange="-10 10"/>
                <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.7 0.7 0.7 1" mesh="right_elbow_link"/>
                <geom name="right_elbow_link" type="mesh" rgba="0.7 0.7 0.7 1" mesh="right_elbow_link"/>
                <!-- Virtual end-effector -->
                <body name="right_ee_link" pos="0.10 0 0">
                  <inertial pos="0 0 0" mass="0.001" diaginertia="1e-7 1e-7 1e-7"/>
                  <geom type="sphere" size="0.01" rgba="0.2 0.5 0.8 1" contype="0" conaffinity="0" density="0" group="1"/>
                  <geom name="right_ee_link" type="sphere" size="0.01"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <statistic center="1.0 0.7 1.0" extent="0.8"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0.9 0.9 0.9"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-140" elevation="-20"/>
  </visual>
  <asset>
    <texture name="texplane" builtin="checker" height="512" width="512" rgb1=".2 .3 .4" rgb2=".1 .15 .2" type="2d"/>
    <material name="MatPlane" reflectance="0.5" shininess="0.01" specular="0.1" texrepeat="1 1" texture="texplane" texuniform="true"/>
  </asset>
</mujoco>
```

- [ ] **Step 2: Verify XML loads in MuJoCo**

```bash
source scripts/source_isaacsim_setup.sh && python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.xml')
print('nq:', model.nq, '(expected: 28 = 7 base + 21 joints)')
print('nv:', model.nv, '(expected: 27 = 6 base + 21 joints)')
print('nu:', model.nu, '(expected: 21)')
print('nbody:', model.nbody, '(expected: >22 due to virtual links)')
"
```

Expected output:
```
nq: 28 (expected: 28 = 7 base + 21 joints)
nv: 27 (expected: 27 = 6 base + 21 joints)
nu: 21 (expected: 21)
nbody: ... (expected: >22 due to virtual links)
```

- [ ] **Step 3: Check named geom lookup works for foot spheres**

```bash
source scripts/source_isaacsim_setup.sh && python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.xml')
for name in ['left_foot_sphere_1_link', 'right_foot_sphere_1_link', 'left_toe_link', 'right_toe_link', 'left_ee_link', 'right_ee_link']:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    print(f'{name}: geom_id={gid}')
"
```

Expected: All 6 lookups succeed with positive geom IDs.

- [ ] **Step 4: Verify joint limits from XML**

```bash
source scripts/source_isaacsim_setup.sh && python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.xml')
import numpy as np
# Joint qpos indices start after base (7:7+21)
jl = model.jnt_range[1:]  # skip freejoint
names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(1, 22)]
for n, r in zip(names, jl):
    print(f'{n}: [{r[0]:.2f}, {r[1]:.2f}]')
"
```

Expected: 21 joints printed with their limits.

- [ ] **Step 5: Commit**

```bash
git add src/holosoma_retargeting/holosoma_retargeting/models/irmv_v3/irmv_v3_21dof.xml
git commit -m "feat: add IRMV V3 MJCF model with virtual links"
```

---

### Task 6: End-to-end smoke test

**Files:** None (read-only verification)

- [ ] **Step 1: Verify full config chain from CLI**

```bash
source scripts/source_isaacsim_setup.sh && python -c "
from holosoma_retargeting.config_types.robot import RobotConfig
from holosoma_retargeting.config_types.data_type import MotionDataConfig
from holosoma_retargeting.config_types.data_conversion import DataConversionConfig

rc = RobotConfig(robot_type='irmv_v3')
mc = MotionDataConfig(data_format='lafan', robot_type='irmv_v3')
dc = DataConversionConfig(input_file='dummy.npz', robot='irmv_v3')

# Check all properties resolve
assert rc.ROBOT_DOF == 21
assert rc.ROBOT_HEIGHT == 1.32
assert rc.ROBOT_NAME == 'irmv_v3_21dof'
assert rc.ROBOT_URDF_FILE.endswith('irmv_v3_21dof.urdf')
assert len(rc.FOOT_STICKING_LINKS) == 8
assert 'Spine1' in mc.resolved_joints_mapping
assert mc.resolved_joints_mapping['Spine1'] == 'pelvis'
assert len(dc.JOINT_NAMES) == 21
print('All config checks passed!')
"
```

Expected: `All config checks passed!`

- [ ] **Step 2: Verify XML path derivation works**

The retargeter derives the XML path from the URDF path by replacing `.urdf` with `.xml`. Verify:

```bash
source scripts/source_isaacsim_setup.sh && python -c "
from holosoma_retargeting.config_types.robot import RobotConfig
rc = RobotConfig(robot_type='irmv_v3')
xml = rc.ROBOT_URDF_FILE.replace('.urdf', '.xml')
print('XML path:', xml)
import os
assert os.path.exists('src/holosoma_retargeting/holosoma_retargeting/' + xml), 'XML not found'
print('XML file exists!')
"
```

Expected:
```
XML path: models/irmv_v3/irmv_v3_21dof.xml
XML file exists!
```

- [ ] **Step 3: Commit** (if any changes from fixes)

```bash
git status
```

If clean, no commit needed. This task is verification-only.

---

### Task 7: Run ruff + mypy

**Files:** All modified Python files

- [ ] **Step 1: Run ruff check + format**

```bash
ruff check --fix src/holosoma_retargeting/holosoma_retargeting/config_types/robot.py src/holosoma_retargeting/holosoma_retargeting/config_types/data_type.py src/holosoma_retargeting/holosoma_retargeting/config_types/data_conversion.py && ruff format src/holosoma_retargeting/holosoma_retargeting/config_types/robot.py src/holosoma_retargeting/holosoma_retargeting/config_types/data_type.py src/holosoma_retargeting/holosoma_retargeting/config_types/data_conversion.py
```

Expected: No errors, files formatted.

- [ ] **Step 2: Run mypy type check**

```bash
mypy src/holosoma_retargeting/holosoma_retargeting/config_types/
```

Expected: No type errors.

- [ ] **Step 3: Verify pre-commit passes on changed files**

```bash
git diff --name-only HEAD~6..HEAD -- | xargs pre-commit run --files
```

Expected: All hooks pass.

- [ ] **Step 4: Commit any lint fixes**

```bash
git add -u && git commit -m "chore: lint fixes for IRMV V3 config changes"
```
(only if changes exist)
