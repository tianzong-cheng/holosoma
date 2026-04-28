# Design: IRMV V3 Retargeting Support

**Date:** 2026-04-28
**Robot:** `irmv_v3` (21 DOF humanoid)
**Data format:** LAFAN only

## Overview

Add retargeting support for the IRMV V3 humanoid robot to the holosoma retargeting pipeline. The robot is a 21-DOF lower-body humanoid from the robot description at `robot_description/irmv/irmv-humanoid-v3-description/`.

## Robot Characteristics

| Property | Value |
|----------|-------|
| DOF | 21 |
| Height | 1.32 m |
| Waist | 1 DOF (yaw, Z-axis rotation) |
| Each leg | 6 DOF (hip pitch/roll/yaw, knee, ankle pitch/roll) |
| Each arm | 4 DOF (shoulder pitch/roll/yaw, elbow) |
| No wrists/hands | Arms end at elbow |
| Passive ankles | Both ankle joints have `effort=0` |
| Name quirk | Left hip yaw is `lefT_hip_yaw_joint` (capital T) |
| Hip pitch axes | Angled (`(0, 0.866, -0.5)` left / `(0, 0.866, 0.5)` right) |

## Virtual Links in MJCF

Since the real robot has no wrist/hand joints and no toe links, virtual
rigid bodies are added to the MJCF XML to serve as retargeting targets:

| Virtual Link | Parent | Purpose |
|-------------|--------|---------|
| `left_ee_link` / `right_ee_link` | `*_elbow_link` | Wrist/hand position tracking |
| `left_toe_link` / `right_toe_link` | `*_ankle_roll_link` | Toe position tracking |
| `left_foot_sphere_{1..4}_link` | `*_ankle_roll_link` | Foot-ground contact for sticking constraint |

## Files to Create

### `models/irmv_v3/` (new directory)

| File | Source |
|------|--------|
| `irmv_v3_21dof.urdf` | Copied from robot description `urdf/irmv_v3.urdf` |
| `irmv_v3_21dof.xml` | **New** — MuJoCo XML with virtual links + foot spheres |
| `meshes/*.STL` | Copied from robot description `meshes/*.STL` |

### MJCF XML structure

Based on the URDF kinematic tree, with additions:
- Virtual body `left_ee_link` welded after `left_elbow_link`
- Virtual body `right_ee_link` welded after `right_elbow_link`
- Virtual body `left_toe_link` welded after `left_ankle_roll_link`
- Virtual body `right_toe_link` welded after `right_ankle_roll_link`
- 4 small sphere geoms per foot (e.g., `left_foot_sphere_1_link`) welded to `*_ankle_roll_link`, used for foot-sticking constraints

## Files to Modify

### 1. `config_types/robot.py`

**`_ROBOT_DEFAULTS` entry:**
```python
"irmv_v3": {"robot_dof": 21, "robot_height": 1.32, "object_name": "ground"},
```

**Foot sticking links (`_foot_sticking_links`):**
```python
if self.robot_type == "irmv_v3":
    return [
        "left_foot_sphere_1_link", "right_foot_sphere_1_link",
        "left_foot_sphere_2_link", "right_foot_sphere_2_link",
        "left_foot_sphere_3_link", "right_foot_sphere_3_link",
        "left_foot_sphere_4_link", "right_foot_sphere_4_link",
    ]
```

No manual joint limits, costs, or nominal tracking indices needed — XML joint limits are used directly.

### 2. `config_types/data_type.py`

Add one entry to `JOINTS_MAPPINGS` for `("lafan", "irmv_v3")`:

| Human Joint | Robot Link |
|-------------|------------|
| `Spine1` | `pelvis` |
| `LeftUpLeg` | `left_hip_pitch_link` |
| `LeftLeg` | `left_knee_link` |
| `LeftFoot` | `left_ankle_pitch_link` |
| `LeftToeBase` | `left_toe_link` |
| `RightUpLeg` | `right_hip_pitch_link` |
| `RightLeg` | `right_knee_link` |
| `RightFoot` | `right_ankle_pitch_link` |
| `RightToeBase` | `right_toe_link` |
| `LeftArm` | `left_shoulder_roll_link` |
| `LeftForeArm` | `left_elbow_link` |
| `LeftHand` | `left_ee_link` |
| `RightArm` | `right_shoulder_roll_link` |
| `RightForeArm` | `right_elbow_link` |
| `RightHand` | `right_ee_link` |

### 3. `config_types/data_conversion.py`

Add `"irmv_v3"` entry to `_ROBOT_JOINT_NAMES_DEFAULT` with the 21 ordered joint names:
```
left_hip_pitch_joint, left_hip_roll_joint, lefT_hip_yaw_joint, left_knee_joint,
left_ankle_pitch_joint, left_ankle_roll_joint,
right_hip_pitch_joint, right_hip_roll_joint, right_hip_yaw_joint, right_knee_joint,
right_ankle_pitch_joint, right_ankle_roll_joint,
waist_yaw_joint,
left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint, left_elbow_joint,
right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_elbow_joint
```

## No Changes Required

- `examples/robot_retarget.py` — auto-discovers robots from `_ROBOT_DEFAULTS`
- `examples/parallel_robot_retarget.py` — same
- `interaction_mesh_retargeter.py` — no robot-specific logic
- `utils.py` — no robot-specific logic

## Testing

After implementation, verify with:
```bash
source scripts/source_isaacsim_setup.sh
python src/holosoma_retargeting/holosoma_retargeting/examples/robot_retarget.py \
    --data_path <data_path> \
    --task-type robot_only \
    --task-name <sequence_name> \
    --data_format lafan \
    --robot irmv_v3 \
    --retargeter.debug \
    --retargeter.visualize
```
