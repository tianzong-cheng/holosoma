# Session Findings: Adding a New Robot to Holosoma Retargeting

**Date:** 2026-04-29
**Based on:** Adding IRMV V3 (21-DOF humanoid) to the retargeting pipeline

## Overview

The official guide (`ADD_ROBOT_TYPE_README.md`) covers the config-side edits but omits the largest piece of work: **creating the MuJoCo XML model**. The retargeter loads the MJCF XML, not the URDF — the URDF is only for viser visualization. This document fills those gaps with practical findings.

## Files to Create/Modify (Complete List)

| # | File | Action | Criticality |
|---|------|--------|-------------|
| 1 | `models/{robot}/meshes/*` | Copy meshes from robot description | Required |
| 2 | `models/{robot}/{robot}_{dof}dof.urdf` | Copy URDF, fix `package://` → `meshes/` paths | Required |
| 3 | `models/{robot}/{robot}_{dof}dof.xml` | **Create MuJoCo XML from scratch** | Required |
| 4 | `config_types/robot.py` | Add `_ROBOT_DEFAULTS` entry + foot sticking links | Required |
| 5 | `config_types/data_type.py` | Add `JOINTS_MAPPINGS` per format | Required |
| 6 | `config_types/data_conversion.py` | Add `_ROBOT_JOINT_NAMES_DEFAULT` entry | Required |

## 1. MJCF XML Construction (The Hard Part)

### Key Differences from URDF

| Concept | URDF | MuJoCo XML |
|---------|------|------------|
| Joint origin | Separate `<joint>` with `<origin xyz rpy>` | `pos` on child `<body>` |
| Joint axis | `<axis xyz>` on joint element | `axis` attribute on `<joint>` inside body |
| Inertial pose | `<origin xyz rpy>` with `rpy=0 0 0` | `pos` only (quaternion default identity) |
| Inertia matrix | Full 6-component `ixx ixy ixz iyy iyz izz` | Diagonal only: `diaginertia="ixx iyy izz"` |
| Floating base | `<link name="pelvis"/>` (implicit) | `<freejoint/>` inside root body |
| Mesh | `<mesh filename="package://..."/>` | `<mesh content_type="model/stl" file="mesh.STL"/>` |

### Conversion Steps

**Body positions:** Copy the `<joint><origin xyz>` value directly as `pos` on the child `<body>`. URDF joint origin = MJCF body position (relative to parent).

**Joint axes:** Copy `<axis xyz>` directly. Supports any axis vector (e.g., `axis="0 0.86603 -0.5"` for angled hip joints).

**Joint limits:** Copy `<limit lower upper>` to `range="lower upper"`. Omit `velocity`.

**Joint names:** **Preserve exactly**, including any typos in the source URDF (e.g., `lefT_hip_yaw_joint` with capital T). The name string must match everywhere.

**Inertial:** Copy `<mass value>`, use `<inertia ixx iyy izz>` (diagonal only — off-diagonal terms are negligible and MuJoCo doesn't support them in the XML body element).

**Actuators:** Add `actuatorfrcrange` for actuated joints only. **Omit for passive joints** (ankles with `effort=0` in URDF). Without `actuatorfrcrange`, the joint has no actuator in MuJoCo (nu = active DOF count, which will be less than total DOF).

**Mesh types:** SolidWorks exports STL files. Use `content_type="model/stl"` in `<mesh>` asset declarations. Set `meshdir="meshes/"` in `<compiler>`.

**Mesh face limits:** MuJoCo has a ~200k face limit per mesh. Simplify large meshes (e.g., with `fast_simplification`) if needed.

### Two `<geom>` Per Body (Pattern from G1)

Each real body gets two geom elements:
```xml
<geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="..." mesh="..."/>
<geom name="link_name" type="mesh" rgba="..." mesh="..."/>
```
- First: inert (no collision/contact), visual-only (group=1)
- Second: **named** — this is what the retargeter looks up by name for joint position tracking

### Root Body

```xml
<body name="pelvis">
  <inertial pos="..." mass="..." diaginertia="..."/>
  <freejoint/>
  <geom type="mesh" ... mesh="pelvis"/>
  <geom name="pelvis" type="mesh" ... mesh="pelvis"/>
  <!-- children... -->
</body>
```

The root body name is what `Spine1` (LAFAN) or `Pelvis` (SMPLH) maps to in joint mappings.

## 2. Virtual Links (Essential for Retargeting)

The retargeter works by tracking spatial positions of named geoms. Robots missing certain body parts need virtual links:

| Virtual Link | Parent | Purpose | Typical Position |
|-------------|--------|---------|-----------------|
| `{lr}_ankle_intermediate_1_link` | `{lr}_knee_link` | Ankle spatial anchor (sibling of ankle_pitch) | `pos="0 0 -0.28"` (~70mm above spheres) |
| `{lr}_toe_link` | `{lr}_ankle_roll_link` | Toe position tracking | `pos="0.15 0 -0.018"` (front of foot, same Z as spheres) |
| `{lr}_ee_link` | `{lr}_elbow_link` | Wrist/hand position tracking | `pos="0.22 0 0"` (forward from elbow) |
| `{lr}_foot_sphere_{1..4}_link` | `{lr}_ankle_roll_link` | Foot-ground contact detection | See foot sphere layout below |

### Foot Sphere Layout (4 per foot, following G1 pattern)

All spheres share the **same Z** from ankle_roll_link. The gap between spheres and the intermediate link should be ~67-70mm:

```
       sphere_3 (0.12, +0.03, z)    sphere_4 (0.12, -0.03, z)
       [front-left]                 [front-right]

       sphere_1 (-0.05, +0.03, z)   sphere_2 (-0.05, -0.03, z)
       [rear-left]                  [rear-right]
```

Typical values: z = -0.018 to -0.03 (depending on foot thickness).

### Why Virtual Links Are Needed

The retargeter builds an **interaction mesh** (Delaunay tetrahedralization) from the positions of tracked joints/links. Each joint mapping entry adds a vertex to this mesh. Virtual links provide spatial anchors that:
- Define the arm endpoint (EE link for wrist position)
- Define foot contact points (spheres for sticking constraint)
- Define toe position for step detection
- Define ankle position for leg kinematics

## 3. Joint Mappings: Choosing Which Link to Track

The mapping maps a **human skeleton joint** to a **MuJoCo geom name** (from the XML). The choice matters:

| Body Part | Human Joint | Map to (if available) | Rationale |
|-----------|------------|----------------------|-----------|
| Pelvis | Spine1 / Pelvis | Root body (`pelvis`) | Base of kinematic tree |
| Hip | LeftUpLeg / L_Hip | `*_hip_pitch_link` | First leg link after pelvis |
| Knee | LeftLeg / L_Knee | `*_knee_link` | Mid-leg |
| Ankle | LeftFoot / L_Foot | `*_ankle_intermediate_1_link` | Spatial anchor, NOT the pitch link directly |
| Toe | LeftToeBase / L_Toe | `*_toe_link` | Virtual toe link |
| Shoulder | LeftArm / L_Shoulder | `*_shoulder_roll_link` | Second shoulder link (roll, not pitch) |
| Elbow | LeftForeArm / L_Elbow | `*_elbow_link` | End of real arm chain |
| Wrist | LeftHand / L_Wrist | `*_ee_link` | Virtual end-effector after elbow |

### Why intermediate_1_link for the ankle?

G1 maps `LeftFoot` to `left_ankle_intermediate_1_link` — a fixed dummy body at `pos="0 0 -0.28"` from the knee. This sits **above** the ankle joint chain (pitch + roll), providing a stable spatial anchor that doesn't move with ankle articulation. Mapping to `ankle_pitch_link` directly would place the vertex at the ankle joint, which shifts during foot flexion.

## 4. Foot Sticking Links

These are the **named geoms** used by the foot-sticking constraint (ground contact detection). They must be `<geom name="...">` elements with a `name` attribute:

```python
# In config_types/robot.py, _foot_sticking_links()
if self.robot_type == "myrobot":
    return [
        "left_foot_sphere_1_link", "right_foot_sphere_1_link",
        "left_foot_sphere_2_link", "right_foot_sphere_2_link",
        "left_foot_sphere_3_link", "right_foot_sphere_3_link",
        "left_foot_sphere_4_link", "right_foot_sphere_4_link",
    ]
```

These names must match the `name` attribute of `<geom>` in the XML exactly.

## 5. Joint Names for Data Conversion

The `_ROBOT_JOINT_NAMES_DEFAULT` dict in `config_types/data_conversion.py` lists the **ordered joint names** used for post-retargeting format conversion. The order convention (from the robot's controller config):

```
left leg (6) → right leg (6) → waist (1) → left arm (4) → right arm (4)
```

This is not the MuJoCo qpos order (which follows the XML nesting order). The data conversion module remaps between the two. Verify the order against the robot's controller joint name list.

## 6. Verification Checklist

After setup, verify:

```bash
# 1. Config chain resolves
python -c "
from holosoma_retargeting.config_types.robot import RobotConfig
rc = RobotConfig(robot_type='myrobot')
assert rc.ROBOT_DOF == expected_dof
assert rc.ROBOT_HEIGHT == expected_height
print('Config OK')
"

# 2. Joint mapping resolves for target format
python -c "
from holosoma_retargeting.config_types.data_type import MotionDataConfig
mc = MotionDataConfig(data_format='lafan', robot_type='myrobot')
print(mc.resolved_joints_mapping)
"

# 3. XML loads in MuJoCo
python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('models/myrobot/myrobot_{dof}dof.xml')
print(f'nq={m.nq} nv={m.nv} nu={m.nu}')
# nq = 7 + DOF, nv = 6 + DOF, nu = actuated DOF count
"

# 4. Named geoms resolve
python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('models/myrobot/myrobot_{dof}dof.xml')
for name in foot_sticking_links + ['left_ee_link', 'right_ee_link', 'left_toe_link', 'right_toe_link']:
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    print(f'{name}: geom_id={gid}')
"

# 5. XML path derivation works (retargeter replaces .urdf → .xml)
python -c "
rc = RobotConfig(robot_type='myrobot')
xml = rc.ROBOT_URDF_FILE.replace('.urdf', '.xml')
import os; assert os.path.exists('src/.../' + xml)
"
```

## 7. Common Pitfalls

1. **Joint name typos in URDF must be preserved everywhere.** If the URDF has `lefT_hip_yaw_joint`, every reference in config files must use `lefT_hip_yaw_joint`.

2. **The retargeter loads XML, not URDF.** The URDF is only for viser. The XML must exist or the retargeter crashes. The path derivation is automatic: `robot_urdf_file.replace('.urdf', '.xml')`.

3. **MuJoCo mesh face limit (~200k).** SolidWorks STL exports can exceed this (e.g., torso_link at 217k faces). Pre-simplify large meshes.

4. **`nu` (actuator count) may be less than DOF.** Passive ankle joints don't need actuators. `nu = DOF - passive_joint_count`. This is expected.

5. **Hip pitch axes can be non-standard.** Some robots use angled hip pitch axes (e.g., `axis="0 0.86603 -0.5"`). Copy the exact vector from the URDF.

6. **The `<geom name="...">` must exist** for every entry in foot sticking links AND joint mappings. If a mapping references `left_ee_link`, that geom must be in the XML.

7. **Don't add `actuatorfrcrange` to virtual links.** They are fixed bodies (no joint), so they don't need actuators. The retargeter tracks their position through forward kinematics.

8. **The gap between intermediate link and foot spheres should be ~67-70mm** (matching G1). This ensures the foot contact plane is at a consistent distance from the ankle spatial anchor.
