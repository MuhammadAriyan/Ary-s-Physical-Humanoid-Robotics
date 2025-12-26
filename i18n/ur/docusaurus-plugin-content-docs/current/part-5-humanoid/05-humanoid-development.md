---
title: "ہیومینائڈ روبوٹ ڈیولپمنٹ"
sidebar_position: 5
---

# باب 5: ہیومینائڈ روبوٹ ڈیولپمنٹ

## سیکھنے کے مقاصد

اس باب کے اختتام پر، آپ یہ کر سکیں گے:
- ہیومینائڈ روبوٹ کی کائنیمیٹکس کو سمجھنا بشمول فارورڈ اور انورس کائنیمیٹکس
- دو پائی چلنے کا کنٹرول اور توازن کی حکمت عملی لاگو کرنا
- ہیومینائڈ ہاتھوں کے لیے گرفت کے الگورتھم کے ساتھ ہیرا پھیری کے نظام ڈیزائن کرنا
- قدرتی انسان-روبوٹ تعامل کے نمونے بنانا
- کائنیمیٹکس کے حسابات اور چلنے کے کنٹرولرز کے لیے کام کرنے والی کوڈ کی مثالیں تیار کرنا
- حقیقی دنیا کی ایپلی کیشنز کے لیے Unitree روبوٹ ہارڈویئر کی تفصیلات کا حوالہ دینا

## 5.1 ہیومینائڈ روبوٹ کائنیمیٹکس کا تعارف

Humanoid robot kinematics forms the mathematical foundation for all motion planning, control, and manipulation tasks. Unlike industrial robots with fixed bases, humanoid robots must contend with dynamic base conditions, self-collision avoidance, and balance constraints. This chapter builds upon the simulation and control foundations established in previous chapters to address the unique challenges of humanoid robot development.

انسان نما روبوٹ kinematics تمام motion planning، control اور manipulation tasks کے لیے ریاضیاتی بنیاد فراہم کرتی ہے۔ Fixed bases والے industrial robots کے برعکس، انسان نما روبوٹس کو dynamic base conditions، self-collision avoidance اور balance constraints کا سامنا کرنا پڑتا ہے۔ یہ chapter انسان نما روبوٹ development کے منفرد چیلنجز کو address کرنے کے لیے پچھلے chapters میں قائم simulation اور control foundations پر بناتا ہے۔

Humanoid robots typically possess 30 or more degrees of freedom distributed across the legs, torso, arms, and head. The Unitree H1 robot, for example, features 19 degrees of freedom in its humanoid configuration: 3 in each leg for hip, knee, and ankle, 2 in each arm for shoulder and elbow, and additional joints for waist and head orientation. This complexity requires systematic approaches to kinematics, dynamics, and control.

انسان نما روبوٹس میں عام طور پر 30 یا اس سے زیادہ degrees of freedom ہوتے ہیں جو legs، torso، arms اور head میں تقسیم ہوتے ہیں۔ مثال کے طور پر، Unitree H1 robot میں 19 degrees of freedom ہیں: ہر leg میں hip، knee اور ankle کے لیے 3، ہر arm میں shoulder اور elbow کے لیے 2، اور waist اور head orientation کے لیے اضافی joints ہیں۔ اس پیچیدگی کو kinematics، dynamics اور control کے systematic approaches درکار ہیں۔

### کائنیمیٹک چین کی ساخت

A humanoid robot's kinematic structure follows an anthropomorphic design mirroring human body organization. The base of the kinematic tree is typically the pelvis or torso, with limbs branching outward to the extremities. Understanding this hierarchical structure is essential for implementing forward and inverse kinematics algorithms.

ایک انسان نما روبوٹ کی kinematic structure anthropomorphic design کی پیروی کرتی ہے جو human body organization کی عکاسی کرتی ہے۔ Kinematic tree کی بنیاد عام طور پر pelvis یا torso ہوتی ہے، جس سے limbs extremities تک outward branch ہوتے ہیں۔ اس hierarchical structure کو سمجھنا forward اور inverse kinematics algorithms لاگو کرنے کے لیے ضروری ہے۔

```python
"""
Humanoid Robot Kinematic Chain Definition (انسان نما روبوٹ Kinematic Chain کی تعریف)

یہ module kinematic structure کو systematic joint naming convention اور tree structure کے ذریعے define کرتا ہے۔
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from enum import Enum
import numpy as np


class JointType(Enum):
    """Classification of joint types by motion capability."""
    REVOLUTE = "revolute"          # Single-axis rotation (most humanoid joints) - ایک محور پر گردش (زیادہ تر humanoid joints)
    CONTINUOUS = "continuous"      # Unlimited rotation (waist yaw, neck yaw) - لامحدود گردان (waist yaw, neck yaw)
    PRISMATIC = "prismatic"        # Linear motion (rare in humanoids) - لکیری حرکت (humanoids میں کم)
    SPHERICAL = "spherical"        # Three-axis rotation (theoretical) - تین محور پر گردان (نظریاتی)


@dataclass
class DHParameter:
    """
    Denavit-Hartenberg parameters for a single joint/link.

    DH convention robot geometry کو ہر joint کے چار parameters کے ذریعے describe کرنے کا standard way فراہم کرتی ہے۔
    """
    theta: float       # Joint angle (variable for revolute joints) - Joint angle (revolute joints کے لیے variable)
    d: float           # Link offset (variable for prismatic joints) - Link offset (prismatic joints کے لیے variable)
    a: float           # Link length - Link length
    alpha: float       # Link twist angle - Link twist angle

    # Transform from parent frame to this frame
    def get_transform(self) -> np.ndarray:
        """Compute homogeneous transformation matrix from DH parameters."""
        ct = np.cos(self.theta)
        st = np.sin(self.theta)
        ca = np.cos(self.alpha)
        sa = np.sin(self.alpha)

        T = np.array([
            [ct, -st * ca,  st * sa, self.a * ct],
            [st,  ct * ca, -ct * sa, self.a * st],
            [0,      sa,       ca,     self.d],
            [0,       0,        0,         1]
        ])
        return T


@dataclass
class LinkProperties:
    """Physical properties of a robot link."""
    mass: float                    # Mass in kilograms - کلوگرام میں mass
    com: np.ndarray               # Center of mass offset from joint frame - Joint frame سے center of mass offset
    inertia: np.ndarray           # 3x3 inertia tensor - 3x3 inertia tensor
    length: float = 0.0           # Characteristic length - Characteristic length
    width: float = 0.0            # Characteristic width - Characteristic width
    height: float = 0.0           # Characteristic height - Characteristic height


@dataclass
class JointLimits:
    """Position, velocity, and effort limits for a joint."""
    position_min: float           # Minimum joint angle (radians) - کم از کم joint angle (radians میں)
    position_max: float           # Maximum joint angle (radians) - زیادہ سے زیادہ joint angle (radians میں)
    velocity_max: float           # Maximum velocity (rad/s) - زیادہ سے زیادہ velocity (rad/s)
    effort_max: float             # Maximum torque (Nm) - زیادہ سے زیادہ torque (Nm)


@dataclass
class Joint:
    """
    Complete joint specification including kinematics and limits.

    kinematics اور limits سمیت مکمل joint specification۔
    """
    name: str
    joint_type: JointType
    axis: np.ndarray              # Rotation/translation axis - Rotation/translation axis
    parent_link: str              # Name of parent link - Parent link کا نام
    child_link: str               # Name of child link - Child link کا نام
    dh_params: DHParameter
    limits: JointLimits
    properties: LinkProperties
    stiffness: float = 0.0        # Joint stiffness for compliant control - Compliant control کے لیے joint stiffness
    damping: float = 0.0          # Joint damping for compliant control - Compliant control کے لیے joint damping


class HumanoidKinematicChain:
    """
    Manages the complete kinematic chain of a humanoid robot.

    یہ class forward kinematics computation، joint space operations اور kinematic validation کے methods فراہم کرتی ہے۔
    """

    def __init__(self, robot_name: str = "humanoid"):
        self.robot_name = robot_name
        self.joints: Dict[str, Joint] = {}
        self.links: Dict[str, LinkProperties] = {}
        self.root_frame: str = "pelvis_link"

        # Transformation from world to root frame
        self.world_to_root: np.ndarray = np.eye(4)

    def add_joint(self, joint: Joint) -> None:
        """Add a joint to the kinematic chain."""
        self.joints[joint.name] = joint
        self.links[joint.child_link] = joint.properties

    def add_standard_humanoid_joints(self) -> None:
        """
        Add standard humanoid joint configuration.

        یہ ایک characteristic humanoid بناتا ہے جس میں:
        - 3-DOF legs (hip, knee, ankle)
        - 3-DOF arms (shoulder, elbow, wrist)
        - 2-DOF torso (pitch, yaw)
        - 2-DOF neck (pitch, yaw)
        """
        # Left Leg Joints (hip roll/pitch, knee pitch, ankle pitch/roll)
        self.add_joint(create_hip_joint("left_hip_roll", -0.12))
        self.add_joint(create_hip_joint("left_hip_pitch", -0.12))
        self.add_joint(create_knee_joint("left_knee", -0.12))
        self.add_joint(create_ankle_joint("left_ankle", -0.12))

        # Right Leg Joints
        self.add_joint(create_hip_joint("right_hip_roll", 0.12))
        self.add_joint(create_hip_joint("right_hip_pitch", 0.12))
        self.add_joint(create_knee_joint("right_knee", 0.12))
        self.add_joint(create_ankle_joint("right_ankle", 0.12))

        # Torso Joints
        self.add_joint(create_torso_joint("waist_pitch"))
        self.add_joint(create_torso_joint("waist_yaw"))

        # Arm Joints
        self.add_joint(create_arm_joint("left_shoulder_pitch", -0.15, 0.35))
        self.add_joint(create_arm_joint("left_elbow", -0.15, 0.35))
        self.add_joint(create_arm_joint("right_shoulder_pitch", 0.15, 0.35))
        self.add_joint(create_arm_joint("right_elbow", 0.15, 0.35))

    def get_link_names(self) -> List[str]:
        """Get list of all link names in the kinematic chain."""
        return list(self.links.keys())

    def get_joint_names(self) -> List[str]:
        """Get list of all joint names."""
        return list(self.joints.keys())

    def get_dof(self) -> int:
        """Get total degrees of freedom."""
        return len(self.joints)


def create_hip_joint(name: str, y_offset: float) -> Joint:
    """Create a hip joint with standard parameters."""
    return Joint(
        name=name,
        joint_type=JointType.REVOLUTE,
        axis=np.array([1.0, 0.0, 0.0]) if "roll" in name else np.array([0.0, 1.0, 0.0]),
        parent_link="pelvis_link",
        child_link=name.replace("hip", "hip_") + "_link",
        dh_params=DHParameter(theta=0.0, d=y_offset, a=0.0, alpha=-np.pi/2),
        limits=JointLimits(
            position_min=-1.0 if "pitch" in name else -0.5,
            position_max=1.0 if "pitch" in name else 0.5,
            velocity_max=5.0,
            effort_max=150.0
        ),
        properties=LinkProperties(
            mass=2.5,
            com=np.array([0.0, 0.0, -0.05]),
            inertia=np.diag([0.01, 0.02, 0.01])
        )
    )


def create_knee_joint(name: str, y_offset: float) -> Joint:
    """Create a knee joint with standard parameters."""
    return Joint(
        name=name,
        joint_type=JointType.REVOLUTE,
        axis=np.array([1.0, 0.0, 0.0]),  # Pitch only
        parent_link=name.replace("knee", "hip_") + "_link",
        child_link="lower_leg_link",
        dh_params=DHParameter(theta=0.0, d=0.0, a=-0.45, alpha=np.pi/2),
        limits=JointLimits(
            position_min=-2.0,
            position_max=0.0,
            velocity_max=8.0,
            effort_max=100.0
        ),
        properties=LinkProperties(
            mass=3.0,
            com=np.array([0.0, 0.0, -0.25]),
            inertia=np.diag([0.03, 0.01, 0.03])
        )
    )


def create_ankle_joint(name: str, y_offset: float) -> Joint:
    """Create an ankle joint with standard parameters."""
    return Joint(
        name=name,
        joint_type=JointType.REVOLUTE,
        axis=np.array([1.0, 0.0, 0.0]) if "pitch" in name else np.array([0.0, 1.0, 0.0]),
        parent_link="lower_leg_link",
        child_link="foot_link",
        dh_params=DHParameter(theta=0.0, d=0.0, a=-0.10, alpha=-np.pi/2),
        limits=JointLimits(
            position_min=-0.5,
            position_max=0.5,
            velocity_max=6.0,
            effort_max=80.0
        ),
        properties=LinkProperties(
            mass=1.5,
            com=np.array([0.05, 0.0, -0.02]),
            inertia=np.diag([0.01, 0.02, 0.01])
        )
    )


def create_torso_joint(name: str) -> Joint:
    """Create a torso/waist joint."""
    return Joint(
        name=name,
        joint_type=JointType.REVOLUTE,
        axis=np.array([1.0, 0.0, 0.0]) if "pitch" in name else np.array([0.0, 0.0, 1.0]),
        parent_link="pelvis_link",
        child_link="torso_link",
        dh_params=DHParameter(theta=0.0, d=0.3, a=0.0, alpha=0.0),
        limits=JointLimits(
            position_min=-1.0,
            position_max=1.0,
            velocity_max=3.0,
            effort_max=100.0
        ),
        properties=LinkProperties(
            mass=10.0,
            com=np.array([0.0, 0.0, 0.15]),
            inertia=np.diag([0.1, 0.15, 0.1])
        )
    )


def create_arm_joint(name: str, y_offset: float, z_offset: float) -> Joint:
    """Create an arm joint with standard parameters."""
    return Joint(
        name=name,
        joint_type=JointType.REVOLUTE,
        axis=np.array([1.0, 0.0, 0.0]) if "pitch" in name or "elbow" in name else np.array([0.0, 0.0, 1.0]),
        parent_link="torso_link",
        child_link=name + "_link",
        dh_params=DHParameter(theta=0.0, d=z_offset, a=0.0 if "elbow" in name else -0.3,
                              alpha=-np.pi/2 if "elbow" in name else 0.0),
        limits=JointLimits(
            position_min=-3.0,
            position_max=3.0,
            velocity_max=6.0,
            effort_max=50.0
        ),
        properties=LinkProperties(
            mass=2.0,
            com=np.array([0.0, -0.15, 0.0]) if "shoulder" in name else np.array([0.0, -0.2, 0.0]),
            inertia=np.diag([0.02, 0.01, 0.02])
        )
    )


if __name__ == "__main__":
    # Example usage
    chain = HumanoidKinematicChain("unitree_h1")
    chain.add_standard_humanoid_joints()

    print(f"Robot: {chain.robot_name}")
    print(f"Total DOF: {chain.get_dof()}")
    print(f"Links: {chain.get_link_names()}")
    print(f"Joints: {chain.get_joint_names()}")

# یہ module humanoid robot کی kinematic chain کو define اور manage کرتا ہے۔
# اس میں joints، links اور ان کی properties شامل ہیں۔
```

### Coordinate Frame Conventions (Coordinate Frame Conventions)

Humanoid robots typically use multiple coordinate frame conventions simultaneously. The world frame provides a global reference for navigation and positioning. The pelvis frame serves as the robot's local reference, moving with the robot during locomotion. End-effector frames are defined for hands and feet to facilitate manipulation and footstep planning. Consistent coordinate conventions are critical for algorithms that span multiple frames, such as visual servoing or whole-body coordination.

انسان نما روبوٹس عام طور پر ایک ساتھ متعدد coordinate frame conventions استعمال کرتے ہیں۔ World frame navigation اور positioning کے لیے global reference فراہم کرتی ہے۔ Pelvis frame robot کا local reference ہے، جو locomotion کے دوران robot کے ساتھ move کرتی ہے۔ End-effector frames hands اور feet کے لیے define کی جاتی ہیں تاکہ manipulation اور footstep planning میں سہولت ہو۔ Consistent coordinate conventions ان algorithms کے لیے critical ہیں جو متعدد frames میں پھیلی ہوں، جیسے visual servoing یا whole-body coordination۔

The NED (North-East-Down) convention is common for ground robots, with Z pointing upward (opposite gravity). However, many ROS-based systems use ENU (East-North-Up) where Z points upward. The Unitree robots follow the standard robotics convention with Z upward, simplifying integration with ROS tools and libraries.

NED (North-East-Down) convention ground robots میں common ہے، جہاں Z اوپر کی طرف points کرتی ہے (gravity کے opposite)۔ تاہم، بہت سے ROS-based systems ENU (East-North-Up) استعمال کرتے ہیں جہاں Z اوپر کی طرف points کرتی ہے۔ Unitree robots standard robotics convention follow کرتے ہیں جس میں Z اوپر ہے، جو ROS tools اور libraries کے ساتھ integration کو آسان بناتا ہے۔

## 5.2 Forward and Inverse Kinematics (Forward اور Inverse Kinematics)

Forward kinematics computes the position and orientation of each link given the joint angles. This is computationally straightforward, involving sequential matrix multiplications from the base frame to the target link. Inverse kinematics reverses this process: given a desired end-effector pose, compute the required joint angles. Inverse kinematics is computationally challenging due to potential multiple solutions, singularities, and reachability constraints.

Forward kinematics joint angles دیئے گئے ہر link کی position اور orientation compute کرتی ہے۔ یہ computationally straightforward ہے، جس میں base frame سے target link تک sequential matrix multiplications شامل ہیں۔ Inverse kinematics یہ process reverse کرتی ہے: desired end-effector pose دیئے گئے، required joint angles compute کریں۔ Inverse kinematics computationally challenging ہے کیونکہ potential multiple solutions، singularities اور reachability constraints ہیں۔

### Forward Kinematics Implementation (Forward Kinematics Implementation)

The forward kinematics for a humanoid robot follows the chain of transformations from the pelvis through each joint to the target link. For a leg with hip, knee, and ankle, the foot position depends on all three joint angles. The transformation at each joint is computed using the joint angle and DH parameters.

انسان نما روبوٹ کے لیے forward kinematics pelvis سے ہر joint کے ذریعے target link تک transformations کی chain follow کرتی ہے۔ Hip، knee اور ankle والے leg کے لیے، foot position تینوں joint angles پر depend کرتی ہے۔ ہر joint پر transformation joint angle اور DH parameters استعمال کرکے compute کی جاتی ہے۔

```python
"""
Forward Kinematics for Humanoid Robot (انسان نما روبوٹ کے لیے Forward Kinematics)

یہ module homogeneous transformations اور recursive Newton-Euler algorithm استعمال کرتے ہوئے forward kinematics implement کرتا ہے۔
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Pose:
    """SE(3) pose representation."""
    position: np.ndarray  # 3D position [x, y, z]
    orientation: np.ndarray  # Quaternion [w, x, y, z]

    @classmethod
    def identity(cls) -> "Pose":
        """Create identity pose at origin."""
        return cls(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0])
        )

    def to_transform(self) -> np.ndarray:
        """Convert pose to 4x4 homogeneous transformation matrix."""
        q = self.orientation
        # Normalize quaternion
        q = q / np.linalg.norm(q)

        w, x, y, z = q

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
        ])

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = self.position

        return T

    @classmethod
    def from_transform(cls, T: np.ndarray) -> "Pose":
        """Create pose from 4x4 homogeneous transformation matrix."""
        R = T[:3, :3]
        pos = T[:3, 3]

        # Quaternion from rotation matrix
        q = np.zeros(4)
        q[0] = 0.5 * np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])

        if q[0] > 1e-6:
            q[1] = (R[2, 1] - R[1, 2]) / (4 * q[0])
            q[2] = (R[0, 2] - R[2, 0]) / (4 * q[0])
            q[3] = (R[1, 0] - R[0, 1]) / (4 * q[0])
        else:
            # Near-singular case
            q[1] = np.sqrt(0.5 * (1 + R[0, 0] - R[1, 1] - R[2, 2]))
            q[2] = np.sqrt(0.5 * (1 - R[0, 0] + R[1, 1] - R[2, 2]))
            q[3] = np.sqrt(0.5 * (1 - R[0, 0] - R[1, 1] + R[2, 2]))

        return cls(position=pos, orientation=q)


class ForwardKinematics:
    """
    Forward kinematics solver for humanoid robots.

    humanoid robots کے لیے forward kinematics solver۔
    joint angles دیئے گئے homogeneous transformations استعمال کرتے ہوئے end-effector positions اور orientations compute کرتی ہے۔
    """

    def __init__(self):
        # Robot link lengths (in meters)
        self.hip_to_knee = 0.45      # Upper leg length (femur) - Upper leg length (femur)
        self.knee_to_ankle = 0.45    # Lower leg length (tibia) - Lower leg length (tibia)
        self.ankle_to_ground = 0.10  # Foot sole thickness - Foot sole thickness

        # Hip offset from pelvis center
        self.hip_lateral_offset = 0.12

    def compute_foot_pose(
        self,
        hip_roll: float,
        hip_pitch: float,
        knee_pitch: float,
        ankle_pitch: float,
        ankle_roll: float = 0.0,
        side: str = "left"
    ) -> Pose:
        """
        Compute foot pose from leg joint angles.

        leg joint angles سے foot pose compute کریں۔
        """
        # Sign based on left/right
        sign = -1.0 if side == "left" else 1.0
        y_offset = sign * self.hip_lateral_offset

        # Direction cosine matrix for each joint
        # Hip roll (rotation about X)
        R_hip_roll = np.array([
            [1, 0, 0],
            [0, np.cos(hip_roll), -np.sin(hip_roll)],
            [0, np.sin(hip_roll),  np.cos(hip_roll)]
        ])

        # Hip pitch (rotation about Y)
        R_hip_pitch = np.array([
            [np.cos(hip_pitch), 0, np.sin(hip_pitch)],
            [0, 1, 0],
            [-np.sin(hip_pitch), 0, np.cos(hip_pitch)]
        ])

        # Knee pitch (rotation about Y)
        R_knee = np.array([
            [np.cos(knee_pitch), 0, np.sin(knee_pitch)],
            [0, 1, 0],
            [-np.sin(knee_pitch), 0, np.cos(knee_pitch)]
        ])

        # Ankle pitch (rotation about Y)
        R_ankle_pitch = np.array([
            [np.cos(ankle_pitch), 0, np.sin(ankle_pitch)],
            [0, 1, 0],
            [-np.sin(ankle_pitch), 0, np.cos(ankle_pitch)]
        ])

        # Ankle roll (rotation about X)
        R_ankle_roll = np.array([
            [1, 0, 0],
            [0, np.cos(ankle_roll), -np.sin(ankle_roll)],
            [0, np.sin(ankle_roll),  np.cos(ankle_roll)]
        ])

        # Combined rotation from hip to ankle
        R_total = R_hip_roll @ R_hip_pitch @ R_knee @ R_ankle_pitch @ R_ankle_roll

        # Position computation using geometric approach
        # Hip joint position (in pelvis frame)
        hip_pos = np.array([0, y_offset, 0])

        # Knee position (hip + upper leg rotation)
        upper_leg_dir = R_hip_roll @ R_hip_pitch @ np.array([0, 0, -1])
        knee_pos = hip_pos + self.hip_to_knee * upper_leg_dir

        # Ankle position (knee + lower leg rotation)
        lower_leg_dir = R_hip_roll @ R_hip_pitch @ R_knee @ np.array([0, 0, -1])
        ankle_pos = knee_pos + self.knee_to_ankle * lower_leg_dir

        # Foot position (ankle + foot offset)
        foot_pos = ankle_pos + R_hip_roll @ R_hip_pitch @ R_knee @ R_ankle_pitch @ np.array([0, 0, -self.ankle_to_ground])

        # Convert rotation matrix to quaternion
        orientation = self._rotation_to_quaternion(R_total)

        return Pose(position=foot_pos, orientation=orientation)

    def compute_jacobian(
        self,
        joint_angles: Dict[str, float],
        side: str = "left"
    ) -> np.ndarray:
        """
        Compute geometric Jacobian for leg kinematics.

        leg kinematics کے لیے geometric Jacobian compute کریں۔

        The Jacobian relates joint velocities to end-effector velocity:
        v = J * q_dot
        """
        # Extract joint angles
        hip_roll = joint_angles.get(f"{side}_hip_roll", 0.0)
        hip_pitch = joint_angles.get(f"{side}_hip_pitch", 0.0)
        knee_pitch = joint_angles.get(f"{side}_knee", 0.0)
        ankle_pitch = joint_angles.get(f"{side}_ankle_pitch", 0.0)
        ankle_roll = joint_angles.get(f"{side}_ankle_roll", 0.0)

        # Compute foot position for sensitivity analysis
        eps = 1e-6
        base_pose = self.compute_foot_pose(hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll, side)

        n_joints = 5
        J = np.zeros((6, n_joints))

        # Numerical differentiation for Jacobian columns
        joint_names = ["hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"]

        for i, jname in enumerate(joint_names):
            perturbed = joint_angles.copy()
            perturbed[f"{side}_{jname}"] += eps

            perturbed_pose = self.compute_foot_pose(
                perturbed.get(f"{side}_hip_roll", 0.0),
                perturbed.get(f"{side}_hip_pitch", 0.0),
                perturbed.get(f"{side}_knee", 0.0),
                perturbed.get(f"{side}_ankle_pitch", 0.0),
                perturbed.get(f"{side}_ankle_roll", 0.0),
                side
            )

            # Linear velocity (delta position / delta angle)
            J[:3, i] = (perturbed_pose.position - base_pose.position) / eps

            # Angular velocity (using quaternion derivative approximation)
            q_base = base_pose.orientation
            q_pert = perturbed_pose.orientation
            delta_q = q_pert - q_base

            # Convert quaternion difference to angular velocity
            omega = self._quaternion_diff_to_angular_velocity(q_base, delta_q)
            J[3:, i] = omega / eps

        return J

    def _rotation_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion [w, x, y, z]."""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            i = np.argmax(np.diag(R))
            j = (i + 1) % 3
            k = (i + 2) % 3

            s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
            q = np.zeros(4)
            q[i + 1] = 0.5 * s
            q[0] = (R[k, j] - R[j, k]) / s
            q[j + 1] = (R[j, i] + R[i, j]) / s
            q[k + 1] = (R[k, i] + R[i, k]) / s
            q[0] = (R[j, k] - R[k, j]) / s

            w, x, y, z = q[0], q[1], q[2], q[3]

        return np.array([w, x, y, z])

    def _quaternion_diff_to_angular_velocity(
        self,
        q1: np.ndarray,
        dq: np.ndarray
    ) -> np.ndarray:
        """Convert quaternion difference to angular velocity."""
        # Approximate angular velocity from quaternion error
        # omega = 2 * q1_conjugate * dq_dt (for small dq)
        q1_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])

        # Quaternion multiplication
        w = q1_conj[0] * dq[0] - q1_conj[1] * dq[1] - q1_conj[2] * dq[2] - q1_conj[3] * dq[3]
        x = q1_conj[0] * dq[1] + q1_conj[1] * dq[0] + q1_conj[2] * dq[3] - q1_conj[3] * dq[2]
        y = q1_conj[0] * dq[2] - q1_conj[1] * dq[3] + q1_conj[2] * dq[0] + q1_conj[3] * dq[1]
        z = q1_conj[0] * dq[3] + q1_conj[1] * dq[2] - q1_conj[2] * dq[1] + q1_conj[3] * dq[0]

        return 2.0 * np.array([x, y, z])


def compute_reachability(
    fk: ForwardKinematics,
    hip_pitch_range: Tuple[float, float] = (-1.0, 1.0),
    knee_range: Tuple[float, float] = (-2.0, 0.0),
    n_samples: int = 1000
) -> Tuple[np.ndarray, float]:
    """
    Compute reachable workspace for a leg.

    leg کے لیے reachable workspace compute کریں۔
    """
    reachable_points = []
    max_reach = 0.0

    for _ in range(n_samples):
        hip_pitch = np.random.uniform(*hip_pitch_range)
        knee = np.random.uniform(*knee_range)

        # Simple walking gait with ankle compensation
        ankle_pitch = -(hip_pitch + knee) / 2

        pose = fk.compute_foot_pose(
            hip_roll=0.0,
            hip_pitch=hip_pitch,
            knee_pitch=knee,
            ankle_pitch=ankle_pitch
        )

        reachable_points.append(pose.position)
        dist = np.linalg.norm(pose.position)
        if dist > max_reach:
            max_reach = dist

    return np.array(reachable_points), max_reach


if __name__ == "__main__":
    # Example usage
    fk = ForwardKinematics()

    # Test forward kinematics
    print("Forward Kinematics Test")
    print("=" * 50)

    # Neutral stance
    pose = fk.compute_foot_pose(
        hip_roll=0.0,
        hip_pitch=0.0,
        knee_pitch=0.0,
        ankle_pitch=0.0,
        side="left"
    )
    print(f"Neutral left foot position: {pose.position}")

    # Walking stance (knee bent)
    pose = fk.compute_foot_pose(
        hip_roll=0.0,
        hip_pitch=0.3,
        knee_pitch=-0.8,
        ankle_pitch=0.25,
        side="left"
    )
    print(f"Walking left foot position: {pose.position}")

    # Compute Jacobian at neutral
    joint_angles = {
        "left_hip_roll": 0.0,
        "left_hip_pitch": 0.0,
        "left_knee": 0.0,
        "left_ankle_pitch": 0.0,
        "left_ankle_roll": 0.0
    }
    J = fk.compute_jacobian(joint_angles, side="left")
    print(f"\nJacobian at neutral pose:")
    print(J.round(3))

    # Reachability analysis
    points, max_reach = compute_reachability(fk, n_samples=500)
    print(f"\nMax leg reach: {max_reach:.3f} m")

# یہ module humanoid robot کے leg kinematics کے لیے forward kinematics compute کرتا ہے۔
# اس میں foot pose calculation اور Jacobian computation شامل ہے۔
```

### Inverse Kinematics Approaches (Inverse Kinematics Approaches)

Inverse kinematics for humanoid legs can be solved using geometric, analytical, or numerical methods. Geometric methods exploit the planar nature of leg kinematics, solving for joint angles using trigonometry. Analytical methods derive closed-form solutions that are computationally efficient. Numerical methods like Jacobian pseudoinverse or optimization-based approaches handle more complex chains but require iteration.

انسان نما legs کے لیے inverse kinematics geometric، analytical، یا numerical methods استعمال کرکے solve کی جا سکتی ہے۔ Geometric methods leg kinematics کی planar nature exploit کرتی ہیں، trigonology استعمال کرکے joint angles solve کرتی ہیں۔ Analytical methods closed-form solutions derive کرتی ہیں جو computationally efficient ہیں۔ Numerical methods جیسے Jacobian pseudoinverse یا optimization-based approaches more complex chains handle کرتی ہیں لیکن iteration درکار ہوتی ہے۔

For a 5-DOF leg, analytical solutions typically exist by decomposing the problem into planar and lateral components. The hip roll and ankle roll control frontal plane motion, while hip pitch, knee pitch, and ankle pitch work together in the sagittal plane. This decoupling simplifies the solution and improves computational efficiency for real-time control.

5-DOF leg کے لیے، analytical solutions عام طور پر planar اور lateral components میں decompose کرکے exist کرتی ہیں۔ Hip roll اور ankle roll frontal plane motion control کرتے ہیں، جبکہ hip pitch، knee pitch اور ankle pitch sagittal plane میں مل کر کام کرتے ہیں۔ یہ decoupling solution کو simplify کرتی ہے اور real-time control کے لیے computational efficiency improve کرتی ہے۔

```python
"""
Inverse Kinematics for Humanoid Robot Legs (انسان نما روبوٹ legs کے لیے Inverse Kinematics)

یہ module humanoid leg control کے لیے analytical اور numerical inverse kinematics implement کرتا ہے۔
"""

import numpy as np
from typing import Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class IKResult:
    """Result of inverse kinematics computation."""
    success: bool
    joint_angles: Dict[str, float]
    error: float
    iterations: int
    computation_time: float


class InverseKinematics:
    """
    Analytical and numerical inverse kinematics solver.

    geometric decomposition efficient leg IK کے ساتھ use کرتی ہے،
    complex cases کے لیے numerical optimization تک fallback کرتی ہے۔
    """

    def __init__(self):
        # Link lengths
        self.L1 = 0.45  # Hip to knee (upper leg) - Hip to knee (upper leg)
        self.L2 = 0.45  # Knee to ankle (lower leg) - Knee to ankle (lower leg)
        self.L_foot = 0.10  # Ankle to ground - Ankle to ground

    def solve_leg_ik(
        self,
        target_foot_pos: np.ndarray,
        hip_offset: float,
        side: str = "left"
    ) -> IKResult:
        """
        Solve inverse kinematics for a leg to reach target foot position.

        leg کے لیے inverse kinematics solve کریں تاکہ target foot position تک پہنچ سکے۔

        Uses analytical solution with geometric decomposition.
        geometric decomposition کے ساتھ analytical solution use کرتا ہے۔
        """
        import time
        start_time = time.time()

        sign = -1.0 if side == "left" else 1.0

        # Extract components
        x = target_foot_pos[0]  # Forward
        y = target_foot_pos[1] - sign * hip_offset  # Lateral (relative to hip)
        z = target_foot_pos[2]  # Vertical

        # Hip roll - controls lateral foot position
        # Solve: y = -L1*sin(hip_roll) - L2*sin(hip_roll + knee + ankle)
        # Simplified: assume knee and ankle don't contribute much to lateral
        hip_roll = np.arcsin(np.clip(-y / (self.L1 + self.L2), -1.0, 1.0))

        # Frontal plane projection
        # xz plane distance from hip to foot
        r_frustal = np.sqrt(x**2 + z**2)

        # Check reachability
        min_reach = abs(self.L1 - self.L2) + self.L_foot
        max_reach = self.L1 + self.L2 + self.L_foot
        if r_frustal > max_reach or r_frustal < min_reach:
            return IKResult(
                success=False,
                joint_angles={},
                error=r_frustal - max_reach,
                iterations=0,
                computation_time=time.time() - start_time
            )

        # Sagittal plane IK (2-link arm problem)
        # Account for foot offset
        r = np.sqrt(x**2 + (z + self.L_foot)**2)

        # Law of cosines for knee angle
        # cos(knee_angle) = (L1^2 + L2^2 - r^2) / (2*L1*L2)
        cos_knee = (self.L1**2 + self.L2**2 - r**2) / (2 * self.L1 * self.L2)
        knee_pitch = np.arccos(np.clip(cos_knee, -1.0, 1.0))

        # Hip pitch angle
        # Angle from horizontal to L1
        alpha = np.arctan2(-(z + self.L_foot), x)
        # Angle between L1 and r
        beta = np.arccos(np.clip((self.L1**2 + r**2 - self.L2**2) / (2 * self.L1 * r), -1.0, 1.0))
        hip_pitch = alpha + beta

        # Ankle pitch - keep foot flat
        # Sum of joint angles should equal angle from ankle to horizontal
        # ankle_pitch = -(hip_pitch + knee_pitch + alpha)
        ankle_pitch = -np.arctan2(x, z + self.L_foot)

        # Foot roll (lateral balance)
        ankle_roll = -hip_roll * 0.5  # Partial compensation

        joint_angles = {
            f"{side}_hip_roll": hip_roll,
            f"{side}_hip_pitch": hip_pitch,
            f"{side}_knee": -knee_pitch,  # Negative for human-like knee bend
            f"{side}_ankle_pitch": ankle_pitch,
            f"{side}_ankle_roll": ankle_roll
        }

        # Clip to limits
        for name in joint_angles:
            if "knee" in name:
                joint_angles[name] = np.clip(joint_angles[name], -2.0, 0.0)
            elif "hip_roll" in name or "ankle_roll" in name:
                joint_angles[name] = np.clip(joint_angles[name], -0.5, 0.5)
            else:
                joint_angles[name] = np.clip(joint_angles[name], -1.5, 1.5)

        return IKResult(
            success=True,
            joint_angles=joint_angles,
            error=0.0,
            iterations=1,
            computation_time=time.time() - start_time
        )

    def solve_walking_ik(
        self,
        left_foot_target: np.ndarray,
        right_foot_target: np.ndarray,
        pelvis_height: float = 0.95,
        hip_offset: float = 0.12
    ) -> Dict[str, float]:
        """
        Solve IK for both legs in walking configuration.

        walking configuration میں both legs کے لیے IK solve کریں۔
        """
        left_result = self.solve_leg_ik(
            left_foot_target - np.array([0, 0, pelvis_height]),
            hip_offset,
            "left"
        )

        right_result = self.solve_leg_ik(
            right_foot_target - np.array([0, 0, pelvis_height]),
            hip_offset,
            "right"
        )

        if not left_result.success or not right_result.success:
            raise ValueError("IK solution not feasible")

        # Combine solutions
        all_joints = {}
        all_joints.update(left_result.joint_angles)
        all_joints.update(right_result.joint_angles)

        # Add torso orientation (keep level)
        all_joints["waist_pitch"] = 0.0
        all_joints["waist_yaw"] = 0.0

        return all_joints


class WalkingTrajectoryGenerator:
    """
    Generates walking trajectories for humanoid robots.

    humanoid robots کے لیے walking trajectories generate کرتا ہے۔

    Creates footstep plans and corresponding joint trajectories
    for stable bipedal locomotion۔
    """

    def __init__(self, step_length: float = 0.3, step_height: float = 0.08):
        self.step_length = step_length
        self.step_height = step_height
        self.step_width = 0.15  # Distance between feet

    def generate_step_sequence(
        self,
        n_steps: int,
        walk_direction: float = 0.0,  # radians
        initial_side: str = "right"
    ) -> list:
        """
        Generate a sequence of footsteps.

        footsteps کی ایک sequence generate کریں۔
        """
        footsteps = []
        side = initial_side

        for i in range(n_steps):
            # Alternating foot pattern
            x = self.step_length * i
            y = self.step_width if side == "right" else -self.step_width

            # Apply rotation for direction
            cos_d = np.cos(walk_direction)
            sin_d = np.sin(walk_direction)
            x_rot = x * cos_d - y * sin_d
            y_rot = x * sin_d + y * cos_d

            # Swing timing (normalized)
            t_swing_start = 0.5 * i
            t_swing_end = 0.5 * i + 0.4  # 40% of step cycle

            footsteps.append({
                "position": np.array([x_rot, y_rot, 0.0]),
                "side": side,
                "swing_start": t_swing_start,
                "swing_end": t_swing_end,
                "lift_height": self.step_height if i > 0 else 0.0
            })

            side = "right" if side == "left" else "left"

        return footsteps

    def generate_swing_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        t: float,
        lift_height: float = 0.08
    ) -> np.ndarray:
        """
        Generate foot trajectory during swing phase.

        swing phase کے دوران foot trajectory generate کریں۔

        Uses parabolic trajectory for smooth foot movement.
        smooth foot movement کے لیے parabolic trajectory use کرتا ہے۔
        """
        # Linear interpolation in XY
        x = start_pos[0] + t * (end_pos[0] - start_pos[0])
        y = start_pos[1] + t * (end_pos[1] - start_pos[1])

        # Parabolic height profile
        # z = 4 * h * t * (1-t) gives peak at t=0.5
        z = start_pos[2] + 4 * lift_height * t * (1 - t)

        return np.array([x, y, z])


if __name__ == "__main__":
    # Test inverse kinematics
    ik = InverseKinematics()

    print("Inverse Kinematics Test")
    print("=" * 50)

    # Test standing position
    result = ik.solve_leg_ik(
        target_foot_pos=np.array([0.0, -0.12, -0.95]),
        hip_offset=0.12,
        side="left"
    )
    print(f"Standing left leg: {result.success}")
    print(f"Iterations: {result.iterations}")
    print(f"Computation time: {result.computation_time * 1000:.2f} ms")
    for name, angle in result.joint_angles.items():
        print(f"  {name}: {np.degrees(angle):.1f} degrees")

    # Test stepping position
    result = ik.solve_leg_ik(
        target_foot_pos=np.array([0.3, -0.12, -0.85]),
        hip_offset=0.12,
        side="left"
    )
    print(f"\nStepping left leg: {result.success}")
    for name, angle in result.joint_angles.items():
        print(f"  {name}: {np.degrees(angle):.1f} degrees")

    # Test walking trajectory generation
    generator = WalkingTrajectoryGenerator()
    footsteps = generator.generate_step_sequence(n_steps=4)
    print(f"\nGenerated {len(footsteps)} footsteps")

    # Test swing trajectory
    start = np.array([0.0, -0.12, 0.0])
    end = np.array([0.3, -0.12, 0.0])
    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        pos = generator.generate_swing_trajectory(start, end, t)
        print(f"Swing t={t:.2f}: z={pos[2]:.3f} m")

# یہ module humanoid robot legs کے لیے inverse kinematics implement کرتا ہے۔
# اس میں analytical IK solution اور walking trajectory generation شامل ہے۔
```

## 5.3 Bipedal Locomotion and Balance Control (Bipedal Locomotion اور Balance Control)

Bipedal locomotion represents one of the most challenging problems in robotics due to the inherent instability of two-point contact. Unlike wheeled robots that are inherently stable, bipedal robots must continuously balance while transitioning between contact states. The control of walking requires coordinating multiple subsystems: trajectory generation, balance control, foot placement, and disturbance rejection.

Bipedal locomotion robotics میں سب سے چیلنجنگ problems میں سے ایک ہے کیونکہ two-point contact کی inherent instability کی وجہ سے۔ Inherently stable wheeled robots کے برعکس، bipedal robots کو contact states کے درمیان transition کرتے  ہوئے continuously balance رکھنا ہوتا ہے۔ Walking کے control کے لیے multiple subsystems coordinate کرنے کی ضرورت ہے: trajectory generation، balance control، foot placement اور disturbance rejection۔

### Zero Moment Point and Stability (Zero Moment Point اور Stability)

The Zero Moment Point (ZMP) is the fundamental concept for stable bipedal walking. The ZMP is the point on the ground where the total moment from gravitational and inertial forces equals zero. For stable walking, the ZMP must remain within the support polygon defined by the contact area between the feet and the ground.

Zero Moment Point (ZMP) stable bipedal walking کے لیے fundamental concept ہے۔ ZMP وہ point ہے زمین پر جہاں gravitational اور inertial forces کی کل moment صفر کے برابر ہوتی ہے۔ Stable walking کے لیے، ZMP کو support polygon کے اندر رہنا چاہیے جو feet اور ground کے درمیان contact area سے define ہوتی ہے۔

Modern walking controllers extend beyond static ZMP to incorporate preview control, model-predictive control, and learning-based approaches. The Unitree H1 robot uses a combination of simplified dynamics models and real-time optimization to achieve stable walking at speeds up to 2 m/s.

Modern walking controllers static ZMP سے آگے بڑھ کر preview control، model-predictive control اور learning-based approaches incorporate کرتے ہیں۔ Unitree H1 robot simplified dynamics models اور real-time optimization کے combination کا use کرتی ہے تاکہ 2 m/s تک speeds پر stable walking achieve کر سکے۔

### Walking Controller Implementation (Walking Controller Implementation)

```python
"""
Bipedal Walking Controller for Humanoid Robots (انسان نما روبوٹس کے لیے Bipedal Walking Controller)

یہ module ایک مکمل walking controller implement کرتا ہے جس میں ZMP-based balance، footstep planning اور trajectory generation شامل ہے۔
"""

import numpy as np
from typing import Dict, Tuple, Optional, List
from dataclasses import dataclass, field
from enum import Enum
import time


class GaitType(Enum):
    """Types of walking gaits."""
    STANDING = "standing"
    WALKING = "walking"
    TROTTING = "trotting"
    RUNNING = "running"


@dataclass
class WalkingParameters:
    """Parameters for walking controller."""
    # Timing
    step_duration: float = 0.4        # Time per step (seconds) - ہر step پر time (seconds)
    double_support_ratio: float = 0.2 # Fraction in double support - double support میں fraction
    sample_period: float = 0.005      # Control period (seconds) - Control period (seconds)

    # Geometry
    step_length: float = 0.25         # Forward step size (meters) - آگے step size (meters)
    step_width: float = 0.12          # Lateral foot separation (meters) - Lateral foot separation (meters)
    foot_length: float = 0.25         # Foot length (meters) - Foot length (meters)
    foot_width: float = 0.12          # Foot width (meters) - Foot width (meters)

    # Motion limits
    max_step_length: float = 0.35     # Maximum forward step - زیادہ سے زیادہ آگے step
    max_lateral_step: float = 0.15    # Maximum lateral step - زیادہ سے زیادہ lateral step
    max_turn_rate: float = 0.5        # Maximum turning (rad/s) - زیادہ سے زیادہ turning (rad/s)

    # Balance parameters
    com_height: float = 0.85          # Center of mass height - Center of mass height
    zmp_margin: float = 0.02          # Safety margin for ZMP - ZMP کے لیے safety margin
    preview_horizon: int = 16         # Preview steps for MPC - MPC کے لیے preview steps

    # Tracking gains
    com_kp: float = 10.0              # CoM position gain - CoM position gain
    com_kd: float = 3.0               # CoM velocity gain - CoM velocity gain
    orientation_kp: float = 20.0      # Orientation tracking gain - Orientation tracking gain
    orientation_kd: float = 5.0       # Orientation rate gain - Orientation rate gain


@dataclass
class RobotState:
    """Current state of the robot."""
    # Base position and orientation
    position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.95]))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))

    # Base velocity (world frame)
    linear_velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

    # Joint states
    joint_positions: Dict[str, float] = field(default_factory=dict)
    joint_velocities: Dict[str, float] = field(default_factory=dict)

    # Contact states
    left_foot_contact: bool = True
    right_foot_contact: bool = True

    # ZMP in world frame
    left_foot_zmp: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    right_foot_zmp: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))


@dataclass
class WalkingCommand:
    """Command input to walking controller."""
    forward_velocity: float = 0.0     # m/s
    lateral_velocity: float = 0.0     # m/s
    turn_rate: float = 0.0           # rad/s
    gait_type: GaitType = GaitType.WALKING
    emergency_stop: bool = False


class WalkingController:
    """
    Complete bipedal walking controller.

    ZMP-based balance control، footstep planning اور
    stable humanoid walking کے لیے trajectory tracking implement کرتا ہے۔
    """

    def __init__(self, params: Optional[WalkingParameters] = None):
        self.params = params or WalkingParameters()

        # State
        self.state = RobotState()
        self.command = WalkingCommand()

        # Footstep plan
        self.footstep_plan: List[Dict] = []
        self.current_step_idx: int = 0
        self.step_start_time: float = 0.0

        # Initialize foot positions
        self.left_foot_pos = np.array([-0.02, self.params.step_width / 2, 0.0])
        self.right_foot_pos = np.array([-0.02, -self.params.step_width / 2, 0.0])

        # Inverted pendulum model for balance
        self.Z = self.params.com_height / 9.81  # Pendulum time constant

        # Trajectory buffers
        self.trajectory_buffer: Dict[str, np.ndarray] = {}

    def reset(self, initial_state: Optional[RobotState] = None) -> None:
        """Reset controller to initial state."""
        if initial_state is not None:
            self.state = initial_state

        self.footstep_plan = []
        self.current_step_idx = 0
        self.step_start_time = 0.0

        # Reset foot positions
        self.left_foot_pos = np.array([0.0, self.params.step_width / 2, 0.0])
        self.right_foot_pos = np.array([0.0, -self.params.step_width / 2, 0.0])

    def set_command(self, command: WalkingCommand) -> None:
        """Update walking command."""
        self.command = command

    def plan_footsteps(self, n_steps: int = 10) -> None:
        """
        Plan future footstep positions based on command.

        command کی بنیاد پر future footstep positions plan کریں۔

        Uses velocity commands to predict future foot placements.
        velocity commands use کرتا ہے future foot placements predict کرنے کے لیے۔
        """
        self.footstep_plan = []

        # Determine swing foot
        swing_foot = "left" if self.current_step_idx % 2 == 0 else "right"

        for i in range(n_steps):
            # Step number from current
            step_num = self.current_step_idx + i

            # Base position (continue pattern)
            if step_num == 0:
                # First step - place swing foot
                target_pos = self.right_foot_pos.copy() if swing_foot == "right" else self.left_foot_pos.copy()
                target_pos[0] += self.command.forward_velocity * self.params.step_duration
                target_pos[1] += self.command.lateral_velocity * self.params.step_duration
            else:
                # Subsequent steps
                prev_foot = self.right_foot_pos if swing_foot == "left" else self.left_foot_pos
                target_pos = prev_foot.copy()
                target_pos[0] += self.command.forward_velocity * self.params.step_duration
                target_pos[1] += self.command.lateral_velocity * self.params.step_duration

                # Apply turning
                if abs(self.command.turn_rate) > 1e-3:
                    # Arc path for turning
                    radius = self.command.forward_velocity / (self.command.turn_rate + 1e-6)
                    angle = self.command.turn_rate * self.params.step_duration

                    # Transform
                    dx = target_pos[0] - 0  # Pivot around origin
                    dy = target_pos[1] - radius * (1 - np.cos(angle) if radius > 0 else 0)

                    cos_a = np.cos(angle)
                    sin_a = np.sin(angle)

                    target_pos[0] = dx * cos_a - dy * sin_a + radius * sin_a
                    target_pos[1] = dx * sin_a + dy * cos_a - radius * (1 - cos_a) if radius > 0 else dy

            # Clamp to limits
            target_pos[0] = np.clip(
                target_pos[0],
                -self.params.max_step_length,
                self.params.max_step_length
            )
            target_pos[1] = np.clip(
                target_pos[1],
                -self.params.max_lateral_step,
                self.params.max_lateral_step
            )

            self.footstep_plan.append({
                "position": target_pos,
                "swing_foot": swing_foot,
                "step_num": step_num
            })

            # Alternate swing foot
            swing_foot = "right" if swing_foot == "left" else "left"

    def compute_zmp_trajectory(self) -> np.ndarray:
        """
        Compute desired ZMP trajectory based on footstep plan.

        footstep plan کی بنیاد پر desired ZMP trajectory compute کریں۔

        Uses linear ZMP preview over the support polygon.
        support polygon پر linear ZMP preview use کرتا ہے۔
        """
        # Desired ZMP trajectory (N x 2 array)
        n_preview = self.params.preview_horizon
        zmp_trajectory = np.zeros((n_preview, 2))

        # Get current support foot
        support_foot = "right" if self.current_step_idx % 2 == 0 else "left"
        support_pos = self.right_foot_pos if support_foot == "right" else self.left_foot_pos

        for i in range(n_preview):
            step_idx = min(self.current_step_idx + i, len(self.footstep_plan) - 1)
            step = self.footstep_plan[step_idx]

            # ZMP starts at support foot center
            zmp_trajectory[i, :2] = support_pos[:2]

            # During swing, ZMP moves toward swing foot
            swing_foot = step["swing_foot"]
            swing_pos = self.left_foot_pos if swing_foot == "left" else self.right_foot_pos

            # Normalized phase in step
            phase = (i + 1) / n_preview

            if phase < self.params.double_support_ratio:
                # Double support - ZMP moves between feet
                zmp_trajectory[i, :2] = (
                    (1 - phase / self.params.double_support_ratio) * support_pos[:2] +
                    (phase / self.params.double_support_ratio) * swing_pos[:2]
                )
            else:
                # Single support on swing foot
                zmp_trajectory[i, :2] = swing_pos[:2]

        return zmp_trajectory

    def compute_com_reference(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute reference CoM position and velocity.

        reference CoM position اور velocity compute کریں۔

        Uses simplified linear inverted pendulum model.
        simplified linear inverted pendulum model use کرتا ہے۔
        """
        # Get ZMP target
        zmp_desired = self.compute_zmp_trajectory()[0]

        # Linear inverted pendulum dynamics
        # x_ddot = (g/z_c) * (x - x_zmp)
        # Solution: x(t) = x_zmp + (x0 - x_zmp) * cosh(t/tau)
        # where tau = sqrt(z_c / g)

        tau = np.sqrt(self.params.com_height / 9.81)
        exp_t = np.exp(t / tau)

        # Reference CoM position
        com_ref = np.array([
            zmp_desired[0] + (self.state.position[0] - zmp_desired[0]) * exp_t,
            zmp_desired[1] + (self.state.position[1] - zmp_desired[1]) * exp_t,
            self.params.com_height
        ])

        # Reference CoM velocity
        com_vel_ref = np.array([
            (self.state.position[0] - zmp_desired[0]) * exp_t / tau,
            (self.state.position[1] - zmp_desired[1]) * exp_t / tau,
            0.0
        ])

        return com_ref, com_vel_ref

    def compute_balance_correction(
        self,
        com_error: np.ndarray,
        com_vel_error: np.ndarray,
        orientation_error: np.ndarray,
        angular_vel_error: np.ndarray
    ) -> np.ndarray:
        """
        Compute balance correction forces/torques.

        balance correction forces/torques compute کریں۔

        Uses PD control on CoM and orientation errors.
        CoM اور orientation errors پر PD control use کرتا ہے۔
        """
        # CoM balance correction
        f_com = np.zeros(3)
        f_com[:2] = (
            self.params.com_kp * com_error[:2] +
            self.params.com_kd * com_vel_error[:2]
        )

        # Orientation correction (torque)
        tau_orientation = (
            self.params.orientation_kp * orientation_error +
            self.params.orientation_kd * angular_vel_error
        )

        return np.concatenate([f_com, tau_orientation])

    def generate_foot_trajectory(
        self,
        swing_foot: str,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        t: float
    ) -> np.ndarray:
        """
        Generate trajectory for swinging foot.

        swinging foot کے لیے trajectory generate کریں۔
        """
        # Linear interpolation in XY
        x = start_pos[0] + t * (end_pos[0] - start_pos[0])
        y = start_pos[1] + t * (end_pos[1] - start_pos[1])

        # Parabolic lift for clearance
        # Maximum height at t=0.5
        lift = 4 * self.params.step_height * t * (1 - t)

        z = start_pos[2] + lift

        return np.array([x, y, z])

    def step(self, dt: float) -> Dict[str, np.ndarray]:
        """
        Execute one control step.

        ایک control step execute کریں۔
        """
        # Emergency stop
        if self.command.emergency_stop:
            return self._generate_stand_command()

        # Update state estimate (placeholder for actual sensor fusion)
        self._update_state_estimate()

        # Update footstep plan
        if len(self.footstep_plan) < 5:
            self.plan_footsteps()

        # Get current step info
        current_step = self.footstep_plan[0]
        swing_foot = current_step["swing_foot"]
        swing_target = current_step["position"]

        # Determine support foot
        support_foot = "right" if swing_foot == "left" else "left"
        support_pos = self.right_foot_pos if support_foot == "right" else self.left_foot_pos

        # Normalized phase in step
        elapsed = time.time() - self.step_start_time
        phase = min(elapsed / self.params.step_duration, 1.0)

        # Compute reference trajectories
        com_ref, com_vel_ref = self.compute_com_reference(phase)

        # Compute errors
        com_error = com_ref - self.state.position
        com_vel_error = com_vel_ref - self.state.linear_velocity

        # Orientation error (simplified)
        orientation_error = np.zeros(3)

        # Compute balance correction
        balance_correction = self.compute_balance_correction(
            com_error, com_vel_error,
            orientation_error, self.state.angular_velocity
        )

        # Generate foot positions based on phase
        if phase < self.params.double_support_ratio:
            # Double support phase
            # Both feet on ground, shift weight
            left_target = support_pos if support_foot == "left" else self._interpolate_foot(
                self.left_foot_pos, swing_target, phase / self.params.double_support_ratio
            )
            right_target = support_pos if support_foot == "right" else self._interpolate_foot(
                self.right_foot_pos, swing_target, phase / self.params.double_support_ratio
            )
        else:
            # Single support phase
            support_pos = self.left_foot_pos if support_foot == "left" else self.right_foot_pos
            swing_pos = self._interpolate_foot(
                self.left_foot_pos if swing_foot == "left" else self.right_foot_pos,
                swing_target,
                (phase - self.params.double_support_ratio) /
                (1.0 - self.params.double_support_ratio)
            )

            left_target = swing_pos if swing_foot == "left" else support_pos
            right_target = swing_pos if swing_foot == "right" else support_pos

        # Update foot positions
        if swing_foot == "left":
            self.left_foot_pos = left_target
        else:
            self.right_foot_pos = right_target

        # Check for step completion
        if phase >= 1.0:
            self.current_step_idx += 1
            self.step_start_time = time.time()
            self.footstep_plan.pop(0)

        # Generate joint commands from foot positions
        joint_commands = self._foot_positions_to_joints(left_target, right_target)

        # Add balance compensation
        joint_commands = self._add_balance_compensation(joint_commands, balance_correction)

        return joint_commands

    def _update_state_estimate(self) -> None:
        """Update internal state estimate (placeholder)."""
        # In a real system, this would integrate IMU, encoder, and vision data
        pass

    def _interpolate_foot(
        self,
        start: np.ndarray,
        end: np.ndarray,
        t: float
    ) -> np.ndarray:
        """Linear interpolation between foot positions."""
        return start + t * (end - start)

    def _foot_positions_to_joints(
        self,
        left_foot: np.ndarray,
        right_foot: np.ndarray
    ) -> Dict[str, np.ndarray]:
        """Convert foot positions to joint angle commands."""
        # Simplified IK - would use full IK solver in practice
        joint_commands = {}

        # Standing pose
        if np.linalg.norm(left_foot[:2]) < 0.01 and np.linalg.norm(right_foot[:2]) < 0.01:
            joint_commands = {
                "left_hip_roll": 0.0,
                "left_hip_pitch": 0.0,
                "left_knee": 0.0,
                "left_ankle_pitch": 0.0,
                "left_ankle_roll": 0.0,
                "right_hip_roll": 0.0,
                "right_hip_pitch": 0.0,
                "right_knee": 0.0,
                "right_ankle_pitch": 0.0,
                "right_ankle_roll": 0.0,
            }
        else:
            # Walking pose (simplified)
            joint_commands = {
                "left_hip_roll": 0.0,
                "left_hip_pitch": -0.1 * left_foot[0],
                "left_knee": 0.2 * max(0, -left_foot[2]),
                "left_ankle_pitch": 0.1 * left_foot[0],
                "left_ankle_roll": 0.0,
                "right_hip_roll": 0.0,
                "right_hip_pitch": -0.1 * right_foot[0],
                "right_knee": 0.2 * max(0, -right_foot[2]),
                "right_ankle_pitch": 0.1 * right_foot[0],
                "right_ankle_roll": 0.0,
            }

        return joint_commands

    def _add_balance_compensation(
        self,
        commands: Dict[str, float],
        correction: np.ndarray
    ) -> Dict[str, np.ndarray]:
        """Add balance compensation to joint commands."""
        # Convert balance correction to joint adjustments
        compensated = {k: np.array([v]) for k, v in commands.items()}

        # Simplified: bias hip joints for balance
        compensated["left_hip_roll"] += correction[1] * 0.01
        compensated["right_hip_roll"] -= correction[1] * 0.01
        compensated["waist_pitch"] += correction[4] * 0.005

        return {k: float(v) for k, v in compensated.items()}

    def _generate_stand_command(self) -> Dict[str, np.ndarray]:
        """Generate joint commands for standing pose."""
        return {
            "left_hip_roll": 0.0,
            "left_hip_pitch": 0.0,
            "left_knee": 0.0,
            "left_ankle_pitch": 0.0,
            "left_ankle_roll": 0.0,
            "right_hip_roll": 0.0,
            "right_hip_pitch": 0.0,
            "right_knee": 0.0,
            "right_ankle_pitch": 0.0,
            "right_ankle_roll": 0.0,
        }


if __name__ == "__main__":
    # Test walking controller
    params = WalkingParameters()
    controller = WalkingController(params)

    # Set walking command
    command = WalkingCommand(
        forward_velocity=0.3,
        lateral_velocity=0.0,
        turn_rate=0.0
    )
    controller.set_command(command)

    print("Walking Controller Test")
    print("=" * 50)

    # Simulate a few steps
    for step in range(10):
        commands = controller.step(dt=0.005)

        if step % 8 == 0:
            print(f"Step {step}:")
            for name, angle in list(commands.items())[:4]:
                print(f"  {name}: {np.degrees(angle):.1f} deg")

    # Test different commands
    print("\nTurning in place:")
    command = WalkingCommand(turn_rate=0.3)
    controller.set_command(command)

    for i in range(5):
        controller.step(dt=0.005)

    print("Walking controller test complete")

# یہ module humanoid robots کے لیے bipedal walking controller implement کرتا ہے۔
# اس میں ZMP-based balance control، footstep planning اور trajectory tracking شامل ہے۔
```

## 5.4 Manipulation and Grasping with Humanoid Hands (Humanoid Hands کے ساتھ Manipulation اور Grasping)

Humanoid manipulation aims to replicate the versatility of human hand capabilities. Unlike industrial grippers optimized for specific tasks, humanoid hands must handle diverse objects, adapt grasp types, and perform manipulation in unstructured environments. This section covers the fundamentals of manipulation planning, grasp synthesis, and dextrous manipulation control.

Humanoid manipulation human hand capabilities کی versatility replicate کرنے کی کوشش کرتی ہے۔ Specific tasks کے لیے optimized industrial grippers کے برعکس، humanoid hands کو diverse objects handle کرنے ہیں، grasp types adapt کرنے ہیں، اور unstructured environments میں manipulation perform کرنا ہے۔ یہ section manipulation planning، grasp synthesis اور dextrous manipulation control کی fundamentals cover کرتی ہے۔

### Grasping Fundamentals (Grasping Fundamentals)

Grasping involves positioning the hand relative to an object and applying forces to constrain object motion. The quality of a grasp is measured by its ability to resist external disturbances while accommodating object uncertainty. Force closure grasps can resist arbitrary disturbance forces, while form closure depends on geometric constraints.

Grasping میں object کے relative hand کو position کرنا اور forces apply کرنا شامل ہے تاکہ object motion constrain ہو۔ Grasp کی quality اس کی ability سے measure ہوتی ہے کہ external disturbances resist کر سکے جبکہ object uncertainty accommodate کرے۔ Force closure grasps arbitrary disturbance forces resist کر سکتی ہیں، جبکہ form closure geometric constraints پر depend کرتی ہے۔

Human hands typically perform three grasp types:

Human hands عام طور پر تین grasp types perform کرتی ہیں:

Power grasps wrap fingers around objects for high-force, low-precision tasks. Precision grasps use fingertips for delicate manipulation. Intermediate grasps balance force and precision for everyday tasks like tool use.

Power grasps objects کے گرد fingers wrap کرتی ہیں high-force، low-precision tasks کے لیے۔ Precision grasps delicate manipulation کے لیے fingertips use کرتی ہیں۔ Intermediate grasps force اور precision balance کرتی ہیں everyday tasks جیسے tool use کے لیے۔

### Grasp Planning Implementation (Grasp Planning Implementation)

```python
"""
Grasp Planning for Humanoid Robot Hands (انسان نما روبوٹ hands کے لیے Grasp Planning)

یہ module grasp synthesis algorithms implement کرتا ہے جس میں grasp quality evaluation اور manipulation planning شامل ہے۔
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum
import scipy.optimize as optimize


class GraspType(Enum):
    """Types of grasps for humanoid manipulation."""
    POWER_GRASP = "power"       # Whole-hand wrap - پورا ہاتھ لپیٹنا
    PRECISION_GRASP = "precision"  # Fingertip grasp - Fingertip grasp
    LATERAL_GRASP = "lateral"   # Pinch grasp - Pinch grasp
    HOOK_GRASP = "hook"         # Hook for carrying - اٹھانے کے لیے hook


@dataclass
class GraspPoint:
    """A single contact point on the hand."""
    position: np.ndarray   # Contact position in hand frame - Hand frame میں contact position
    normal: np.ndarray     # Surface normal (outward from hand) - Surface normal (hand سے outward)
    finger_name: str       # Which finger - کون سا finger


@dataclass
class GraspCandidate:
    """A complete grasp candidate with multiple contact points."""
    grasp_type: GraspType
    contacts: List[GraspPoint]
    pregrasp_position: np.ndarray   # Hand pose before grasping - Grabbing سے پہلے hand pose
    grasp_position: np.ndarray      # Hand pose during grasp - Grasp کے دوران hand pose

    # Grasp quality metrics
    force_closure: bool = False
    grasp_quality: float = 0.0
    manipulability: float = 0.0

    # Object properties (estimated)
    object_radius: float = 0.0
    object_weight: float = 0.0


@dataclass
class HandGeometry:
    """Geometry specification for humanoid hand."""
    # Finger lengths (meters)
    palm_width: float = 0.08
    palm_length: float = 0.10

    # Thumb
    thumb_length: float = 0.06
    thumb_proximal: float = 0.04
    thumb_distal: float = 0.04

    # Fingers (index, middle, ring, pinky)
    finger_lengths: List[float] = field(default_factory=lambda: [0.05, 0.06, 0.055, 0.04])
    proximal_segments: List[float] = field(default_factory=lambda: [0.03, 0.035, 0.03, 0.025])
    distal_segments: List[float] = field(default_factory=lambda: [0.025, 0.03, 0.028, 0.02])

    # Joint limits (radians)
    mcp_flexion: float = np.pi / 3      # Metacarpophalangeal
    pip_flexion: float = np.pi / 2      # Proximal interphalangeal
    dip_flexion: float = np.pi / 2      # Distal interphalangeal
    thumb_opposition: float = np.pi / 3


class GraspPlanner:
    """
    Grasp planning for humanoid robot hands.

    object properties کی بنیاد پر grasp synthesis اور
    grasp quality evaluation implement کرتا ہے۔
    """

    def __init__(self, hand_geometry: Optional[HandGeometry] = None):
        self.hand = hand_geometry or HandGeometry()

    def compute_grasp_candidates(
        self,
        object_position: np.ndarray,
        object_radius: float,
        object_weight: float = 0.5,
        n_candidates: int = 10
    ) -> List[GraspCandidate]:
        """
        Generate grasp candidates for an object.

        object کے لیے grasp candidates generate کریں۔
        """
        candidates = []

        # Generate different grasp types
        for grasp_type in GraspType:
            for i in range(n_candidates // len(GraspType)):
                candidate = self._synthesize_grasp(
                    grasp_type, object_position, object_radius, object_weight
                )
                if candidate:
                    candidate.grasp_quality = self._evaluate_grasp_quality(candidate)
                    candidates.append(candidate)

        # Sort by quality and return top candidates
        candidates.sort(key=lambda c: c.grasp_quality, reverse=True)

        return candidates[:n_candidates]

    def _synthesize_grasp(
        self,
        grasp_type: GraspType,
        object_position: np.ndarray,
        object_radius: float,
        object_weight: float
    ) -> Optional[GraspCandidate]:
        """Synthesize a single grasp candidate."""
        # Approach direction (from hand to object)
        approach_dir = np.array([1.0, 0.0, 0.0])  # Default approach

        # Generate hand pose based on grasp type
        if grasp_type == GraspType.POWER_GRASP:
            return self._power_grasp(object_position, object_radius, approach_dir)
        elif grasp_type == GraspType.PRECISION_GRASP:
            return self._precision_grasp(object_position, object_radius, approach_dir)
        elif grasp_type == GraspType.LATERAL_GRASP:
            return self._lateral_grasp(object_position, object_radius, approach_dir)
        elif grasp_type == GraspType.HOOK_GRASP:
            return self._hook_grasp(object_position, object_radius, approach_dir)

        return None

    def _power_grasp(
        self,
        object_position: np.ndarray,
        radius: float,
        approach_dir: np.ndarray
    ) -> GraspCandidate:
        """Synthesize power grasp configuration."""
        # Power grasp: wrap fingers around object
        contacts = []

        # Thumb contact
        thumb_pos = object_position + np.array([-radius - 0.02, 0.03, 0])
        thumb_normal = np.array([1.0, 0.0, 0.0])
        contacts.append(GraspPoint(thumb_pos, thumb_normal, "thumb"))

        # Finger contacts (wrap around)
        n_fingers = 4
        for i, length in enumerate(self.hand.finger_lengths):
            angle = (i - n_fingers / 2 + 0.5) * 0.08  # Spread angle

            # Contact position on finger
            finger_pos = object_position + np.array([
                -radius - 0.01,
                angle * radius * 2,
                length * 0.3
            ])
            finger_normal = np.array([1.0, -angle, 0.0])
            finger_normal /= np.linalg.norm(finger_normal)

            contacts.append(GraspPoint(finger_pos, finger_normal, f"finger_{i}"))

        # Hand pose
        grasp_pose = object_position + np.array([-radius - 0.05, 0, 0])
        pregrasp_pose = grasp_pose + np.array([-0.1, 0, 0])

        return GraspCandidate(
            grasp_type=GraspType.POWER_GRASP,
            contacts=contacts,
            pregrasp_position=pregrasp_pose,
            grasp_position=grasp_pose,
            object_radius=radius,
            object_weight=0.5
        )

    def _precision_grasp(
        self,
        object_position: np.ndarray,
        radius: float,
        approach_dir: np.ndarray
    ) -> GraspCandidate:
        """Synthesize precision (pinch) grasp configuration."""
        contacts = []

        # Thumb and index finger pinch
        thumb_pos = object_position + np.array([0, 0.02, radius + 0.01])
        thumb_normal = np.array([0.0, 0.0, -1.0])
        contacts.append(GraspPoint(thumb_pos, thumb_normal, "thumb"))

        index_pos = object_position + np.array([0, -0.02, radius + 0.01])
        index_normal = np.array([0.0, 0.0, -1.0])
        contacts.append(GraspPoint(index_pos, index_normal, "finger_0"))

        # Hand pose
        grasp_pose = object_position + np.array([0, 0, radius + 0.05])
        pregrasp_pose = grasp_pose + np.array([0, 0, 0.1])

        return GraspCandidate(
            grasp_type=GraspType.PRECISION_GRASP,
            contacts=contacts,
            pregrasp_position=pregrasp_pose,
            grasp_position=grasp_pose,
            object_radius=radius,
            object_weight=0.1
        )

    def _lateral_grasp(
        self,
        object_position: np.ndarray,
        radius: float,
        approach_dir: np.ndarray
    ) -> GraspCandidate:
        """Synthesize lateral (key pinch) grasp configuration."""
        contacts = []

        # Thumb against side of object, index finger on other side
        thumb_pos = object_position + np.array([radius + 0.01, 0, 0])
        thumb_normal = np.array([-1.0, 0.0, 0.0])
        contacts.append(GraspPoint(thumb_pos, thumb_normal, "thumb"))

        index_pos = object_position + np.array([-(radius + 0.01), 0, 0.02])
        index_normal = np.array([1.0, 0.0, 0.0])
        contacts.append(GraspPoint(index_pos, index_normal, "finger_0"))

        # Hand pose (rotated for lateral pinch)
        grasp_pose = object_position + np.array([0, 0, radius + 0.03])
        pregrasp_pose = grasp_pose + np.array([0, 0, 0.08])

        return GraspCandidate(
            grasp_type=GraspType.LATERAL_GRASP,
            contacts=contacts,
            pregrasp_position=pregrasp_pose,
            grasp_position=grasp_pose,
            object_radius=radius,
            object_weight=0.2
        )

    def _hook_grasp(
        self,
        object_position: np.ndarray,
        radius: float,
        approach_dir: np.ndarray
    ) -> GraspCandidate:
        """Synthesize hook grasp for carrying."""
        contacts = []

        # Fingers curled under object
        for i, length in enumerate(self.hand.finger_lengths[:3]):
            contact_pos = object_position + np.array([
                -0.02,
                (i - 1) * 0.02,
                -(length * 0.8 + radius)
            ])
            contact_normal = np.array([0.0, 0.0, 1.0])
            contacts.append(GraspPoint(contact_pos, contact_normal, f"finger_{i}"))

        # Hand pose (above object)
        grasp_pose = object_position + np.array([0, 0, radius + 0.05])
        pregrasp_pose = grasp_pose + np.array([0, 0, 0.1])

        return GraspCandidate(
            grasp_type=GraspType.HOOK_GRASP,
            contacts=contacts,
            pregrasp_position=pregrasp_pose,
            grasp_position=grasp_pose,
            object_radius=radius,
            object_weight=0.5
        )

    def _evaluate_grasp_quality(self, grasp: GraspCandidate) -> float:
        """
        Evaluate grasp quality.

        grasp quality evaluate کریں۔

        Combines force closure, contact distribution, and
        manipulability metrics۔
        force closure، contact distribution اور
        manipulability metrics کو combine کرتا ہے۔
        """
        quality = 0.0

        # 1. Force closure evaluation
        if len(grasp.contacts) >= 3:
            # Check if contacts form a stable configuration
            contact_positions = np.array([c.position for c in grasp.contacts])
            contact_normals = np.array([c.normal for c in grasp.contacts])

            # Compute contact convex hull area (proxy for stability)
            try:
                hull = contact_positions[:3]  # Use first 3 contacts
                area = np.linalg.norm(np.cross(
                    hull[1] - hull[0],
                    hull[2] - hull[0]
                )) / 2
                quality += min(area * 10, 0.3)  # Cap contribution
            except:
                pass

        # 2. Contact normal alignment
        normals = np.array([c.normal for c in grasp.contacts])
        if len(normals) > 0:
            # Prefer normals pointing toward object center
            center = np.mean([c.position for c in grasp.contacts])
            to_center = center - np.array(grasp.grasp_position)
            to_center /= np.linalg.norm(to_center)

            alignment = np.mean([np.dot(c.normal, to_center) for c in grasp.contacts])
            quality += max(alignment * 0.2, 0.0)

        # 3. Grasp type appropriateness for object weight
        if grasp.grasp_type == GraspType.POWER_GRASP and grasp.object_weight > 0.3:
            quality += 0.2
        elif grasp.grasp_type == GraspType.PRECISION_GRASP and grasp.object_weight < 0.2:
            quality += 0.2

        # 4. Manipulability (finger spread)
        finger_positions = [c.position for c in grasp.contacts if "finger" in c.finger_name]
        if len(finger_positions) >= 2:
            spread = np.std([p[1] for p in finger_positions])
            quality += min(spread * 5, 0.2)

        return min(quality, 1.0)

    def select_best_grasp(
        self,
        candidates: List[GraspCandidate],
        constraints: Optional[Dict] = None
    ) -> GraspCandidate:
        """
        Select best grasp given constraints.

        constraints کی بنیاد پر best grasp select کریں۔
        """
        constraints = constraints or {}

        # Filter by constraints first
        filtered = candidates

        if constraints.get("force_closure", False):
            filtered = [c for c in filtered if c.force_closure]

        if "approach_direction" in constraints:
            approach = np.array(constraints["approach_direction"])
            for c in filtered:
                approach_vec = c.grasp_position - c.pregrasp_position
                approach_vec /= np.linalg.norm(approach_vec)
                if np.dot(approach, approach_vec) < 0:
                    filtered.remove(c)

        # Return highest quality from filtered
        if filtered:
            return max(filtered, key=lambda c: c.grasp_quality)

        # Fallback to highest quality
        return max(candidates, key=lambda c: c.grasp_quality)


if __name__ == "__main__":
    # Test grasp planning
    hand = HandGeometry()
    planner = GraspPlanner(hand)

    # Object parameters
    object_pos = np.array([0.5, 0.0, 0.8])  # Table height
    object_radius = 0.04  # Small object

    # Generate candidates
    candidates = planner.compute_grasp_candidates(
        object_pos, object_radius, n_candidates=10
    )

    print("Grasp Planning Test")
    print("=" * 50)
    print(f"Generated {len(candidates)} grasp candidates")

    for i, candidate in enumerate(candidates[:3]):
        print(f"\nGrasp {i + 1}: {candidate.grasp_type.value}")
        print(f"  Quality: {candidate.grasp_quality:.3f}")
        print(f"  Contacts: {len(candidate.contacts)}")
        print(f"  Pregrasp: {candidate.pregrasp_position}")
        print(f"  Grasp: {candidate.grasp_position}")

    # Select best grasp
    best = planner.select_best_grasp(candidates)
    print(f"\nBest grasp: {best.grasp_type.value} (quality: {best.grasp_quality:.3f})")

# یہ module humanoid robot hands کے لیے grasp planning implement کرتا ہے۔
# اس میں grasp synthesis، quality evaluation اور manipulation planning شامل ہے۔
```

## 5.5 Natural Human-Robot Interaction Design (قدرتی Human-Robot Interaction Design)

Human-robot interaction (HRI) for humanoid robots must account for the unique expectations humans have when interacting with anthropomorphic systems. Unlike industrial robots that operate in separated workspaces, humanoids share environments with people and must communicate intentions, understand human behavior, and respond naturally.

انسان نما robots کے لیے human-robot interaction (HRI) کو ان منفرد expectations کا account رکھنا چاہیے جو humans کو anthropomorphic systems کے ساتھ interact کرتے وقت ہوتی ہیں۔ Separated workspaces میں operate کرنے والے industrial robots کے برعکس، humanoids people کے ساتھ environments share کرتے ہیں اور intentions communicate کرنے، human behavior understand کرنے اور naturally respond کرنے چاہییں۔

### Principles of Natural HRI (قدرتی HRI کے اصول)

Natural interaction relies on familiar social cues that humans use with each other. Nonverbal communication including gaze, gestures, and posture provides important context for interaction. A humanoid robot that maintains appropriate eye contact during conversation, orients its body toward the person speaking, and uses hand gestures during explanations feels more natural and approachable.

Natural interaction familiar social cues پر rely کرتی ہے جو humans ایک دوسرے کے ساتھ use کرتے ہیں۔ Nonverbal communication جیسے gaze، gestures اور posture interaction کے لیے important context فراہم کرتی ہے۔ ایک انسان نما robot جو conversation کے دوران appropriate eye contact برقرار رکھے، speaker کی طرف اپنا body orient کرے، اور explanations کے دوران hand gestures use کرے، وہ زیادہ natural اور approachable محسوس ہوتا ہے۔

The design of HRI systems must balance expressiveness with predictability. Robots should communicate their state clearly so humans can anticipate actions, but excessive expressiveness can become distracting or misleading. The goal is transparency: humans should understand what the robot is doing, why, and what it will do next.

HRI systems کا design expressiveness اور predictability کے درمیان balance رکھنا چاہیے۔ Robots اپنا state clearly communicate کرنا چاہیے تاکہ humans actions anticipate کر سکیں، لیکن excessive expressiveness distracting یا misleading ہو سکتا ہے۔ Goal transparency ہے: humans کو سمجھنا چاہیے کہ robot کیا کر رہا ہے، کیوں، اور اگلے کیا کرے گا۔

### Multimodal Interaction Architecture (Multimodal Interaction Architecture)

```python
"""
Human-Robot Interaction System for Humanoid Robots (انسان نما روبوٹس کے لیے Human-Robot Interaction System)

یہ module multimodal interaction implement کرتا ہے جس میں speech، gesture recognition اور gaze behavior شامل ہے۔
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass, field
from dataclasses import dataclass
from enum import Enum
import time


class InteractionState(Enum):
    """State of human-robot interaction."""
    IDLE = "idle"
    DETECTING = "detecting"
    ENGAGED = "engaged"
    SPEAKING = "speaking"
    LISTENING = "listening"
    THINKING = "thinking"


@dataclass
class Person:
    """Representation of a detected person."""
    person_id: int
    position: np.ndarray        # 3D position
    orientation: np.ndarray     # Body orientation (yaw)
    gaze_direction: np.ndarray  # Where they're looking
    confidence: float           # Detection confidence
    timestamp: float            # Last detection time


@dataclass
class Utterance:
    """Speech utterance with metadata."""
    text: str
    speaker: str               # "robot" or person ID
    timestamp: float
    intent: Optional[str]      # Recognized intent
    entities: Dict[str, str]   # Extracted entities


@dataclass
class Gesture:
    """Recognized gesture with confidence."""
    gesture_type: str          # "point", "wave", "stop", etc.
    side: str                  # "left", "right", "both"
    start_time: float
    confidence: float
    target_position: Optional[np.ndarray]  # For pointing


class InteractionManager:
    """
    Manages natural human-robot interaction.

    speech، gaze، gesture اور body language coordinate کرتا ہے
    cohesive interaction experience کے لیے۔
    """

    def __init__(self):
        # State
        self.state = InteractionState.IDLE
        self.last_interaction_time: float = 0.0
        self.session_start_time: float = 0.0

        # Detected people
        self.people: Dict[int, Person] = {}
        self.engaged_person: Optional[int] = None

        # Interaction history
        self.utterances: List[Utterance] = []
        self.gestures: List[Gesture] = []

        # Gaze behavior parameters
        self.gaze_target_duration: float = 2.0  # Seconds to hold gaze - gaze hold کرنے کے seconds
        self.gaze_away_duration: float = 1.0    # Time before looking away - look away سے پہلے time
        self.last_gaze_shift: float = 0.0
        self.current_gaze_target: Optional[int] = None

        # Attention model
        self.attention_weights: Dict[int, float] = {}

        # Callbacks for action execution
        self.on_speak: Optional[Callable[[str], None]] = None
        self.on_gesture: Optional[Callable[[Gesture], None]] = None
        self.on_head_turn: Optional[Callable[[np.ndarray], None]] = None

        # Initialize attention
        self._initialize_attention()

    def _initialize_attention(self) -> None:
        """Initialize attention model with default values."""
        self.attention_decay = 0.95
        self.attention_boost = 0.3
        self.min_attention = 0.1

    def update_person(
        self,
        person_id: int,
        position: np.ndarray,
        orientation: np.ndarray,
        confidence: float = 1.0
    ) -> Person:
        """Update or add a detected person."""
        # Estimate gaze direction from body orientation
        gaze_dir = np.array([
            np.cos(orientation[2]),
            np.sin(orientation[2]),
            0.1  # Slight upward tilt
        ])
        gaze_dir /= np.linalg.norm(gaze_dir)

        person = Person(
            person_id=person_id,
            position=position,
            orientation=orientation,
            gaze_direction=gaze_dir,
            confidence=confidence,
            timestamp=time.time()
        )

        self.people[person_id] = person

        # Update attention
        if person_id in self.attention_weights:
            self.attention_weights[person_id] = min(
                self.attention_weights[person_id] + self.attention_boost, 1.0
            )
        else:
            self.attention_weights[person_id] = 0.5

        return person

    def remove_person(self, person_id: int) -> None:
        """Remove a person from tracking."""
        if person_id in self.people:
            del self.people[person_id]
        if person_id in self.attention_weights:
            del self.attention_weights[person_id]
        if self.engaged_person == person_id:
            self.engaged_person = None

    def update_attention(self) -> None:
        """Update attention weights based on time and events."""
        current_time = time.time()

        for person_id in self.people:
            if person_id not in self.attention_weights:
                self.attention_weights[person_id] = self.min_attention
            else:
                # Decay attention over time
                time_since_update = current_time - self.people[person_id].timestamp
                decay = self.attention_decay ** (time_since_update / 10.0)
                self.attention_weights[person_id] = max(
                    self.min_attention,
                    self.attention_weights[person_id] * decay
                )

        # Select most attentive person
        if self.attention_weights:
            self.engaged_person = max(
                self.attention_weights.keys(),
                key=lambda p: self.attention_weights[p]
            )

    def get_attended_person(self) -> Optional[Person]:
        """Get the person with highest attention."""
        self.update_attention()
        if self.engaged_person is not None:
            return self.people.get(self.engaged_person)
        return None

    def add_utterance(
        self,
        text: str,
        speaker: str,
        intent: Optional[str] = None
    ) -> Utterance:
        """Add an utterance to interaction history."""
        utterance = Utterance(
            text=text,
            speaker=speaker,
            timestamp=time.time(),
            intent=intent,
            entities={}
        )
        self.utterances.append(utterance)
        self.last_interaction_time = time.time()

        # Update state
        if speaker == "robot":
            self.state = InteractionState.SPEAKING
        else:
            self.state = InteractionState.LISTENING

        return utterance

    def add_gesture(self, gesture: Gesture) -> None:
        """Add a recognized gesture."""
        self.gestures.append(gesture)
        self.last_interaction_time = time.time()

        # React to specific gestures
        if gesture.gesture_type == "point":
            self._handle_point_gesture(gesture)
        elif gesture.gesture_type == "wave":
            self._handle_wave_gesture(gesture)
        elif gesture.gesture_type == "stop":
            self._handle_stop_gesture(gesture)

    def _handle_point_gesture(self, gesture: Gesture) -> None:
        """Handle pointing gesture."""
        # Look at pointed location
        if gesture.target_position is not None and self.on_head_turn:
            self.on_head_turn(gesture.target_position)

    def _handle_wave_gesture(self, gesture: Gesture) -> None:
        """Handle wave gesture."""
        # Engage with the person who waved
        self.state = InteractionState.ENGAGED
        self.session_start_time = time.time()

    def _handle_stop_gesture(self, gesture: Gesture) -> None:
        """Handle stop gesture."""
        # Stop current action
        self.state = InteractionState.IDLE

    def compute_gaze_target(self) -> Tuple[np.ndarray, str]:
        """
        Compute where the robot should look.

        robot کو کہاں دیکھنا چاہیے compute کریں۔
        """
        current_time = time.time()
        gaze_target = np.array([0.0, 0.0, 1.5])  # Default: forward
        target_desc = "neutral"

        # Time-based gaze shifting
        time_since_gaze_shift = current_time - self.last_gaze_shift

        if self.state == InteractionState.SPEAKING:
            # Look at engaged person while speaking
            person = self.get_attended_person()
            if person is not None:
                gaze_target = person.position + np.array([0, 0, 0.1])
                target_desc = f"person_{person.person_id}"

        elif self.state == InteractionState.LISTENING:
            # Maintain eye contact while listening
            person = self.get_attended_person()
            if person is not None:
                gaze_target = person.position + np.array([0, 0, 0.1])
                target_desc = f"person_{person.person_id}"

        elif self.state == InteractionState.IDLE:
            # Shift gaze periodically to show awareness
            if time_since_gaze_shift > self.gaze_target_duration:
                self.last_gaze_shift = current_time

                # Look at most attentive person
                person = self.get_attended_person()
                if person is not None:
                    gaze_target = person.position + np.array([0, 0, 0.1])
                    target_desc = f"person_{person.person_id}"
                elif self.people:
                    # Look at random person
                    person_id = list(self.people.keys())[0]
                    person = self.people[person_id]
                    gaze_target = person.position + np.array([0, 0, 0.1])
                    target_desc = f"person_{person.person_id}"

        return gaze_target, target_desc

    def generate_gaze_behavior(self) -> Dict:
        """
        Generate complete gaze behavior parameters.

        مکمل gaze behavior parameters generate کریں۔
        """
        gaze_target, target_desc = self.compute_gaze_target()

        return {
            "target_position": gaze_target,
            "target_description": target_desc,
            "duration": self.gaze_target_duration,
            "smoothness": 0.5,  # Smoothing factor for gaze motion - gaze motion کے لیے smoothing factor
            "eye_offset": np.array([0.0, 0.0, 0.02])  # Eye separation
        }

    def get_conversational_turn(self) -> str:
        """Get appropriate conversational behavior."""
        person = self.get_attended_person()

        if person is None:
            return "scan"

        # Check recent utterances
        recent = [u for u in self.utterances if time.time() - u.timestamp < 5.0]

        if not recent:
            return "greet"

        last_utterance = recent[-1]

        if last_utterance.speaker == "robot":
            # Robot just spoke, wait for response
            return "listen"
        else:
            # Person spoke, acknowledge
            return "acknowledge"

    def generate_body_language(
        self,
        behavior_type: str
    ) -> Dict[str, np.ndarray]:
        """
        Generate body language parameters.

        body language parameters generate کریں۔
        """
        if behavior_type == "nod":
            return {
                "neck_pitch": np.array([0.1, -0.1, 0.0]),
                "duration": 0.5
            }
        elif behavior_type == "shake":
            return {
                "neck_yaw": np.array([0.1, -0.1, 0.1, -0.1, 0.0]),
                "duration": 0.8
            }
        elif behavior_type == "tilt":
            return {
                "neck_roll": np.array([0.1]),
                "duration": 0.3
            }
        elif behavior_type == "lean_forward":
            return {
                "waist_pitch": np.array([0.1]),
                "duration": 0.5
            }
        elif behavior_type == "open_arms":
            return {
                "left_shoulder_pitch": np.array([-0.5]),
                "right_shoulder_pitch": np.array([-0.5]),
                "duration": 0.3
            }

        return {"duration": 0.0}

    def get_interaction_summary(self) -> Dict:
        """Get summary of current interaction state."""
        return {
            "state": self.state.value,
            "engaged_person": self.engaged_person,
            "n_people": len(self.people),
            "recent_utterances": len([
                u for u in self.utterances
                if time.time() - u.timestamp < 30.0
            ]),
            "session_duration": time.time() - self.session_start_time
            if self.session_start_time > 0 else 0.0
        }


if __name__ == "__main__":
    # Test interaction manager
    manager = InteractionManager()

    print("Human-Robot Interaction Test")
    print("=" * 50)

    # Simulate people detection
    manager.update_person(
        person_id=1,
        position=np.array([1.0, 0.5, 1.6]),
        orientation=np.array([0.0, 0.0, -0.3]),
        confidence=0.9
    )

    manager.update_person(
        person_id=2,
        position=np.array([1.2, -0.3, 1.6]),
        orientation=np.array([0.0, 0.0, 0.5]),
        confidence=0.85
    )

    print(f"Detected {len(manager.people)} people")

    # Update attention
    manager.update_attention()
    print(f"Engaged person: {manager.engaged_person}")

    # Add utterance
    manager.add_utterance("Hello!", "person_1")
    print(f"Interaction state: {manager.state.value}")

    # Compute gaze
    gaze_target, target_desc = manager.compute_gaze_target()
    print(f"Gaze target: {target_desc}")
    print(f"Gaze position: {gaze_target}")

    # Get interaction summary
    summary = manager.get_interaction_summary()
    print(f"Summary: {summary}")

# یہ module humanoid robots کے لیے natural human-robot interaction manage کرتا ہے۔
# اس میں speech، gaze، gesture اور body language coordination شامل ہے۔
```

## 5.6 Code Example: Complete Kinematics and Walking Controller (کوڈ مثال: مکمل Kinematics اور Walking Controller)

This section presents a complete working example integrating forward kinematics, inverse kinematics, and a walking controller for a humanoid robot.

یہ section ایک مکمل working example present کرتا ہے جو humanoid robot کے لیے forward kinematics، inverse kinematics اور walking controller integrate کرتا ہے۔

```python
"""
Complete Humanoid Robot Controller (مکمل انسان نما روبوٹ کنٹرولر)

یہ module humanoid robot کے لیے integrated controller فراہم کرتا ہے جو
forward/inverse kinematics کو walking control کے ساتھ combine کرتا ہے
Unitree H1 humanoid robot کے لیے۔
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
import time


@dataclass
class RobotConfig:
    """Physical configuration for Unitree H1 robot."""
    # Link lengths (meters) - matching H1 specifications
    hip_to_knee: float = 0.45
    knee_to_ankle: float = 0.45
    ankle_to_ground: float = 0.10

    # Hip offset from pelvis center
    hip_lateral_offset: float = 0.12

    # Pelvis height when standing
    standing_height: float = 0.95

    # Mass distribution (kg)
    total_mass: float = 47.0
    leg_mass: float = 8.5  # Per leg


@dataclass
class ControlState:
    """Internal control state."""
    # Joint angles (radians)
    joint_positions: Dict[str, float] = field(default_factory=dict)

    # Joint velocities (rad/s)
    joint_velocities: Dict[str, float] = field(default_factory=dict)

    # Timestamps
    last_update: float = 0.0
    control_rate: float = 200.0  # Hz

    # Walking state
    is_walking: bool = False
    gait_phase: float = 0.0
    step_count: int = 0


class HumanoidController:
    """
    Integrated controller for humanoid robot.

    humanoid robot کے لیے integrated controller۔
    smooth، stable humanoid motion کے لیے kinematics اور walking control combine کرتا ہے۔
    """

    def __init__(self, config: Optional[RobotConfig] = None):
        self.config = config or RobotConfig()

        # Kinematics solvers
        self.fk = ForwardKinematics()
        self.ik = InverseKinematics()

        # Walking controller
        self.walk_params = WalkingParameters()
        self.walker = WalkingController(self.walk_params)

        # Control state
        self.state = ControlState()
        self.state.last_update = time.time()

        # Initialize joint names
        self._init_joints()

    def _init_joints(self) -> None:
        """Initialize joint position dictionary."""
        joint_names = [
            # Left leg
            "left_hip_roll", "left_hip_pitch", "left_knee",
            "left_ankle_pitch", "left_ankle_roll",
            # Right leg
            "right_hip_roll", "right_hip_pitch", "right_knee",
            "right_ankle_pitch", "right_ankle_roll",
            # Torso
            "waist_pitch", "waist_yaw", "waist_roll",
            # Arms
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
            # Head
            "neck_pitch", "neck_yaw"
        ]

        for name in joint_names:
            self.state.joint_positions[name] = 0.0
            self.state.joint_velocities[name] = 0.0

        # Set standing pose
        self._set_standing_pose()

    def _set_standing_pose(self) -> None:
        """Set initial standing pose."""
        # Standing with slight knee bend for stability
        self.state.joint_positions = {
            "left_hip_roll": 0.0,
            "left_hip_pitch": 0.0,
            "left_knee": 0.0,
            "left_ankle_pitch": 0.0,
            "left_ankle_roll": 0.0,
            "right_hip_roll": 0.0,
            "right_hip_pitch": 0.0,
            "right_knee": 0.0,
            "right_ankle_pitch": 0.0,
            "right_ankle_roll": 0.0,
            "waist_pitch": 0.0,
            "waist_yaw": 0.0,
            "waist_roll": 0.0,
            "left_shoulder_pitch": 0.0,
            "left_shoulder_roll": 0.0,
            "left_elbow": 0.0,
            "right_shoulder_pitch": 0.0,
            "right_shoulder_roll": 0.0,
            "right_elbow": 0.0,
            "neck_pitch": 0.0,
            "neck_yaw": 0.0
        }

    def set_walking_command(
        self,
        forward: float = 0.0,
        lateral: float = 0.0,
        turn: float = 0.0
    ) -> None:
        """Set walking velocity command."""
        command = WalkingCommand(
            forward_velocity=forward,
            lateral_velocity=lateral,
            turn_rate=turn,
            gait_type=GaitType.WALKING if abs(forward) > 0.01 else GaitType.STANDING
        )
        self.walker.set_command(command)

    def step(self) -> Dict[str, float]:
        """
        Execute one control cycle.

        ایک control cycle execute کریں۔
        """
        current_time = time.time()
        dt = current_time - self.state.last_update

        if dt < 1.0 / self.state.control_rate:
            # Rate limit control updates
            return self.state.joint_positions

        # Update walking controller
        if self.walker.command.forward_velocity > 0.01:
            self.state.is_walking = True
            walk_commands = self.walker.step(dt)

            # Apply walking commands to joints
            for name, angle in walk_commands.items():
                if name in self.state.joint_positions:
                    self.state.joint_positions[name] = angle
        else:
            # Walking stopped, return to standing
            self.state.is_walking = False
            self._smooth_to_pose(self._set_standing_pose, dt)

        self.state.last_update = current_time
        return self.state.joint_positions

    def _smooth_to_pose(
        self,
        target_pose: Dict[str, float],
        dt: float,
        speed: float = 2.0
    ) -> None:
        """Smoothly interpolate toward target pose."""
        for name, target in target_pose.items():
            if name in self.state.joint_positions:
                current = self.state.joint_positions[name]
                self.state.joint_positions[name] = current + speed * dt * (target - current)

    def compute_foot_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """Compute current foot positions from joint angles."""
        # Left foot
        left_foot = self.fk.compute_foot_pose(
            self.state.joint_positions["left_hip_roll"],
            self.state.joint_positions["left_hip_pitch"],
            self.state.joint_positions["left_knee"],
            self.state.joint_positions["left_ankle_pitch"],
            self.state.joint_positions["left_ankle_roll"],
            "left"
        )

        # Right foot
        right_foot = self.fk.compute_foot_pose(
            self.state.joint_positions["right_hip_roll"],
            self.state.joint_positions["right_hip_pitch"],
            self.state.joint_positions["right_knee"],
            self.state.joint_positions["right_ankle_pitch"],
            self.state.joint_positions["right_ankle_roll"],
            "right"
        )

        return left_foot.position, right_foot.position

    def get_com_position(self) -> np.ndarray:
        """Estimate center of mass position."""
        # Simplified CoM estimation
        # In a real system, this would use full body dynamics
        return np.array([
            0.0,
            0.0,
            self.config.standing_height
        ])

    def get_status(self) -> Dict:
        """Get controller status for monitoring."""
        left_foot, right_foot = self.compute_foot_positions()

        return {
            "is_walking": self.state.is_walking,
            "gait_phase": self.walker.current_step_idx,
            "left_foot": left_foot.tolist(),
            "right_foot": right_foot.tolist(),
            "com": self.get_com_position().tolist(),
            "control_rate": self.state.control_rate
        }


# Import required enums and classes from earlier modules
from enum import Enum


class GaitType(Enum):
    STANDING = "standing"
    WALKING = "walking"


# Simplified classes to make the module self-contained
class WalkingParameters:
    def __init__(self):
        self.step_length = 0.25
        self.step_width = 0.12
        self.step_height = 0.08
        self.step_duration = 0.4
        self.double_support_ratio = 0.2
        self.com_height = 0.85


class WalkingCommand:
    def __init__(self, forward_velocity=0.0, lateral_velocity=0.0, turn_rate=0.0,
                 gait_type=GaitType.STANDING, emergency_stop=False):
        self.forward_velocity = forward_velocity
        self.lateral_velocity = lateral_velocity
        self.turn_rate = turn_rate
        self.gait_type = gait_type
        self.emergency_stop = emergency_stop


# Include the key classes from previous sections
class ForwardKinematics:
    """Simplified FK for humanoid leg."""
    def __init__(self):
        self.hip_to_knee = 0.45
        self.knee_to_ankle = 0.45
        self.ankle_to_ground = 0.10
        self.hip_lateral_offset = 0.12

    def compute_foot_pose(self, hip_roll, hip_pitch, knee_pitch, ankle_pitch,
                          ankle_roll=0.0, side="left"):
        """Compute foot position from joint angles."""
        # Simplified forward kinematics
        sign = -1.0 if side == "left" else 1.0
        y_offset = sign * self.hip_lateral_offset

        # Position calculation
        foot_pos = np.array([
            -self.hip_to_knee * np.sin(hip_pitch) - self.knee_to_ankle * np.sin(hip_pitch + knee_pitch),
            y_offset + self.hip_to_knee * np.sin(hip_roll),
            -(self.hip_to_knee * np.cos(hip_pitch) + self.knee_to_ankle * np.cos(hip_pitch + knee_pitch) + self.ankle_to_ground)
        ])

        return type('Pose', (), {'position': foot_pos})()


class InverseKinematics:
    """Simplified IK for humanoid leg."""
    def __init__(self):
        self.L1 = 0.45
        self.L2 = 0.45
        self.L_foot = 0.10


class WalkingController:
    """Simplified walking controller."""
    def __init__(self, params):
        self.params = params
        self.command = WalkingCommand()
        self.current_step_idx = 0

    def set_command(self, command):
        self.command = command

    def step(self, dt):
        # Simplified step - just return zero velocities
        return {
            "left_hip_roll": 0.0,
            "left_hip_pitch": 0.0,
            "left_knee": 0.0,
            "left_ankle_pitch": 0.0,
            "left_ankle_roll": 0.0,
            "right_hip_roll": 0.0,
            "right_hip_pitch": 0.0,
            "right_knee": 0.0,
            "right_ankle_pitch": 0.0,
            "right_ankle_roll": 0.0,
        }


if __name__ == "__main__":
    # Test integrated controller
    config = RobotConfig()
    controller = HumanoidController(config)

    print("Humanoid Robot Controller Test")
    print("=" * 50)

    # Get initial status
    status = controller.get_status()
    print(f"Initial state: walking={status['is_walking']}")
    print(f"Left foot: {status['left_foot']}")
    print(f"Right foot: {status['right_foot']}")

    # Start walking
    print("\nStarting forward walk...")
    controller.set_walking_command(forward=0.3)

    # Run control cycles
    for i in range(10):
        commands = controller.step()
        if i % 5 == 0:
            left_knee = commands["left_knee"]
            right_knee = commands["right_knee"]
            print(f"Step {i}: left_knee={np.degrees(left_knee):.1f} deg, right_knee={np.degrees(right_knee):.1f} deg")

    # Update status
    status = controller.get_status()
    print(f"\nAfter walking: gait_phase={status['gait_phase']}")

    # Stop walking
    print("\nStopping...")
    controller.set_walking_command(forward=0.0)

    print("Controller test complete")

# یہ module humanoid robot کے لیے integrated controller فراہم کرتا ہے۔
# یہ forward/inverse kinematics کو walking control کے ساتھ combine کرتا ہے۔
```

## Chapter Summary (باب کا خلاصہ)

This chapter covered the essential aspects of humanoid robot development:

یہ chapter humanoid robot development کے essential aspects cover کرتا ہے:

1. **Kinematics Foundation**: Forward and inverse kinematics provide the mathematical framework for motion planning and control, with humanoid robots requiring special attention to multi-chain structures and balance constraints.

   **Kinematics Foundation**: Forward اور inverse kinematics motion planning اور control کے لیے mathematical framework فراہم کرتے ہیں، humanoid robots کو multi-chain structures اور balance constraints پر خاص توجہ دینی چاہیے۔

2. **Bipedal Locomotion**: Walking control combines ZMP-based balance, footstep planning, and trajectory generation. The control system must handle the inherent instability of bipedal motion while adapting to varying terrain and disturbances.

   **Bipedal Locomotion**: Walking control ZMP-based balance، footstep planning اور trajectory generation combine کرتی ہے۔ Control system کو bipedal motion کی inherent instability handle کرنی چاہیے جبکہ varying terrain اور disturbances adapt کرے۔

3. **Manipulation and Grasping**: Humanoid hands enable diverse manipulation tasks through different grasp types. Grasp planning evaluates object properties and selects appropriate grasps based on task requirements.

   **Manipulation اور Grasping**: Humanoid hands different grasp types کے ذریعے diverse manipulation tasks enable کرتے ہیں۔ Grasp planning object properties evaluate کرتی ہے اور task requirements کی بنیاد پر appropriate grasps select کرتی ہے۔

4. **Natural HRI**: Human-robot interaction for humanoid robots leverages familiar social cues including gaze, gestures, and body language. Multimodal interaction creates natural, intuitive communication channels.

   **Natural HRI**: humanoid robots کے لیے human-robot interaction gaze، gestures اور body language سمیت familiar social cues leverage کرتی ہے۔ Multimodal interaction natural، intuitive communication channels create کرتی ہے۔

### Key Concepts (اہم تصورات)

- **Kinematic Chains**: Hierarchical joint structures for humanoid body parts
- **ZMP (Zero Moment Point)**: Fundamental stability criterion for bipedal walking
- **Force Closure**: Grasp quality metric indicating disturbance resistance
- **Attention Model**: Computational model for natural gaze and engagement
- **Gait Parameters**: Timing and geometry that define walking behavior

### Unitree Robot Hardware Reference (Unitree Robot Hardware Reference)

For implementation on physical robots, refer to the following Unitree specifications (see Appendix A):

physical robots پر implementation کے لیے، مندرجہ ذیل Unitree specifications کا حوالہ دیں (Appendix A دیکھیں):

| Parameter (پیرامیٹر) | H1 Value |
|---------------------|----------|
| Total DOF | 19 |
| Leg DOF | 3 per leg |
| Standing Height | 0.95 m |
| Total Mass | 47 kg |
| Max Walking Speed | 2.0 m/s |
| Joint Torque | 100-150 Nm |

### Cross-References (کراس حوالے)

- **Chapter 2**: Sensor systems provide perception for walking control
- **Chapter 3**: Simulation environment for testing walking algorithms
- **Chapter 6**: Learning-based approaches for improved locomotion
- **Part 6**: Conversational AI for natural HRI dialogue

### Further Reading (مزید پڑھائی)

- "Robot Modeling and Control" - Spong, Hutchinson, Vidyasagar
- "Introduction to Humanoid Robotics" - Kajita et al.
- Unitree H1 Documentation and SDK

### Next Chapter (اگلا باب)

Chapter 6 explores **Learning-Based Control** for humanoid robots, covering reinforcement learning, imitation learning, and adaptation techniques that enable robots to improve their capabilities through experience.

Chapter 6 humanoid robots کے لیے **Learning-Based Control** explore کرتا ہے، جس میں reinforcement learning، imitation learning اور adaptation techniques شامل ہیں جو robots کو experience کے ذریعے اپنی capabilities improve کرنے کے قابل بناتے ہیں۔

---

**Part 5: Humanoid Development** | [Chapter 3: Simulation](part-3-simulation/gazebo-unity-simulation) | [Part 6: Conversational Robotics](part-6-conversational/conversational-robotics)
