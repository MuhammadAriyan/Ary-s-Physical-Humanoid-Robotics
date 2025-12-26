---
title: "ہفتے 8-10: NVIDIA Isaac پلیٹ فارم"
sidebar_position: 5
---

# حصہ 4: NVIDIA Isaac پلیٹ فارم - ہفتے 8-10 کا جائزہ

یہ حصہ NVIDIA Isaac ایکوسسٹم میں مہارت حاصل کرنے کے لیے ایک منظم سیکھنے کا منصوبہ فراہم کرتا ہے۔ تین ہفتوں کا نصاب اعلی معیار کی تقلید، AI پر مبنی ادراک، GPU تیز رفتار تقویتی سیکھنے، اور حقیقت کی منتقلی کی تکنیکوں میں عملی مہارتیں بناتا ہے۔

## ہفتہ 8: Isaac Sim کی تنصیب اور AI پر مبنی ادراک

### سیکھنے کے مقاصد

ہفتہ 8 کے اختتام تک، آپ درج ذیل کرنے میں قادر ہوں گے:
- RTX پاورڈ ورک اسٹیشنز اور سرورز پر Isaac Sim کی تنصیب اور کانفگریشن کریں
- USD (Universal Scene Description) کا استعمال کرتے ہوئے ہیومینائڈ روبوٹس کے ساتھ تقلیدی ماحول بنائیں
- اشیاء کی detection اور segmentation کے لیے GPU تیز رفتار کمپیوٹر ویژن پائپ لائنز لاگو کریں
- کیمروں، ڈیپتھ سینسرز، اور IMUs سمیت حقیقی سینسر simulation کانفیگر کریں
- ادراک ماڈلز کی تربیت کے لیے مصنوعی ڈیٹا سیٹس تیار کریں
- ROS 2 کے ساتھ Isaac ادراک آؤٹ پٹس کو اینٹیگریٹ کریں

### اہم موضوعات

#### 1. Isaac Sim کی تنصیب اور کانفگریشن

Isaac Sim کو صحیح طریقے سے کام کرنے کے لیے مخصوص ہارڈویئر اور سافٹ ویئر کی ضروریات ہوتی ہیں۔ تنصیب کے عمل میں NVIDIA Omniverse سیٹ اپ کرنا، Isaac Sim ایکسٹینشن انسٹال کرنا، اور ڈیولپمنٹ ماحول کانفیگر کرنا شامل ہے۔ ان ضروریات کو سمجھنا عام سیٹ اپ کی خرابیوں کو روکتا ہے اور بہترین کارکردگی کو یقینی بناتا ہے۔

تنصیب کا عمل NVIDIA Omniverse Launcher کی تنصیب سے شروع ہوتا ہے۔ Launcher سیٹ اپ کے بعد، Isaac Sim کو ایک kit extension کے طور پر انسٹال کیا جا سکتا ہے۔ تنصیب کا سائز بڑا ہے (10+ GB) کیونکہ یہ جامع simulation assets اور dependencies پر مشتمل ہے۔ تنصیب کے بعد کی کانفگریشن میں Python environments سیٹ اپ کرنا، GPU rendering options کانفیگر کرنا، اور USD asset paths قائم کرنا شامل ہے۔

```bash
# Isaac Sim کی تنصیب کے اقدامات (Ubuntu 22.04)

# 1. NVIDIA ڈرائیور انسٹال کریں (اگر پہلے سے انسٹال نہیں ہے)
sudo apt update
sudo apt install nvidia-driver-535
sudo reboot

# 2. NVIDIA ویب سائٹ سے Omniverse Launcher ڈاؤن لوڈ کریں
# https://www.nvidia.com/en-us/omniverse/download/
chmod +x OmniverseLauncher-linux-x86_64-*.AppImage
./OmniverseLauncher-linux-x86_64-*.AppImage

# 3. Omniverse Launcher کے ذریعے Isaac Sim انسٹال کریں
# - Omniverse Launcher کھولیں
# - "Exchange" ٹیب پر جائیں
# - "Isaac Sim" تلاش کریں
# - Install پر کلک کریں

# 4. Python virtual environment بنائیں (تجویز کردہ)
python -m venv isaac-sim-env
source isaac-sim-env/bin/activate

# 5. ماحولیاتی متغیرات سیٹ اپ کریں
export OMNI_KIT_APP_NAME=isaac-sim
export ISAAC_SIM_PATH=$HOME/.local/share/ov/pkg/isaac_sim-*
export CARB_APP_PATH=$ISAAC_SIM_PATH/apps
export PYTHON_PATH=$ISAAC_SIM_PATH/python:$PYTHON_PATH

# 6. تنصیب کی تصدیق کریں
cd $ISAAC_SIM_PATH
./python.sh scripts/examples/simple_env.py
```

یہ کوڈ Isaac Sim کی تنصیب کے مکمل مراحل کو دکھاتا ہے۔ پہلے NVIDIA ڈرائیور انسٹال کیا جاتا ہے، پھر Omniverse Launcher ڈاؤن لوڈ اور چلایا جاتا ہے۔ اس کے بعد Isaac Sim انسٹال کیا جاتا ہے اور Python virtual environment بنائی جاتی ہے۔ آخر میں ضروری ماحولیاتی متغیرات سیٹ کر کے تنصیب کی تصدیق کی جاتی ہے۔

```bash
# Isaac Sim کی تنصیب کے مکمل مراحل کو یہاں دکھایا گیا ہے
# ڈرائیور سے لے کر تصدیق تک سب کچھ شامل ہے
```

#### 2. USD پر مبنی روبوٹ کی تفصیل

Universal Scene Description (USD) Isaac Sim میں asset management کی بنیاد ہے۔ روایتی ROS روبوٹکس کے لیے URDF کے برعکس، USD ایک ہیئررکل scene graph فراہم کرتا ہے جس میں دولی composition capabilities ہیں۔ USD composition، variants، اور payloads کو سمجھنا robot اور environment کی نمائندگی کو فعال بناتا ہے۔

USD میں ہیومینائڈ روبوٹس کو احتیاط سے articulation setup کی ضرورت ہوتی ہے۔ Articulation root روبوٹ کے links اور joints کے درمی kinematic اور dynamic تعلقات کی وضاحت کرتا ہے۔ Joint types جن میں revolute، prismatic، اور spherical شامل ہیں، کو مناسب limits اور dynamics parameters کے ساتھ کانفیگر کرنا ہوتا ہے۔ Visual اور collision geometry الگ USD فائلوں سے reference کیا جا سکتا ہے، جو projects میں asset reuse کو فعال کرتا ہے۔

```python
# مثال: USD میں ہیومینائڈ روبوٹ articulation بنانا
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

def create_humanoid_articulation(stage, prim_path, config):
    """
    USD میں ہیومینائڈ روبوٹ articulation بنائیں۔

    Args:
        stage: USD stage جہاں articulation بنانا ہے
        prim_path: روبوٹ کا بنیادی پاتھ
        config: روبوٹ کانفگریشن ڈکشنری
    """
    # Robot root کے لیے Xform بنائیں
    robot_xform = UsdGeom.Xform.Define(stage, prim_path)

    # Articulation root کی تعریف کریں
    articulation = UsdPhysics.ArticulationRootAPI.Apply(
        stage.GetPrimAtPath(f"{prim_path}/root_link")
    )
    articulation.CreateEnabledAttr(True)

    # ہر link کے لیے body schemas بنائیں
    link_names = [
        "pelvis", "torso", "head",
        "left_shoulder", "left_upper_arm", "left_lower_arm", "left_hand",
        "right_shoulder", "right_upper_arm", "right_lower_arm", "right_hand",
        "left_hip", "left_upper_leg", "left_lower_leg", "left_foot",
        "right_hip", "right_upper_leg", "right_lower_leg", "right_foot"
    ]

    for link_name in link_names:
        link_path = f"{prim_path}/{link_name}"
        link = UsdGeom.Xform.Define(stage, link_path)

        # Rigid body اور collision شامل کریں
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(link_path))
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(link_path))

    # ملحقہ links کے درمیان joints بنائیں
    joint_connections = [
        ("pelvis", "torso", "revolute"),
        ("torso", "head", "revolute"),
        ("torso", "left_shoulder", "revolute"),
        ("left_shoulder", "left_upper_arm", "revolute"),
        ("left_upper_arm", "left_lower_arm", "revolute"),
        ("left_lower_arm", "left_hand", "revolute"),
        ("torso", "right_shoulder", "revolute"),
        ("right_shoulder", "right_upper_arm", "revolute"),
        ("right_upper_arm", "right_lower_arm", "revolute"),
        ("right_lower_arm", "right_hand", "revolute"),
        ("pelvis", "left_hip", "revolute"),
        ("left_hip", "left_upper_leg", "revolute"),
        ("left_upper_leg", "left_lower_leg", "revolute"),
        ("left_lower_leg", "left_foot", "revolute"),
        ("pelvis", "right_hip", "revolute"),
        ("right_hip", "right_upper_leg", "revolute"),
        ("right_upper_leg", "right_lower_leg", "revolute"),
        ("right_lower_leg", "right_foot", "revolute"),
    ]

    for parent, child, joint_type in joint_connections:
        create_joint(
            stage,
            f"{prim_path}/{parent}",
            f"{prim_path}/{child}",
            joint_type
        )

    return robot_xform


def create_joint(stage, parent_path, child_path, joint_type):
    """دو links کے درمیان joint بنائیں۔"""
    joint_path = f"{child_path}/joint"

    if joint_type == "revolute":
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    elif joint_type == "prismatic":
        joint = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
    else:
        joint = UsdPhysics.Joint.Define(stage, joint_path)

    # Body references سیٹ کریں
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])

    # Joint limits کانفیگر کریں
    if joint_type == "revolute":
        joint.CreateLowerLimitAttr(-1.57)  # -90 degrees
        joint.CreateUpperLimitAttr(1.57)   # 90 degrees
        joint.CreateAxisAttr("Z")

    return joint
```

یہ کوڈ USD میں ایک مکمل ہیومینائڈ روبوٹ articulation بناتا ہے۔ فنکشن `create_humanoid_articulation` روبوٹ کی تمام links اور joints کی تخلیق کرتا ہے، جبکہ `create_joint` فنکشن ملحقہ links کے درمیان joints بناتا ہے۔ یہ ہر joint کے لیے مناسب limits اور axis سیٹ کرتا ہے۔

```python
# USD میں ہیومینائڈ روبوٹ کی مکمل ساخت
# Links، joints، اور articulation سب شامل ہیں
```

#### 3. GPU تیز رفتار ادراک پائپ لائنز

جدید ادراک اشیاء کی detection، segmentation، اور depth estimation کے لیے گہرے neural networks پر انحصار کرتی ہے۔ Isaac GPU acceleration کے لیے optimized implementations فراہم کرتا ہے جو real-time کارکردگی کو ممکن بناتی ہیں۔ ان پائپ لائنز اور simulation کے ساتھ ان کی اینٹیگریشن کو سمجھنا ادراک کے قابل روبوٹ سسٹمز کی ترقی کو فعال بناتا ہے۔

Object detection کیمرا تصاویر میں اشیاء کی identification اور locating کرتی ہے۔ مقبول architectures میں real-time detection کے لیے YOLO variants اور higher accuracy کے لیے Faster R-CNN شامل ہیں۔ Isaac integration TensorRT کا استعمال کرتی ہے optimized inference کے لیے، جو RTX GPUs پر 100+ FPS حاصل کرتی ہے۔ Detection outputs میں bounding boxes، class labels، اور confidence scores شامل ہیں۔

Semantic segmentation تصویر کے ہر pixel کو semantic categories میں classify کرتی ہے۔ ہیومینائڈ روبوٹس کے لیے، floors، walls، furniture، اور objects کو segment کرنا navigation اور manipulation کے لیے scene understanding کو فعال کرتا ہے۔ Segmentation networks full-resolution تصاویر process کرتی ہیں، جو real-time آپریشن کے لیے اہم compute کی ضرورت ہوتی ہے۔

Depth estimation monocular images سے 3D information فراہم کرتی ہے بغیر specialized depth sensors کے۔ RGB-D datasets پر train کردہ neural networks single images سے dense depth maps predict کر سکتی ہیں۔ یہ predictions depth sensors سے محروم platforms پر 3D perception کو فعال کرتی ہیں، اگرچہ براہ راست depth measurement کے مقارنة میں کم accuracy کے ساتھ۔

### اہم تصورات میں مہارت حاصل کرنا

- **Isaac Sim Architecture**: Client-server model، USD composition، physics timestep management
- **GPU Pipeline**: TensorRT optimization، CUDA memory management، inference batching
- **Sensor Simulation**: Camera models، noise injection، depth sensor characteristics
- **ROS 2 Integration**: Isaac-ROS bridges، topic remapping، message type conversions
- **Synthetic Data Generation**: Domain randomization، augmentation strategies، dataset scaling

### مشق کی تماریں

1. **مشق 1: Isaac Sim کی تنصیب اور تصدیق** (3 گھنٹے)
   - Omniverse Launcher اور Isaac Sim extension انسٹال کریں
   - مطلوبہ packages کے ساتھ Python environment کانفیگر کریں
   - تنصیب کی تصدیق کے لیے built-in examples چلائیں
   - مختلف scene complexities کے لیے GPU memory usage benchmark کریں

2. **مشق 2: ہیومینائڈ روبوٹ USD تخلیق** (4 گھنٹے)
   - USD format میں ایک آسان ہیومینائڈ روبوٹ بنائیں
   - مناسب joints کے ساتھ articulation structure کی تعریف کریں
   - Collision geometry اور mass properties کانفیگر کریں
   - Isaac Sim میں load کریں اور visualize کریں

3. **مشق 3: کیمرا simulation اور capture** (3 گھنٹے)
   - ہیومینائڈ روبوٹ میں کیمرا sensors شامل کریں
   - کیمرا intrinsics کانفیگر کریں (resolution، FOV، focal length)
   - simulation سے RGB اور depth images capture کریں
   - بیرونی processing کے لیے captured data export کریں

4. **مشق 4: Object Detection Pipeline** (4 گھنٹے)
   - Detection کے لیے TensorRT model loading لاگو کریں
   - کیمرا images کو detection network سے process کریں
   - bounding boxes کے ساتھ detection results visualize کریں
   - topic publishing کے لیے ROS 2 کے ساتھ integrate کریں

5. **مشق 5: Synthetic Dataset Generation** (4 گھنٹے)
   - object appearance کے لیے domain randomization سیٹ اپ کریں
   - مختلف conditions کے ساتھ 10,000 labeled images تیار کریں
   - downstream training کے لیے COCO format میں export کریں
   - dataset quality metrics کی تصدیق کریں

### تخمینہ وقت کا عہدہ

| سرگرمی | گھنٹے |
|--------|-------|
| پڑھنا اور tutorials | 8 |
| مشق 1: تنصیب | 3 |
| مشق 2: USD تخلیق | 4 |
| مشق 3: کیمرا سیٹ اپ | 3 |
| مشق 4: Detection | 4 |
| مشق 5: ڈیٹا سیٹ جنریشن | 4 |
| troubleshooting اور جائزہ | 4 |
| **کل** | **30 گھنٹے** |

## ہفتہ 9: Isaac Gym اور Manipulation

### سیکھنے کے مقاصد

ہفتہ 9 کے اختتام تک، آپ درج ذیل کرنے میں قادر ہوں گے:
- GPU تیز رفتار parallel simulation کے لیے Isaac Gym کانفیگر کریں
- manipulation tasks کے لیے reward functions ڈیزائن کریں
- PPO یا SAC algorithms کے ساتھ RL training loops لاگو کریں
- grasping، placement، اور tool use کے لیے policies train کریں
- simulation metrics کا استعمال کرتے ہوئے policy performance کا جائزہ لیں
- deployment کے لیے trained policies export کریں

### اہم موضوعات

#### 1. Isaac Gym کی بنیادی باتیں

Isaac Gym robot learning میں ایک انقلاب ہے کیونکہ یہ ایک GPU پر ہزاروں simulations parallel چلاتا ہے۔ روایتی RL approaches CPU-based simulation استعمال کرتے ہیں جو single-core performance سے محدود ہیں۔ Isaac Gym کا GPU-first architecture billions of simulated interactions سے سیکھنے کو sample-efficient بناتا ہے۔

Architecture Python training code کو physics simulation سے الگ کرتا ہے۔ Physics GPU پر PhysX کے ذریعے entirely چلتا ہے، costly CPU-GPU data transfers سے بچتے ہوئے۔ Training code simulation state کو GPU tensors کے ذریعے access کرتا ہے، جو تمام environments پر vectorized operations کو فعال کرتا ہے۔ یہ ڈیزائن GPU capability کے ساتھ directly scales کرتا ہے، RTX workstations سے لے کر A100 data center GPUs تک۔

Environment design in Isaac Gym ایک pattern کی پیروی کرتا ہے: environment creation، reset handling، اور observation collection۔ ہر environment ایک robot اور task-relevant objects رکھتا ہے۔ Training loop سب environments کو parallel میں step کرتا ہے، experiences collect کرتا ہے، اور policy update کرتا ہے۔ یہ parallel structure learning کو serial simulation کے مقابلے میں orders of magnitude تیز کرتی ہے۔

```python
# مثال: Object grasping کے لیے Isaac Gym environment
import numpy as np
import torch
import isaacgym
from isaacgym import gymapi, gymtorch

class GraspingEnv:
    """
    Isaac Gym کے لیے parallel grasping environment۔
    Multiple robot hands parallel میں objects grab کرنا سیکھتے ہیں۔
    """

    def __init__(
        self,
        num_envs: int = 4096,
        num_obs: int = 45,
        num_actions: int = 7,
        device: str = "cuda"
    ):
        self.num_envs = num_envs
        self.num_obs = num_obs
        self.num_actions = num_actions
        self.device = device

        # Gym initialize کریں
        self.gym = gymapi.acquire_gym()

        # Simulation parameters
        self.sim_params = gymapi.SimParams()
        self.sim_params.dt = 1.0 / 60.0
        self.sim_params.num_substeps = 2
        self.sim_params.use_gpu_pipeline = True

        # Simulation بنائیں
        self.sim = self.gym.create_sim(
            device_id=0 if device == "cuda" else -1,
            graphics_device_id=0 if device == "cuda" else -1,
            sim_type=gymapi.SIM_PHYSX,
            sim_params=self.sim_params
        )

        # Environments بنائیں
        self._create_assets()
        self._create_envs()

        # State tensors initialize کریں
        self._init_state_tensors()

    def _create_assets(self):
        """Robot hand اور object assets load کریں۔"""
        # Robot hand asset
        hand_asset_options = gymapi.AssetOptions()
        hand_asset_options.fix_base_link = 0
        hand_asset_options.disable_gravity = False
        self.hand_asset = self.gym.load_asset(
            self.sim, "assets", "robot_hand.urdf", hand_asset_options
        )

        # Object asset (cube)
        object_asset_options = gymapi.AssetOptions()
        object_asset_options.density = 1000.0
        self.object_asset = self.gym.load_asset(
            self.sim, "assets", "cube.usd", object_asset_options
        )

    def _create_envs(self):
        """Parallel environments بنائیں۔"""
        env_spacing = 0.6
        num_per_row = int(np.sqrt(self.num_envs))

        self.envs = []
        self.hand_actors = []
        self.object_actors = []

        for i in range(self.num_envs):
            env = self.gym.create_env(
                self.sim,
                env_spacing * (i % num_per_row),
                env_spacing * (i // num_per_row),
                env_spacing,
                num_per_row
            )

            # Ground plane شامل کریں
            plane_params = gymapi.PlaneParams()
            plane_params.normal = gymapi.Vec3(0, 0, 1)
            self.gym.add_ground(self.sim, plane_params)

            # Hand spawn کریں
            hand_pose = gymapi.Transform()
            hand_pose.p = gymapi.Vec3(0, 0, 0.4)
            hand_actor = self.gym.create_actor(
                env, self.hand_asset, hand_pose, f"hand_{i}", i, 1
            )
            self.hand_actors.append(hand_actor)

            # Object spawn کریں
            object_pose = gymapi.Transform()
            object_pose.p = gymapi.Vec3(
                (np.random.rand() - 0.5) * 0.15,
                (np.random.rand() - 0.5) * 0.15,
                0.05
            )
            object_actor = self.gym.create_actor(
                env, self.object_asset, object_pose, f"object_{i}", i, 2
            )
            self.object_actors.append(object_actor)

            self.envs.append(env)

    def _init_state_tensors(self):
        """GPU state access کے لیے PyTorch tensors initialize کریں۔"""
        # Actor count حاصل کریں
        num_actors = self.gym.get_actor_count(self.sim)

        # State access کے لیے tensors بنائیں
        self.root_tensor = torch.zeros(
            (num_actors, 13),
            dtype=torch.float32,
            device=self.device
        )

        self.dof_tensor = torch.zeros(
            (num_actors, 7),  # 7-DOF hand
            dtype=torch.float32,
            device=self.device
        )

        # Policy tensors
        self.obs_tensor = torch.zeros(
            (self.num_envs, self.num_obs),
            dtype=torch.float32,
            device=self.device
        )

        self.action_tensor = torch.zeros(
            (self.num_envs, self.num_actions),
            dtype=torch.float32,
            device=self.device
        )

        self.rew_tensor = torch.zeros(
            (self.num_envs,),
            dtype=torch.float32,
            device=self.device
        )

        self.done_tensor = torch.zeros(
            (self.num_envs,),
            dtype=torch.bool,
            device=self.device
        )

    def reset(self):
        """سب environments کو initial state میں reset کریں۔"""
        # Object positions کو randomize کریں
        for i, (env, object_actor) in enumerate(zip(self.envs, self.object_actors)):
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(
                (np.random.rand() - 0.5) * 0.15,
                (np.random.rand() - 0.5) * 0.15,
                0.05
            )
            self.gym.set_rigid_body_pose(
                env, object_actor, 0, pose
            )

        # Hand positions reset کریں
        for i, (env, hand_actor) in enumerate(zip(self.envs, self.hand_actors)):
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 0.4)
            self.gym.set_rigid_body_pose(
                env, hand_actor, 0, pose
            )

        return self._get_obs()

    def step(self, actions: torch.Tensor):
        """دیے گئے actions کے ساتھ سب environments step کریں۔"""
        # Actions apply کریں
        self._apply_actions(actions)

        # Physics step کریں
        for _ in range(4):  # Sub-stepping
            self.gym.simulate(self.sim)

        self.gym.fetch_results(self.sim, True)

        # Rewards اور dones compute کریں
        self._compute_rewards()
        self._compute_dones()

        return self._get_obs(), self.rew_tensor, self.done_tensor, {}

    def _apply_actions(self, actions: torch.Tensor):
        """Joint position targets apply کریں۔"""
        for i, (env, hand_actor) in enumerate(zip(self.envs, self.hand_actors)):
            joint_targets = actions[i].cpu().numpy()
            self.gym.set_actor_dof_position_targets(
                env, hand_actor, joint_targets
            )

    def _get_obs(self):
        """سب environments سے observations collect کریں۔"""
        # Simulation سے root tensor refresh کریں
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)

        # Tensors سے observations extract کریں
        # Hand position (3)، orientation (4)، velocity (6)، object position (3)
        # Plus relative position اور distance

        for i in range(self.num_envs):
            hand_idx = i * 2  # Hand ہر env میں پہلا actor ہے
            object_idx = i * 2 + 1  # Object ہر env میں دوسرا actor ہے

            hand_pos = self.root_tensor[hand_idx, :3]
            hand_quat = self.root_tensor[hand_idx, 3:7]
            hand_vel = self.root_tensor[hand_idx, 7:13]
            object_pos = self.root_tensor[object_idx, :3]

            # Relative position compute کریں
            rel_pos = object_pos - hand_pos
            distance = torch.norm(rel_pos)

            # Observation vector build کریں
            obs = torch.cat([
                hand_pos, hand_quat, hand_vel,
                object_pos, rel_pos, distance.unsqueeze(0)
            ])

            self.obs_tensor[i] = obs

        return self.obs_tensor

    def _compute_rewards(self):
        """ہر environment کے لیے reward compute کریں۔"""
        # Hand اور object positions حاصل کریں
        hand_positions = self.root_tensor[0::2, :3]
        object_positions = self.root_tensor[1::2, :3]

        # Distance reward (negative distance)
        distances = torch.norm(hand_positions - object_positions, dim=1)
        reward = -distances

        # Grasp bonus
        grasp_threshold = 0.02
        grasp_bonus = torch.where(
            distances < grasp_threshold,
            torch.ones_like(distances) * 0.5,
            torch.zeros_like(distances)
        )
        reward += grasp_bonus

        self.rew_tensor = reward

    def _compute_dones(self):
        """Episode termination flags compute کریں۔"""
        # Max episode length (simulated steps)
        # Object table سے گر گیا ہے
        object_positions = self.root_tensor[1::2, 2]
        fallen = object_positions < 0.01

        self.done_tensor = fallen

    def close(self):
        """Resources clean up کریں۔"""
        self.gym.destroy_sim(self.sim)
```

یہ کوڈ Isaac Gym میں ایک مکمل parallel grasping environment دکھاتا ہے۔ کلاس 4096 environments کو parallel میں چلاتا ہے، جہاں ہر robot hand ایک object grab کرنا سیکھتا ہے۔ environment reset، observation collection، reward computation، اور episode termination سب شامل ہیں۔

```python
# Isaac Gym parallel simulation environment
# 4096 environments میں سے ہر ایک robot hand سیکھ رہا ہے
```

#### 2. Manipulation کے لیے Reward ڈیزائن

Reward functions policy learning کو مطلوبہ behaviors کی طرف رہنمائی کرتی ہیں۔ موثر reward ڈیزائن میں multiple objectives کو平衡 میں رکھنا، learning trajectories کو shape کرنا، اور local optima سے بھینا شامل ہے۔ Manipulation tasks کے لیے، common objectives میں task completion، effort minimization، اور constraint satisfaction شامل ہیں۔

Sparse rewards task success یا failure کے لیے ایک binary signal فراہم کرتی ہے۔ یہ approach سادہ ہے لیکن slow learning کی طرف لے جا سکتا ہے کیونکہ successful behaviors کی discovery کے لیے exploration درکار ہوتی ہے۔ Shaped rewards continuous feedback فراہم کرتی ہے، learning کو تیز کرتی ہے لیکن reward hacking کا risk ہے جہاں policy unexpectedly reward function کو exploit کرتی ہے۔

Grasping tasks کے لیے، rewards عام طور پر objects کی طرف distance-based attraction، grasp success bonuses، اور ممکنہ excessive joint velocities یا torques کے penalties کو combine کرتی ہیں۔ ان components کا relative weighting learning dynamics اور final policy behavior کو determine کرتی ہے۔

#### 3. Policy Training اور Evaluation

Training loop ڈیزائن learning efficiency اور final policy quality پر اہم اثر ڈالتی ہے۔ اہم considerations میں learning rate scheduling، batch size selection، اور evaluation frequency شامل ہیں۔ Isaac Gym reference implementations فراہم کرتا ہے جو custom training runs کے لیے starting points کے طور پر کام کر سکتی ہیں۔

Training کے دوران evaluation deployment کے لیے best policy checkpoint کی identification کرتی ہے۔ Evaluation environments کو consistent comparisons کے لیے fixed configurations استعمال کرنی چاہیے۔ Metrics including success rate، completion time، اور policy smoothness checkpoint selection کو guide کرتی ہیں۔

### اہم تصورات میں مہارت حاصل کرنا

- **GPU Simulation**: Parallel environment execution، memory management، tensor operations
- **RL Algorithms**: PPO objective، advantage estimation، entropy regularization
- **Reward Engineering**: Dense vs sparse rewards، reward shaping، reward hacking سے بھینا
- **Policy Architecture**: Network design، observation encoding، action parameterization
- **Training Dynamics**: Learning rate schedules، batch sizes، gradient clipping

### مشق کی تماریں

1. **مشق 1: Isaac Gym کی تنصیب اور بنیادی Simulation** (3 گھنٹے)
   - Isaac Gym package انسٹال کریں
   - GPU physics settings کانفیگر کریں
   - parallel simulation کے ساتھ بنیادی environment چلائیں
   - throughput measure کریں (environment steps per second)

2. **مشق 2: Grasping Environment Implementation** (5 گھنٹے)
   - robot hand کے ساتھ grasping environment بنائیں
   - randomization کے ساتھ environment reset لاگو کریں
   - observation اور action spaces کی تعریف کریں
   - بنیادی reward function لاگو کریں

3. **مشق 3: PPO Training Setup** (4 گھنٹے)
   - PPO policy network لاگو کریں
   - parallel rollouts کے ساتھ training loop سیٹ اپ کریں
   - optimizer اور learning rate schedule کانفیگر کریں
   - advantage estimation (GAE) لاگو کریں

4. **مشق 4: Grasp Policy Training** (6 گھنٹے)
   - object grasping کے لیے policy train کریں
   - training progress monitor کریں (reward curves، success rates)
   - تیز تر convergence کے لیے hyperparameters tune کریں
   - desired performance پر checkpoint save کریں

5. **مشق 5: Policy Evaluation اور Export** (3 گھنٹے)
   - held-out scenarios پر trained policy کا جائزہ لیں
   - failure modes analyze کریں اور improvements identify کریں
   - TensorRT format میں policy export کریں
   - deployment کے لیے inference script بنائیں

### تخمینہ وقت کا عہدہ

| سرگرمی | گھنٹے |
|--------|-------|
| پڑھنا اور tutorials | 8 |
| مشق 1: تنصیب | 3 |
| مشق 2: Environment | 5 |
| مشق 3: PPO Setup | 4 |
| مشق 4: Training | 6 |
| مشق 5: Evaluation | 3 |
| troubleshooting اور جائزہ | 5 |
| **کل** | **34 گھنٹے** |

## ہفتہ 10: Reinforcement Learning اور Sim-to-Real Transfer

### سیکھنے کے مقاصد

ہفتہ 10 کے اختتام تک، آپ درج ذیل کرنے میں قادر ہوں گے:
- robust policy training کے لیے domain randomization techniques لاگو کریں
- simulation calibration کے لیے system identification لاگو کریں
- simulation-reality gaps کو پر کرنے کے لیے domain adaptation استعمال کریں
- trained policies کو physical robot hardware پر deploy کریں
- safety monitoring اور fallback behaviors لاگو کریں
- transfer performance کا جائزہ لیں اور improvement opportunities identify کریں

### اہم موضوعات

#### 1. Domain Randomization

Domain randomization training کے دوران policies کو varied conditions سے expose کرتی ہے، deployment پر distribution shift کے لیے robustness کو promote کرتی ہے۔ اہم insight یہ ہے کہ اگر real-world distribution training distribution کے اندر contain ہو، تو policies explicit adaptation کے بغیر generalize کرتی ہیں۔

Effective randomization ان parameters کو target کرتی ہے جو task performance کو affect کرتی ہیں اور real-world variation exhibit کرتی ہیں۔ Manipulation کے لیے، relevant parameters میں object appearance (color، texture، shape)، physical properties (mass، friction، damping)، اور sensor characteristics (noise، exposure، lens distortion) شامل ہیں۔ Randomization range کو expected real-world variation کو span کرنا چاہیے جبکہ learnable رہنا چاہیے۔

Isaac Sim میں implementation environment reset پر configuration randomization استعمال کرتی ہے۔ ہر reset predefined distributions سے parameter values sample کرتی ہے، varied training episodes produce کرتی ہے۔ Randomization schedule training کے دوران evolve کر سکتی ہے، بڑے variation سے شروع ہو کر جیسے جیسے policies improve ہوں narrow ہو جائے۔

```python
# مثال: Manipulation training کے لیے Domain Randomization
import numpy as np
from typing import Dict, Tuple
from dataclasses import dataclass
from enum import Enum

class RandomizationType(Enum):
    UNIFORM = "uniform"
    GAUSSIAN = "gaussian"
    DISCRETE = "discrete"


@dataclass
class RandomizationConfig:
    """ایک واحد randomization parameter کے لیے کانفیگریشن۔"""
    name: str
    base_value: float
    randomization_type: RandomizationType
    min_value: float
    max_value: float
    std: float  # Gaussian randomization کے لیے


class DomainRandomizer:
    """
    Sim-to-real transfer کے لیے domain randomization۔
    Simulation parameters پر randomized variations apply کرتا ہے۔
    """

    def __init__(self):
        self.configs: Dict[str, RandomizationConfig] = {}
        self.randomization_probability = 1.0

    def add_parameter(self, config: RandomizationConfig):
        """Randomize کرنے کے لیے ایک parameter شامل کریں۔"""
        self.configs[config.name] = config

    def randomize_all(self) -> Dict[str, float]:
        """
        سب parameters کے لیے random values sample کریں۔

        Returns:
            values: Parameter names کی dictionary to randomized values
        """
        values = {}

        for name, config in self.configs.items():
            if np.random.random() < self.randomization_probability:
                if config.randomization_type == RandomizationType.UNIFORM:
                    value = np.random.uniform(config.min_value, config.max_value)
                elif config.randomization_type == RandomizationType.GAUSSIAN:
                    value = np.clip(
                        np.random.normal(config.base_value, config.std),
                        config.min_value,
                        config.max_value
                    )
                elif config.randomization_type == RandomizationType.DISCRETE:
                    value = np.random.choice(
                        [config.min_value, config.base_value, config.max_value]
                    )
                else:
                    value = config.base_value
            else:
                value = config.base_value

            values[name] = value

        return values


class IsaacSimRandomizer:
    """
    Isaac Sim کے ساتھ integrated domain randomizer۔
    Physics، appearance، اور sensors پر randomizations apply کرتا ہے۔
    """

    def __init__(self, sim, randomizer: DomainRandomizer):
        self.sim = sim
        self.randomizer = randomizer
        self.current_values = {}

    def randomize(self):
        """Simulation پر randomizations apply کریں۔"""
        values = self.randomizer.randomize()
        self.current_values = values

        # Physics randomizations apply کریں
        self._randomize_physics(values)

        # Visual randomizations apply کریں
        self._randomize_appearance(values)

        # Sensor randomizations apply کریں
        self._randomize_sensors(values)

    def _randomize_physics(self, values: Dict[str, float]):
        """Physics parameters randomize کریں۔"""
        # Object mass randomization
        if "object_mass" in values:
            self._set_object_mass(values["object_mass"])

        # Friction randomization
        if "friction" in values:
            self._set_friction(values["friction"])

        # Damping randomization
        if "damping" in values:
            self._set_damping(values["damping"])

    def _randomize_appearance(self, values: Dict[str, float]):
        """Objects کی visual appearance randomize کریں۔"""
        # Color randomization
        if "object_color" in values:
            self._set_object_color(values["object_color"])

        # Texture randomization
        if "texture_scale" in values:
            self._set_texture_scale(values["texture_scale"])

        # Lighting randomization
        if "light_intensity" in values:
            self._set_light_intensity(values["light_intensity"])

    def _randomize_sensors(self, values: Dict[str, float]):
        """Sensor characteristics randomize کریں۔"""
        # Camera noise
        if "camera_noise" in values:
            self._set_camera_noise_level(values["camera_noise"])

        # Exposure randomization
        if "exposure" in values:
            self._set_camera_exposure(values["exposure"])

        # IMU noise
        if "imu_noise" in values:
            self._set_imu_noise_level(values["imu_noise"])

    def _set_object_mass(self, mass: float):
        """Manipulable objects کے لیے mass set کریں۔"""
        # Isaac Gym APIs کا استعمال کرتے ہوئے implementation
        pass

    def _set_friction(self, friction: float):
        """Contact surfaces کے لیے friction coefficient set کریں۔"""
        pass

    def _set_damping(self, damping: float):
        """Joint damping parameters set کریں۔"""
        pass

    def _set_object_color(self, color: np.ndarray):
        """Object materials کے لیے RGB color set کریں۔"""
        pass

    def _set_texture_scale(self, scale: float):
        """Texture tiling scale set کریں۔"""
        pass

    def _set_light_intensity(self, intensity: float):
        """Light source intensity set کریں۔"""
        pass

    def _set_camera_noise_level(self, noise_std: float):
        """Camera noise standard deviation set کریں۔"""
        pass

    def _set_camera_exposure(self, exposure: float):
        """Camera exposure time set کریں۔"""
        pass

    def _set_imu_noise_level(self, noise_scale: float):
        """IMU noise scale factors set کریں۔"""
        pass


def create_manipulation_randomizer() -> IsaacSimRandomizer:
    """Manipulation tasks کے لیے domain randomizer بنائیں۔"""
    randomizer = DomainRandomizer()

    # Physics randomizations
    randomizer.add_parameter(RandomizationConfig(
        name="object_mass",
        base_value=0.5,
        randomization_type=RandomizationType.GAUSSIAN,
        min_value=0.1,
        max_value=2.0,
        std=0.3
    ))

    randomizer.add_parameter(RandomizationConfig(
        name="friction",
        base_value=0.5,
        randomization_type=RandomizationType.UNIFORM,
        min_value=0.3,
        max_value=0.8,
        std=0.0
    ))

    randomizer.add_parameter(RandomizationConfig(
        name="damping",
        base_value=0.5,
        randomization_type=RandomizationType.GAUSSIAN,
        min_value=0.1,
        max_value=1.0,
        std=0.2
    ))

    # Visual randomizations
    randomizer.add_parameter(RandomizationConfig(
        name="object_color",
        base_value=np.array([0.5, 0.5, 0.5]),
        randomization_type=RandomizationType.DISCRETE,
        min_value=0.0,
        max_value=1.0,
        std=0.0
    ))

    randomizer.add_parameter(RandomizationConfig(
        name="light_intensity",
        base_value=1000.0,
        randomization_type=RandomizationType.GAUSSIAN,
        min_value=500.0,
        max_value=2000.0,
        std=300.0
    ))

    # Sensor randomizations
    randomizer.add_parameter(RandomizationConfig(
        name="camera_noise",
        base_value=0.01,
        randomization_type=RandomizationType.UNIFORM,
        min_value=0.0,
        max_value=0.05,
        std=0.0
    ))

    randomizer.add_parameter(RandomizationConfig(
        name="exposure",
        base_value=0.01,
        randomization_type=RandomizationType.GAUSSIAN,
        min_value=0.001,
        max_value=0.033,
        std=0.005
    ))

    return IsaacSimRandomizer(None, randomizer)
```

یہ کوڈ manipulation training کے لیے ایک مکمل domain randomization system دکھاتا ہے۔ `DomainRandomizer` کلاس مختلف قسم کے randomization configurations کو manage کرتی ہے، جبکہ `IsaacSimRandomizer` Isaac Sim کے ساتں integration فراہم کرتی ہے۔ یہ physics، appearance، اور sensor parameters کو randomize کرتی ہے۔

```python
# Domain randomization system برائے sim-to-real transfer
# Physics، visual، اور sensor parameters کو randomize کرتا ہے
```

#### 2. System Identification

System identification physical system characteristics کو measure کرتا ہے تاکہ simulation parameters tune کیے جا سکیں۔ یہ calibration sim-to-real gap کو کم کرتی ہے by improving simulation fidelity۔ ہیومینائڈ روبوٹس کے لیے، key identification targets میں joint dynamics، sensor characteristics، اور contact properties شامل ہیں۔

Actuator identification commanded signals اور actual motion کے درمیان relationship کو measure کرتی ہے۔ اس میں sweep experiments سے data collect کرنا اور motor models، gearbox dynamics، اور friction کے لیے parameters fit کرنا شامل ہے۔ Identified parameters simulation joint controllers کو configure کرتی ہیں کہ physical behavior سے match کریں۔

Sensor identification IMUs، encoders، اور cameras کے لیے noise، bias، اور scale factors characterize کرتی ہے۔ IMU calibration known rotation sequences کا استعمال کرتی ہے to determine accelerometer اور gyroscope parameters۔ Camera calibration checkerboard patterns کا استعمال کرتی ہے to measure intrinsics اور distortion coefficients۔

#### 3. Physical Deployment

Trained policies کو physical robots پر deploy کرنے کے لیے trained models کو optimized inference engines میں convert کرنا ضروری ہے۔ Isaac Jetson platforms پر GPU-optimized inference کے لیے TensorRT export فراہم کرتا ہے۔ Deployment pipeline sensor input processing، policy inference، اور actuator command execution کو real time میں handle کرتی ہے۔

Physical deployment کے دوران safety monitoring essential ہے۔ Simulation میں train کردہ policies غیر متوقع conditions کا سامنا کر سکتی ہیں جو dangerous behavior کا سبب بنیں۔ Safety systems joint limits، velocities، forces، اور external contact کو monitor کرنی چاہیں۔ Emergency stop triggers fail-safe behavior فراہم کرتی ہیں جب safety limits exceeded ہوں۔

### اہم تصورات میں مہارت حاصل کرنا

- **Domain Randomization**: Parameter selection، randomization ranges، schedules
- **System Identification**: Actuator characterization، sensor calibration، parameter fitting
- **TensorRT Optimization**: Model export، precision selection، inference optimization
- **Safety Systems**: Limit monitoring، emergency procedures، fallback controllers
- **Deployment Pipeline**: Sensor interfaces، real-time constraints، latency management

### مشق کی تماریں

1. **مشق 1: Domain Randomization Implementation** (4 گھنٹے)
   - Domain randomizer class لاگو کریں
   - Physics parameter randomization شامل کریں (mass، friction، damping)
   - Visual randomization شامل کریں (colors، lighting)
   - Sensor noise randomization شامل کریں
   - Training loop کے ساتھ integrate کریں

2. **مشق 2: System Identification Setup** (4 گھنٹے)
   - System identification experiments ڈیزائن کریں
   - Actuator characterization data collect کریں
   - Motor اور friction models fit کریں
   - Simulation parameters update کریں

3. **مشق 3: TensorRT Export** (3 گھنٹے)
   - Trained policy کو TensorRT میں export کریں
   - Precision configure کریں (FP16/INT8)
   - Inference performance benchmark کریں
   - Exported model کی correctness verify کریں

4. **مشق 4: Physical Deployment** (5 گھنٹے)
   - Robot hardware interface سیٹ اپ کریں
   - Sensor input pipeline کانفیگر کریں
   - Safety monitoring لاگو کریں
   - Physical robot پر policy deploy کریں اور test کریں

5. **مشق 5: Transfer Evaluation** (3 گھنٹے)
   - Physical performance vs simulation کا جائزہ لیں
   - Failure modes اور gaps identify کریں
   - Randomization اور calibration پر iterate کریں
   - Transfer results document کریں

### تخمینہ وقت کا عہدہ

| سرگرمی | گھنٹے |
|--------|-------|
| پڑھنا اور tutorials | 8 |
| مشق 1: Randomization | 4 |
| مشق 2: System ID | 4 |
| مشق 3: TensorRT Export | 3 |
| مشق 4: Deployment | 5 |
| مشق 5: Evaluation | 3 |
| troubleshooting اور جائزہ | 5 |
| **کل** | **32 گھنٹے** |

## خلاصہ اور اگلے steps

ہفتے 8-10 نے physical AI development کے لیے NVIDIA Isaac ecosystem کی جامع coverage فراہم کی ہے۔ ہفتہ 8 نے Isaac Sim setup اور perception pipeline development میں foundations قائم کیے۔ ہفتہ 9 نے Isaac Gym کے ساتھ GPU-accelerated manipulation learning cover کیا۔ ہفتہ 10 نے critical sim-to-real transfer challenge کو practical deployment techniques کے ساتھ address کیا۔

ان ہفتوں میں develop کی گئی skills آپ کو حصہ 5: ہیومینائڈ روبوٹ ڈیزائن کے لیے تیار کرتی ہیں، جہاں آپ simulation اور learning techniques کو specific humanoid robot platforms پر apply کریں گے۔ Isaac capabilities کا humanoid kinematics اور dynamics کے ساتھ integration simulation سے physical deployment تک end-to-end development کو enable کرے گی۔

آگے بڑھتے ہوئے، continued learning کے لیے ان areas پر غور کریں:
- Advanced RL algorithms (offline RL، meta-learning)
- Multi-task اور hierarchical learning
- Robotics کے لیے vision-language models
- Manipulation planning کے لیے foundation models
