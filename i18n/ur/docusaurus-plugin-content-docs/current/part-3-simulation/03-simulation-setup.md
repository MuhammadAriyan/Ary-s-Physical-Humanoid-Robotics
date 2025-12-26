---
title: "Gazebo اور Unity سیمولیشن"
sidebar_position: 3
---

# Chapter 3: Gazebo اور Unity سیمولیشن

## سیکھنے کے مقاصد

اس باب کے اختتام پر، آپ درج ذیل کرنے میں قادر ہوں گے:
- روبوٹ سیمولیشن کی اہمیت کو سمجھنا
- ہیومینوئڈ روبوٹ ٹیسٹنگ کے لیے Gazebo سیمولیشن ماحول کی تشکیل کرنا
- سینسرز کے ساتھ روبوٹ ماڈلز کے لیے URDF اور SDF تفصیلات بنانا
- جوڑوں، پابندیوں، اور ٹکراؤ کی خصوصیات سمیت فزکس سیمولیشن کی تشکیل کرنا
- کیمروں، LIDAR، اور IMUs کے لیے سینسر سیمولیشن نافذ کرنا
- ہائی-فیڈیلیٹی روبوٹ ویزولائزیشن اور رینڈرنگ کے لیے Unity کا استعمال کرنا
- مکمل ترقیاتی ورک فلو کے لیے ROS 2 کے ساتھ سیمولیشن کی انٹیگریشن کرنا

## 3.1 روبوٹ سیمولیشن کا تعارف

روبوٹ سیمولیشن ہیومینوئڈ روبوٹکس کی ترقی میں ایک لازمی ٹول بن چکا ہے۔ سیمولیشن کی تشکیل اور کنفیگریشن کی تکنیکی پہلوؤں میں جانے سے پہلے، یہ سمجھنا ضروری ہے کہ سیمولیشن کیوں اہم ہے اور یہ فزیکل ہیومینوئڈ روبوٹس کے لیے مکمل ترقیاتی ورک فلو میں کیسے فٹ ہوتی ہے۔

### سیمولیشن کیوں ضروری ہے؟

ہیومینوئڈ روبوٹس کی ترقی میں اہم خطرات اور لاگت شامل ہے جو سیمولیشن کی مدد سے کم ہوتی ہے۔ اصل ہارڈویئر پر فزیکل ٹیسٹنگ سے آلات کو نقصان، ذاتی چوٹ، اور مرمت کے دوران اہم ڈاؤن ٹائم ہو سکتا ہے۔ 30+ ڈگری آف فریڈم، پیچیدہ سینسرز، اور پیچیدہ کنٹرول سسٹم والا ایک ہیومینوئڈ روبوٹ اکثر لاکھوں ڈالر کی مالیت کی سرمایہ کاری کرتا ہے۔ سیمولیشن ڈویلپرز کو الگورتھم کی جانچ، کنٹرول حکمت عملی کی توثیق، اور ڈیزائن پر تکرار کرنے کی اجازت دیتی ہے بغیر اس مہنگے ہارڈویئر کو خطرے میں ڈالے۔

خطرے کی تخفیف سے آگے، سیمولیشن ایسے ترقیاتی منظرنامے کو فعال کرتی ہے جو فزیکل روبوٹس کے ساتھ غیر عملی یا ناممکن ہوں گے۔ خطرناک ماحول، انتہائی درجہ حرارت، یا خلائی حالات میں روبوٹ سلوک کی جانچ کے لیے سیمولیشن ضروری ہے۔ توثیق اور سرٹیفیکیشن کے لیے کنارے کے کیسز اور ناکامی کے طریقوں کو دہرانا سیمولیشن میں سیدھا ہے۔ محققین رینفورسمینٹ لرننگ کے ذریعے روبوٹ سیکھنے کو تلاش کر سکتے ہیں، جو اصل ہارڈویئر پر سالوں لینے والی لاکھوں تکراروں کی جانچ کرتے ہیں۔

سیمولیشن روبوٹ ڈویلپمنٹ کو جمہوری بناتی ہے فزیکل انفراسٹرکچر کی ضروریات کو کم کرکے۔ اچھے سے لیس روبوٹکس لیب تک رسائی نہ رکھنے والی ٹیمیں بھی ابھی بھی الگورتھم کی ترقی اور جانچ کر سکتی ہیں۔ یہ رسائی اختراع کو تیز کرتی ہے اور زیادہ محققین اور ڈویلپرز کو فیلڈ میں حصہ لینے کی اجازت دیتی ہے۔

### سیمولیشن وفاداری کے پیش نظر

ہر سیمولیشن برابر نہیں بنتی، اور وفاداری کی مناسب سطح کا انتخاب ترقیاتی مرحلے اور مقاصد پر منحصر ہے۔ لو-فیڈیلیٹی سیمولیشن رفتار اور اسکیلیبلٹی کو ترجیح دیتی ہے، جو الگورتھم ڈویلپمنٹ، انٹیگریشن ٹیسٹنگ، اور تیز پروٹو ٹائپنگ کے لیے موزو ہے۔ ہائی-فیڈیلیٹی سیمولیشن فزیکل درستگی، سینسر ریئلزم، اور ماحول کی تفصیل پر توجہ دیتی ہے، جو حتمی توثیق، پرسیپشن الگورتھم ڈویلپمنٹ، اور ہارڈویئر-ان-دی-لوپ ٹیسٹنگ کے لیے ضروری ہے۔

Gazebo وفادور اور کارکردگی کے درمی ایک مؤثر توازن پیش کرتا ہے، کنفیگریبل فزکس انجنز، ریئلسٹک سینسر ماڈلز، اور مؤثر کمپیوٹیشن فراہم کرتا ہے۔ ہیومینوئڈ روبوٹکس کے لیے، Gazebo ROS انٹیگریشن، وسیع سینسر ماڈل لائبریری، اور فعال کمیونٹی سپورٹ کی وجہ سے معیاری سیمولیشن ماحول بن چکا ہے۔ Unity Gazebo کو ایک بہتر رینڈرنگ کی فراہمی کرکے پورا کرتا ہے جو ویزولائزیشن، مارکیٹنگ مواد، اور ہیومن-روبوٹ انٹریکشن کے مطالعات کے لیے ہے۔

### سیمولیشن ڈویلپمنٹ ورک فلو

موثر سیمولیشن انٹیگریشن ایک مرحلہ وار اپروچ پر عمل کرتی ہے جو ہارڈویئر کی تیاری اور ترقی کی پختگی کے ساتھ مماثلت رکھتی ہے۔ ابتدائی مراحل میں، جب الگورتھم تیار کیے جا رہے ہیں اور تصورات کی توثیق ہو رہی ہے، سیمولیشن بنیادی ترقیاتی ماحول فراہم کرتی ہے۔ ہارڈویئر-ان-دی-لوپ ٹیسٹنگ اس وقت شروع ہوتی ہے جب بنیادی کارکردگی کی توثیق ہو جاتی ہے، سیمولیشن کے ساتھ فزیکل روبوٹ ٹیسٹنگ کا استعمال کرتے ہوئے۔ جیسے جیسے ترقی پکتی ہے، سیمولیشن ریگریشن ٹیسٹنگ، کنارے کے کیس کی توثیق، اور مسلسل انٹیگریشن کے لیے بڑھتی ہوئی اہمیت حاصل کرتی ہے۔

اس ورک فلو کو سیمولیشن اور فزیکل روبوٹ کے نفاذات کے درمی مماثلت برقرار رکھنے کی ضرورت ہے۔ سینسر کنفیگریشنز، ایکٹیویٹر کی خصوصیات، یا فزیکل پیرامیٹرز میں تبدیلیاں دونوں ماحولات میں پھیلنا چاہیئے۔ واضح انٹرفیس اور ورژن کنٹرول کی مشقیں سیمولیٹڈ اور فزیکل سسٹمز کے درمی ڈریفٹ کو روکتی ہیں۔

## 3.2 Gazebo سیمولیشن ماحول کی تشکیل

Gazebo کی تعمیر سیمولیشن سرور کو ویزولائزیشن کلائنٹس سے الگ کرتی ہے، ایک سے زیادہ صارفین کو ایک ہی سیمولیشن انسٹینز کو دیکھنے اور اس کے ساتھ تعامل کرنے کی اجازت دیتی ہے۔ اس تعمیر کو سمجھنا مؤثر ترقیاتی ماحول کی تشکیل اور مسائل کے故障诊断 میں مدد کرتا ہے۔

### تنصیب اور کنفیگریشن

Ubuntu 22.04 پر Gazebo کی تنصیب osrfoundation ریپوزٹری کے ذریعے ایک سیدھے عمل پر عمل کرتی ہے:

```bash
# Gazebo ریپوزٹری شامل کریں
sudo apt-get update
sudo apt-get install gnupg
sudo wget https://packages.osrfoundation.org/gazebo/keys/packages.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Gazebo Harmonic (موجودہ تجویز کردہ ورژن) تنصیب کریں
sudo apt-get update
sudo apt-get install gz-harmonic ros-humble-gazebo-ros-pkgs

# تنصیب کی تصدیق کریں
gz sim --version
```

سیمولیشن کو ہیومینوئڈ ماڈلز کے ساتھ ہموار آپریشن کے لیے احتیاط سے وسائل مختص کرنے کی ضرورت ہے۔ فزکس سیمولیشن سائیز کا سائز، عام طور پر درست ڈائنامکس کے لیے 1 ملی سیکنڈ، کمپیوٹیشنل ضروریات کا تعین کرتا ہے۔ ایک سیمولیشن سٹیپ کے لیے تمام جوڑوں کے پوزیشنز، ویلوسیٹیز، اور ایکسیلریشنز کی حساب لگانا، رابطوں کو حل کرنا، اور سینسر ریڈنگ کو اپ ڈیٹ کرنا ضروری ہے۔ 30+ جوڑوں، متعدد رابطے کے پوائنٹس، اور کئی سینسرز والے ہیومینوئڈ روبوٹس سنگل کور کمپیوٹیشن پر دباؤ ڈال سکتے ہیں۔

پیچیدہ ہیومینوئڈ سیمولیشنز کے لیے، ملٹی-تھریڈڈ فزکس یا ڈسٹریبیوٹڈ سیمولیشن کی تشکیل پر غور کریں۔ Gazebo فزکس کو رینڈرنگ سے الگ تھریڈ میں چلانے کی سپورٹ کرتا ہے، سیمولیشن درستگی پر ویزولائزیشن کے اثر کو کم کرتا ہے۔ درج ذیل کنفیگریشن تھریڈ الگ کرنے اور وسائل کی مختص کرنے کا مظاہرہ کرتی ہے:

```xml
<!-- ~/.gz/sim/config/humanoid_simulation.config -->
<sdf version="1.10">
  <world name="humanoid_world">
    <physics name="multi-threaded-physics" default="true" type="omp">
      <engine name="dart">
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1.0</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <num_threads>4</num_threads>
      </engine>
    </physics>
    <gui>
      <camera name="main_camera">
        <pose>2.0 2.0 1.5 0 0.3 2.5</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

### ماحول کی تشکیل

مناسب سیمولیشن ماحول بنانے میں گریویٹی، لائٹنگ، گراؤنڈ پلین، اور ماحولیاتی حالات کی تشکیل شامل ہے۔ ہیومینوئڈ روبوٹس کو احتیاط سے گراؤنڈ رابطے کی ماڈلنگ کی ضرورت ہے، کیونکہ چلتی ڈائنامکس رگڑ اور رابطہ حل پر فیصلی کن طور پر انحصار کرتی ہیں۔

```xml
<!-- humanoid_world.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="humanoid_simulation">
    <!-- چلتے ہوئے سیمولیشن کے لیے فزکس کنفیگریشن -->
    <physics name="dart" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
      <dart name="dart">
        <solver>
          <type>dantzig</type>
          <dtosolver_iterations>30</dtosolver_iterations>
          <sor_iterations>50</sor_iterations>
          <use_adaptive_time_stepping>true</use_adaptive_time_stepping>
        </solver>
      </dart>
    </physics>

    <!-- چلتے ہوئے کے لیے رگڑ کنفیگریشن کے ساتھ گراؤنڈ پلین -->
    <model name="ground_plane">
      <pose>0 0 0 0 0 0</pose>
      <link name="ground">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <size>20 20</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
                <fdir1>0 0 1</fdir1>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
              </ode>
            </friction>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <contact_cfm>0.0</contact_cfm>
              <contact_erp>0.2</contact_erp>
            </contact>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Gray</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- پرسیپشن ٹیسٹنگ کے لیے لائٹنگ کنفیگریشن -->
    <light name="sun" type="directional">
      <pose>5 5 10 0 0.5 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <cast_shadows>true</cast_shadows>
      <intensity>1.0</intensity>
      <direction>0.1 -0.1 -1</direction>
    </light>

    <!-- Gazebo ماڈل ڈیٹابیس سے سینسر ماڈلز شامل کریں -->
    <include>
      <uri>model://camera</uri>
      <name>head_camera</uri>
    </include>
  </world>
</sdf>
```

## 3.3 URDF اور SDF روبوٹ تفصیلات کے فارمیٹ

روبوٹ تفصیلات کے فارمیٹس سیمولیشن، ویزولائزیشن، اور موشن پلاننگ کے لیے روبوٹ ساخت کی ریاضیاتی نمائندگی فراہم کرتے ہیں۔ URDF اور SDF دونوں کو، انی کی طاقتوں، اور مناسب استعمال کے کیسز کو سمجھنا ہیومینوئڈ روبوٹ ڈویلپمنٹ کے لیے ضروری ہے۔

### یونیفائڈ روبوٹ ڈسکرپشن فارمیٹ (URDF)

URDF روبوٹ کائنیٹکس اور بصری خصوصیات کو بیان کرنے کے لیے XML استعمال کرتا ہے۔ فارمیٹ روبوٹس کو جوڑوں سے منسلک لنکس کے درخت کے طور پر منظم کرتا ہے، جہاں لنکس سخت اجسام کی نمائندگی کرتے ہیں اور جوڑیں نسبی حرکت کی اجازت دینے والے کنکشنز کی نمائندگی کرتے ہیں۔

```xml
<!-- humanoid.urdf.xacro -->
<?xml version="1.0" ?>
<robot name="atlas_humanoid" xmlns:xacro="http://wiki.ros.org/xacro">
  <!-- میکرورس اور مستقلات شامل کریں -->
  <xacro:include filename="$(find humanoid_description)/urdf/constants.urdf.xacro" />
  <xacro:include filename="$(find humanoid_description)/urdf/materials.urdf.xacro" />

  <!-- بیس لنک - دنیا کا کنکشن -->
  <link name="base_link">
    <inertial>
      <mass value="${base_mass}" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="${base_ixx}" ixy="${base_ixy}" ixz="${base_ixz}"
               iyy="${base_iyy}" iyz="${base_iyz}" izz="${base_izz}" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${base_width} ${base_depth} ${base_height}" />
      </geometry>
      <material name="base_color" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${base_width} ${base_depth} ${base_height}" />
      </geometry>
    </collision>
  </link>

  <!-- پیلسس لنک - مرکزی جسم کا حوالہ -->
  <link name="pelvis_link">
    <inertial>
      <mass value="${pelvis_mass}" />
      <origin xyz="0 0 ${pelvis_z_offset}" rpy="0 0 0" />
      <inertia ixx="${pelvis_ixx}" ixy="0" ixz="0"
               iyy="${pelvis_iyy}" iyz="0" izz="${pelvis_izz}" />
    </inertial>
    <visual>
      <origin xyz="0 0 ${pelvis_z_offset}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${pelvis_radius}" length="${pelvis_length}" />
      </geometry>
      <material name="pelvis_color" />
    </visual>
    <collision>
      <origin xyz="0 0 ${pelvis_z_offset}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${pelvis_radius}" length="${pelvis_length}" />
      </geometry>
    </collision>
  </link>

  <!-- یاو گردش کے لیے کنٹینیوس جوائنٹ -->
  <joint name="base_to_pelvis" type="fixed">
    <origin xyz="0 0 ${pelvis_height}" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="pelvis_link" />
    <axis xyz="0 0 1" />
  </joint>

  <!-- بائیں ہپ یاو جوائنٹ -->
  <joint name="left_hip_yaw" type="revolute">
    <origin xyz="0 ${hip_separation/2} 0" rpy="0 0 0" />
    <parent link="pelvis_link" />
    <child link="left_hip_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="150" velocity="5.0" />
    <dynamics damping="0.5" friction="2.0" />
  </joint>

  <!-- بائیں ہپ لنک -->
  <link name="left_hip_link">
    <inertial>
      <mass value="2.5" />
      <origin xyz="0 0.05 -0.1" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.02" iyz="0.001" izz="0.01" />
    </inertial>
    <visual>
      <geometry>
        <box size="0.15 0.12 0.25" />
      </geometry>
      <material name="hip_color" />
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.12 0.25" />
      </geometry>
    </collision>
  </link>

  <!-- بائیں ہپ رول جوائنٹ -->
  <joint name="left_hip_roll" type="revolute">
    <origin xyz="0 0 -0.15" rpy="0 0 0" />
    <parent link="left_hip_link" />
    <child link="left_upper_leg_link" />
    <axis xyz="1 0 0" />
    <limit lower="-0.5" upper="0.5" effort="150" velocity="5.0" />
    <dynamics damping="0.5" friction="2.0" />
  </joint>

  <!-- بائیں اپری لیگ لنک -->
  <link name="left_upper_leg_link">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 -0.25" rpy="0 0 0" />
      <inertia ixx="0.05" ixy="0" ixz="0"
               iyy="0.02" iyz="0" izz="0.05" />
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.50" />
      </geometry>
      <material name="leg_color" />
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.50" />
      </geometry>
    </collision>
  </link>

  <!-- بائیں کنی جوائنٹ -->
  <joint name="left_knee" type="revolute">
    <origin xyz="0 0 -0.50" rpy="0 0 0" />
    <parent link="left_upper_leg_link" />
    <child link="left_lower_leg_link" />
    <axis xyz="1 0 0" />
    <limit lower="-2.0" upper="0" effort="100" velocity="8.0" />
    <dynamics damping="0.3" friction="1.5" />
  </joint>

  <!-- بائیں لویر لیگ لنک -->
  <link name="left_lower_leg_link">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 -0.25" rpy="0 0 0" />
      <inertia ixx="0.03" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.03" />
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.50" />
      </geometry>
      <material name="leg_color" />
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.50" />
      </geometry>
    </collision>
  </link>

  <!-- بائیں ٹوڑی جوائنٹ -->
  <joint name="left_ankle" type="revolute">
    <origin xyz="0 0 -0.50" rpy="0 0 0" />
    <parent link="left_lower_leg_link" />
    <child link="left_foot_link" />
    <axis xyz="1 0 0" />
    <limit lower="-0.5" upper="0.5" effort="80" velocity="6.0" />
    <dynamics damping="0.3" friction="1.5" />
  </joint>

  <!-- بائیں فٹ لنک -->
  <link name="left_foot_link">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.02" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.12 0.04" />
      </geometry>
      <material name="foot_color" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.12 0.04" />
      </geometry>
    </collision>
  </link>

  <!-- دایاں پیر بائیں پیر کا عکس ہے -->
  <joint name="right_hip_yaw" type="revolute">
    <origin xyz="0 ${-hip_separation/2} 0" rpy="0 0 0" />
    <parent link="pelvis_link" />
    <child link="right_hip_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="150" velocity="5.0" />
    <dynamics damping="0.5" friction="2.0" />
  </joint>

  <!-- بائیں شولڈر جوائنٹ -->
  <joint name="left_shoulder_pitch" type="revolute">
    <origin xyz="0 ${shoulder_x_offset} ${shoulder_z_offset}" rpy="0 0 0" />
    <parent link="torso_link" />
    <child link="left_upper_arm_link" />
    <axis xyz="1 0 0" />
    <limit lower="-3.14" upper="3.14" effort="80" velocity="6.0" />
    <dynamics damping="0.3" friction="1.0" />
  </joint>

  <!-- IMU سینسر لنک -->
  <link name="imu_link">
    <inertial>
      <mass value="0.05" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="1e-5" ixy="0" ixz="0"
               iyy="1e-5" iyz="0" izz="1e-5" />
    </inertial>
  </link>

  <!-- پیلسس سے IMU جوائنٹ -->
  <joint name="imu_joint" type="fixed">
    <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0" />
    <parent link="torso_link" />
    <child link="imu_link" />
    <axis xyz="0 0 1" />
  </joint>

  <!-- ہیڈ لنک -->
  <link name="head_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0.15" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.12" />
      </geometry>
      <material name="head_color" />
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.12" />
      </geometry>
    </collision>
  </link>

  <!-- ہیڈ جوائنٹ -->
  <joint name="neck_joint" type="revolute">
    <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0" />
    <parent link="torso_link" />
    <child link="head_link" />
    <axis xyz="0 1 0" />
    <limit lower="-1.0" upper="1.0" effort="20" velocity="3.0" />
    <dynamics damping="0.2" friction="0.5" />
  </joint>

  <!-- ہارڈویئر انٹرفیس کے لیے ٹرانسمیشنز -->
  <transmission name="left_hip_yaw_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_hip_yaw">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_hip_yaw_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_knee_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_knee">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_knee_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
  </transmission>
</robot>
```

یہ کوڈ URDF میں ہیومینوئڈ روبوٹ کی تفصیلت کو بیان کرتا ہے۔ میکرورس کے ذریعیرامیٹرے پائزڈ ڈیفینیشنز کا استعمال کرتا ہے جو چھوٹے ماڈلز سے لے کر بڑے ماڈلز تک اسکیل ایبلٹی فراہم کرتی ہے۔ ہر جوائنٹ کی مکمل کنفیگریشن - حدود، ڈائنامکس، اور ٹرانسمیشنز - شامل ہے۔

### سیمولیشن ڈسکرپشن فارمیٹ (SDF)

SDF URDF سے زیادہ جامع ماڈلنگ کی صلاحیتیں فراہم کرتا ہے، نیسٹڈ ماڈلز، مناظر، پلگ انز، اور ایڈوانسڈ فزکس کی سپورٹ کرتا ہے۔ SDF Gazebo کا مقامی فارمیٹ ہے اور پیچیدہ سیمولیشن منظرناموں کے لیے استعمال کیا جانا چاہیئے۔

```xml
<!-- humanoid_complete.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="humanoid_simulation">
    <plugin name="ros2_interface" filename="libgazebo_ros2_control.so">
      <parameters>$(find humanoid_control)/config/controllers.yaml</parameters>
    </plugin>

    <model name="atlas_humanoid">
      <pose>0 0 1.0 0 0 0</pose>

      <!-- جوائنٹ اسٹیٹ پبلشر پلگ ان -->
      <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>joint_states:=joint_states</remapping>
        </ros>
        <publish_rate>100</publish_rate>
      </plugin>

      <!-- مکمل inertial، visual، اور collision کے ساتھ لنک ڈیفینیشنز -->
      <link name="pelvis">
        <pose>0 0 0.95 0 0 0</pose>
        <inertial>
          <mass>15.0</mass>
          <pose>0 0 0 0 0 0</pose>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.3</iyy>
            <iyz>0</iyz>
            <izz>0.5</izz>
          </inertia>
        </inertial>
        <visual name="pelvis_visual">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
        <collision name="pelvis_collision">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>

      <!-- مکمل جوائنٹ چین کے ساتھ بائیں ٹانگ -->
      <link name="left_hip">
        <pose>0 0.12 0 0 0 0</pose>
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.05</iyy>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="left_hip_visual">
          <geometry>
            <box size="0.1 0.08 0.15</size>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/DarkGray</name>
            </script>
          </material>
        </visual>
      </link>

      <joint name="left_hip_yaw" type="revolute">
        <parent>pelvis</parent>
        <child>left_hip</child>
        <pose>0 0.12 0 0 0 0</pose>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>150</effort>
          <velocity>5.0</velocity>
        </limit>
        <physics>
          <ode>
            <cfm>0.0</cfm>
            <erp>0.2</erp>
          </ode>
        </physics>
      </joint>

      <!-- ہیومینوئڈ کے لیے سینسر پلگ انز -->
      <link name="head_camera_link">
        <pose>0 0 0.25 0 -0.3 0</pose>
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>image_raw:=camera/image_raw</remapping>
            <remapping>camera_info:=camera/camera_info</remapping>
          </ros>
          <camera_name>head_camera</camera_name>
          <frame_name>head_camera_link</frame_name>
          <hack_baseline>0.07</hack_baseline>
        </plugin>
      </link>

      <!-- پیلسس پر IMU سینسر -->
      <link name="imu_link">
        <pose>0 0 0.02 0 0 0</pose>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>imu:=imu/data</remapping>
          </ros>
          <frame_name>imu_link</frame_name>
          <topic_name>/humanoid/imu/data</topic_name>
        </plugin>
      </link>

      <!-- پیروں پر فورس/ٹارک سینسرز -->
      <link name="left_foot_link">
        <pose>0.05 0 -0.45 0 0 0</pose>
        <plugin name="ft_sensor" filename="libgazebo_ros_force_torque.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>force_torque:=left_foot/ft</remapping>
          </ros>
          <frame_name>left_foot_link</frame_name>
          <measure_direction>child_to_parent</measure_direction>
        </plugin>
      </link>
    </model>

    <!-- ماحول کے عناصر -->
    <model name="obstacle_course">
      <pose>2.0 0 0 0 0 0</pose>
      <link name="box1">
        <pose>0 0 0.25 0 0 0</pose>
        <visual name="box1_visual">
          <geometry>
            <box size="0.5 0.5 0.5</size>
          </geometry>
        </visual>
        <collision name="box1_collision">
          <geometry>
            <box size="0.5 0.5 0.5</size>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## 3.4 فزکس سیمولیشن

معنی خیز ہیومینوئڈ روبوٹ ٹیسٹنگ کے لیے درست فزکس سیمولیشن بنیادی ہے۔ یہ حصہ جوڑ ڈائنامکس، رابطہ ماڈلنگ، اور حقیقی سیمولیشن سلوک کے لیے پابندی کنفیگریشن کا احاطہ کرتا ہے۔

### جوائنٹ اقسام اور کنفیگریشن

ہیومینوئڈ روبوٹس کو اپنی حرکت کی حد حاصل کرنے کے لیے مختلف جوائنٹ اقسام کی ضرورت ہوتی ہے۔ ریوولیوٹ جوائنٹس واحد محور گردہ فراہم کرتے ہیں جو حیاتیاتی جوڑوں میں عام ہے۔ کنٹینیوئس جوائنٹس گردن اور کمر کی یاو کے لیے لامحدود گردش کی اجازت دیتے ہیں۔ پرزمیٹک جوائنٹس سسپنشن سسٹمز کے لیے لینیئر موشن کو فعال کرتے ہیں۔ فکسڈ جوائنٹس لنکس کو سختی سے جوڑتے ہیں جو ایک دوسرے کے نسبت حرکت نہیں کرنا چاہیئے۔

```yaml
# joint_config.yaml
# ہیومینوئڈ روبوٹ سیمولیشن کے لیے جوائنٹ کنفیگریشن
# حوالہ: فزکس انجن کنفیگریشن کی تفصیلات کے لیے ضمیمہ B دیکھیں

joints:
  # لوئر باڈی جوائنٹس - چلتے ہوئے کے لیے زیادہ پاور
  left_hip_yaw:
    type: revolute
    axis: [0, 0, 1]
    position:
      min: -1.57  # -90 ڈگری
      max: 1.57   # +90 ڈگری
    limits:
      effort: 150.0  # Nm
      velocity: 5.0   # rad/s
    dynamics:
      damping: 0.5    # لینیئر ڈیمپنگ گ-coefficient
      friction: 2.0   # کولومب رگڑ
    safety:
      k_position: 100.0
      k_velocity: 2.0

  left_hip_roll:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -0.5
      max: 0.5
    limits:
      effort: 150.0
      velocity: 5.0
    dynamics:
      damping: 0.5
      friction: 2.0

  left_knee:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -2.0  # تقریباً سیدھا سے موڑا ہوا
      max: 0.0
    limits:
      effort: 100.0
      velocity: 8.0
    dynamics:
      damping: 0.3
      friction: 1.5
    # سیریز ایلاسٹک ایکچویشن کے لیے سٹیفنیس
    stiffness: 500.0
    damping_isa: 10.0

  left_ankle_pitch:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -0.5
      max: 0.5
    limits:
      effort: 80.0
      velocity: 6.0
    dynamics:
      damping: 0.3
      friction: 1.5

  # اپر باڈی جوائنٹس - کم پاور لیکن پریسیژن
  left_shoulder_pitch:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -3.14
      max: 3.14
    limits:
      effort: 80.0
      velocity: 6.0
    dynamics:
      damping: 0.3
      friction: 1.0

  left_elbow:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -2.5
      max: 0.0
    limits:
      effort: 50.0
      velocity: 8.0
    dynamics:
      damping: 0.2
      friction: 0.8

  neck_yaw:
    type: revolute
    axis: [0, 1, 0]
    position:
      min: -1.0
      max: 1.0
    limits:
      effort: 20.0
      velocity: 3.0
    dynamics:
      damping: 0.2
      friction: 0.5

# فٹ-گراؤنڈ رابطے کے لیے رابطہ پیرامیٹرز
contact:
  foot:
    material: rubber
    friction:
      mu: 0.8        # رگڑ کا گ-coefficient
      mu2: 0.8       # سیکنڈری رگڑ کی سمت
      slip1: 0.0     # ویلوسیٹی-منفرد slip
      slip2: 0.0
    contact:
      cfm: 0.0       # کنسترینٹ فورس مکسنگ
      erp: 0.2       # ایرر ریڈکشن پیرامیٹر
      max_contact_cfm: 1e-5
      max_contact_erp: 0.1
    bounce:
      restitution: 0.0  # چلتے ہوئے کے لیے کوئی bounce نہیں
      threshold: 100.0

# ODE فزکس سولور سیٹنگز
physics:
  solver:
    type: dantzig  # تیز LCP سولور
    iterations: 30    # LCP سولور تکرار
    sor: 1.0         # سکسیسیو اوور-ریلیکسیشن
  contacts:
    max_contacts: 20  # زیادہ سے زیادہ رابطے کے پوائنٹس
    max_contacts_per_link: 5
```

### رابطہ اور رگڑ ماڈلنگ

حقیقانہ فٹ-گراؤنڈ رابطے کے لیے احتیاط سے رابطہ پیرامیٹر ٹیوننگ کی ضرورت ہے۔ رگڑ کا ماڈل تعین کرتا ہے کہ کیا روبوٹ بغیر پھسے چل سکتا ہے، جبکہ ریسٹیوشن باؤنس سلوگ کو کنٹرول کرتا ہے۔ انڈور ماحولات کے لیے، معتدل رگڑ کے ساتھ کوئی ریسٹیوشن مستحکم چلتے ہوئے سلوگ فراہم کرتا ہے۔

```xml
<!-- contact_parameters.sdf -->
<sdf version="1.10">
  <world name="humanoid_simulation">
    <physics name="physics" type="dart">
      <dart>
        <collision_detector>bullet</collision_detector>
        <solver>
          <type>dantzig</type>
          <dtosolver_iterations>30</dtosolver_iterations>
          <sor_iterations>50</sor_iterations>
          <use_adaptive_time_stepping>true</use_adaptive_time_stepping>
        </solver>
      </dart>
    </physics>

    <!-- ہیومینوئڈ چلتے ہوئے کے لیے کسٹم رگڑ ماڈل -->
    <model name="custom_friction_floor">
      <link name="floor">
        <collision name="collision">
          <geometry>
            <plane>
              <size>50 50</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.9</mu>
                <mu2>0.9</mu2>
                <fdir1>1 0 0</fdir1>
                <slip1>0.01</slip1>
                <slip2>0.01</slip2>
              </ode>
              <torsional>
                <coefficient>0.1</coefficient>
                <patch_radius>0.1</patch_radius>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0.0</restitution_coefficient>
              <threshold_velocity>0.5</threshold_velocity>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <contact_cfm>0.0</contact_cfm>
              <contact_erp>0.2</contact_erp>
            </contact>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## 3.5 Gazebo میں سینسر سیمولیشن

سیمولیٹڈ سینسرز کو الگورتھم ڈویلپمنٹ کے لیے فزیکل سینسرز جیسے ڈیٹا پیدا کرنا چاہیئے۔ یہ حصہ کیمرا، LIDAR، اور IMU سیمولیشن کو حقیقی نoise ماڈلز اور کنفیگریشنز کے ساتھ احاطہ کرتا ہے۔

### کیمرا سیمولیشن

کیمرا سیمولیشن میں لینز ڈسٹورشن، ایکسپوزر ایفیکٹس، اور نoise ماڈلنگ شامل ہے۔ ہیومینوئڈ ایپلیکیشنز کے لیے، ہیڈ-ماؤنٹڈ کیمروں کو صحیح پوز ٹریکنگ اور روبوٹ موشن کے ساتھ سنکرونائزیشن کی ضرورت ہے۔

```xml
<!-- camera_sensor.sdf -->
<sdf version="1.10">
  <model name="head_camera">
    <link name="camera_link">
      <pose>0 0 0 0 -0.3 0</pose>

      <!-- کیمرا سینسر پلگ ان -->
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
          <remapping>camera_info:=camera/camera_info</remapping>
          <remapping>depth_image:=camera/depth_image</remapping>
        </ros>

        <!-- فزیکل سینسر سے مماثل کیمرا پیرامیٹرز -->
        <camera_name>head_camera</camera_name>
        <frame_name>camera_link</frame_name>

        <!-- امیج پراپرٹیز -->
        <image>
          <width>1280</width>
          <height>720</height>
          <format>R8G8B8</format>
        </image>

        <!-- کلپ پلینز -->
        <clip>
          <near>0.1</near>
          <far>100.0</far>
        </clip>

        <!-- لینز ڈسٹورشن (Brown-Conrady ماڈل) -->
        <distortion>
          <k1>-0.2</k1>
          <k2>0.1</k2>
          <k3>0.0</k3>
          <p1>0.0</p1>
          <p2>0.0</p2>
          <center>0.5 0.5</center>
        </distortion>

        <!-- کیمرا intrinsicس -->
        <intrinsics>
          <fx>800.0</fx>
          <fy>800.0</fy>
          <cx>640.0</cx>
          <cy>360.0</cy>
        </intrinsics>

        <!-- ریئلزم کے لیے نoise ماڈل -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </plugin>
    </link>
  </model>
</sdf>
```

### LIDAR سیمولیشن

LIDAR سینسرز نیویگیشن اور رکاوٹ سے بچنے کے لیے فاصلے کی پیمائش فراہم کرتے ہیں۔ سیمولیشن کو رینج حدود، زاویائی ریزولیشن، اور پیمائش نoise کا حساب لگانا چاہیئے۔

```xml
<!-- lidar_sensor.sdf -->
<sdf version="1.10">
  <model name="head_lidar">
    <link name="lidar_link">
      <pose>0 0 0.1 0 0 0</pose>

      <plugin name="lidar_plugin" filename="libgazebo_ros_laser.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>scan:=lidar/scan</remapping>
          <remapping>pointcloud:=lidar/points</remapping>
        </ros>

        <topic_name>/humanoid/lidar/scan</topic_name>
        <frame_name>lidar_link</frame_name>

        <!-- 2D LIDAR کنفیگریشن -->
        <laser_scan>
          <sampling>360</sampling>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
          <angle>
            <min>-3.14159</min>
            <max>3.14159</max>
            <resolution>0.0087</resolution>  <!-- 0.5 ڈگری -->
          </angle>
        </laser_scan>

        <!-- نoise پیرامیٹرز -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>

        <!-- متعدد ایکو سیٹنگز -->
        <num_echoes>2</num_echoes>
        <cadenary>true</cadenary>
      </plugin>
    </link>
  </model>
</sdf>
```

### IMU سیمولیشن

IMU سیمولیشن کو بیاس ڈریٹ، سکیل فیکٹرز، اور کراس-ایکس سینسیٹیوٹی کا حساب لگانا چاہیئے۔ یہ ناقصیات الگورتھم ڈویلپمنٹ کے لیے اہم ہیں کیونکہ وہ اسٹیٹ ایسٹیمیشن کی درستگی کو متاثر کرتی ہیں۔

```xml
<!-- imu_sensor.sdf -->
<sdf version="1.10">
  <model name="pelvis_imu">
    <link name="imu_link">
      <pose>0 0 0.02 0 0 0</pose>

      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>imu:=imu/data</remapping>
        </ros>

        <frame_name>imu_link</frame_name>
        <topic_name>/humanoid/imu/data</topic_name>

        <!-- ایکسیلیرومیٹر کنفیگریشن -->
        <acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.002</stddev>  <!-- 2 mg RMS نoise -->
            </noise>
            <bias>0.0</bias>
            <scale_factor>1.0</scale_factor>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.002</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.002</stddev>
            </noise>
            <bias>9.81</bias>  <!-- آرام پر گریویٹی -->
          </z>
        </acceleration>

        <!-- جائروسکوپ کنفیگریشن -->
        <gyroscope>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0005</stddev>  <!-- 0.05 deg/s RMS -->
            </noise>
            <bias>0.001</bias>
            <scale_factor>1.0</scale_factor>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0005</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0005</stddev>
            </noise>
          </z>
        </gyroscope>

        <!-- اپڈیٹ ریٹ -->
        <update_rate>200</update_rate>
      </plugin>
    </link>
  </model>
</sdf>
```

### مکمل سینسر انٹیگریشن

درج ذیل Python کوڈ ROS 2 کے ساتھ سینسر انٹیگریشن کا مظاہرہ کرتا ہے:

```python
#!/usr/bin/env python3
"""
ہیومینوئڈ روبوٹ سیمولیشن کے لیے سینسر انٹیگریشن

یہ ماڈیول Gazebo میں سیمولیٹڈ سینسر ڈیٹا کو پڑھنے اور قابل استعمال فارمیٹس میں تبدیل کرنے کے لیے سینسر انٹرفیس فراہم کرتا ہے۔
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
from geometry_msgs.msg import WrenchStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List
import cv2
from cv_bridge import CvBridge


@dataclass
class SensorConfig:
    """سیمولیٹڈ سینسرز کے لیے کنفیگریشن۔"""
    frame_id: str
    topic: str
    update_rate: float = 100.0
    noise_stddev: float = 0.0
    bias: float = 0.0


class HumanoidSensorInterface(Node):
    """
    ہیومینوئڈ روبوٹ سیمولیشن کے لیے یونیفائیڈ سینسر انٹرفیس۔

    سیمولیٹڈ سینسرز تک معیاری رسائی فراہم کرتا ہے بشمول
    کیمروں، LIDAR، IMU، اور فورس/ٹارک سینسرز۔
    """

    def __init__(self):
        super().__init__('humanoid_sensor_interface')

        # سینسر ڈیٹا کے لیے QoS پروفائل
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # کیمرا کنفیگریشن
        self.declare_parameter('camera.frame_id', 'head_camera_link')
        self.declare_parameter('camera.image_topic', '/humanoid/camera/image_raw')
        self.declare_parameter('camera.info_topic', '/humanoid/camera/camera_info')
        self.declare_parameter('camera.width', 1280)
        self.declare_parameter('camera.height', 720)

        # LIDAR کنفیگریشن
        self.declare_parameter('lidar.frame_id', 'lidar_link')
        self.declare_parameter('lidar.topic', '/humanoid/lidar/scan')
        self.declare_parameter('lidar.min_range', 0.1)
        self.declare_parameter('lidar.max_range', 30.0)

        # IMU کنفیگریشن
        self.declare_parameter('imu.frame_id', 'imu_link')
        self.declare_parameter('imu.topic', '/humanoid/imu/data')
        self.declare_parameter('imu.update_rate', 200.0)

        # امیج پروسیسنگ کے لیے CV بریج کی ابتدا کریں
        self.cv_bridge = CvBridge()

        # سینسر ڈیٹا ذخیرہ
        self.latest_image: Optional[Image] = None
        self.latest_lidar: Optional[LaserScan] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_ft_left: Optional[WrenchStamped] = None
        self.latest_ft_right: Optional[WrenchStamped] = None

        # سبسکرائبرز کی ابتدا کریں
        self._init_subscribers()

        self.get_logger().info("Humanoid Sensor Interface initialized")

    def _init_subscribers(self):
        """تمام سینسر سبسکرائبرز کی ابتدا کریں۔"""
        # کیمرا سبسکرائبرز
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            self.sensor_qos
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid/camera/camera_info',
            self.info_callback,
            self.sensor_qos
        )

        # LIDAR سبسکرائبرز
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid/lidar/scan',
            self.lidar_callback,
            self.sensor_qos
        )

        # IMU سبسکرائبرز
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            self.sensor_qos
        )

        # فورس/ٹارک سبسکرائبرز
        self.ft_left_sub = self.create_subscription(
            WrenchStamped,
            '/humanoid/left_foot/ft',
            self.ft_left_callback,
            self.sensor_qos
        )

        self.ft_right_sub = self.create_subscription(
            WrenchStamped,
            '/humanoid/right_foot/ft',
            self.ft_right_callback,
            self.sensor_qos
        )

    def image_callback(self, msg: Image):
        """آنے والی کیمرا امیج کو پروسیس کریں۔"""
        self.latest_image = msg

    def info_callback(self, msg: CameraInfo):
        """کیمرا کیلبریشن معلومات کو محفوظ کریں۔"""
        self.camera_info = msg

    def lidar_callback(self, msg: LaserScan):
        """آنے والی LIDAR سکین کو پروسیس کریں۔"""
        self.latest_lidar = msg

    def imu_callback(self, msg: Imu):
        """آنے والے IMU ڈیٹا کو پروسیس کریں۔"""
        self.latest_imu = msg

    def ft_left_callback(self, msg: WrenchStamped):
        """بائیں فٹ فورس/ٹارک ڈیٹا کو پروسیس کریں۔"""
        self.latest_ft_left = msg

    def ft_right_callback(self, msg: WrenchStamped):
        """دایاں فٹ فورس/ٹارک ڈیٹا کو پروسیس کریں۔"""
        self.latest_ft_right = msg

    def get_latest_image(self) -> Optional[np.ndarray]:
        """تازہ ترین امیج کو numpy array کے طور پر حاصل کریں۔"""
        if self.latest_image is None:
            return None
        try:
            return self.cv_bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return None

    def get_latest_scan(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        تازہ ترین LIDAR سکین ڈیٹا حاصل کریں۔

        Returns:
            (ranges, arrays) کا ٹپل، یا ڈیٹا نہ ہونے پر (None, None)۔
        """
        if self.latest_lidar is None:
            return None, None

        ranges = np.array(self.latest_lidar.ranges)
        angles = np.linspace(
            self.latest_lidar.angle_min,
            self.latest_lidar.angle_max,
            len(ranges)
        )

        return ranges, angles

    def get_latest_imu_reading(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        تازہ ترین IMU ریڈنگز حاصل کریں۔

        Returns:
            (linear_acceleration, angular_velocity) arrays کا ٹپل۔
        """
        if self.latest_imu is None:
            return None

        accel = np.array([
            self.latest_imu.linear_acceleration.x,
            self.latest_imu.linear_acceleration.y,
            self.latest_imu.linear_acceleration.z
        ])

        gyro = np.array([
            self.latest_imu.angular_velocity.x,
            self.latest_imu.angular_velocity.y,
            self.latest_imu.angular_velocity.z
        ])

        return accel, gyro

    def get_center_of_pressure(self) -> Tuple[float, float]:
        """
        فٹ فورس/ٹارک سینسرز سے پریشر کے مرکز کا حساب لگائیں۔

        Returns:
            فٹ فریم کوآرڈینیٹس میں (cop_x, cop_y) کا ٹپل۔
        """
        fx_l = fy_l = tz_l = 0.0
        fx_r = fy_r = tz_r = 0.0

        if self.latest_ft_left:
            fx_l = self.latest_ft_left.wrench.force.x
            fy_l = self.latest_ft_left.wrench.force.y
            tz_l = self.latest_ft_left.wrench.torque.z

        if self.latest_ft_right:
            fx_r = self.latest_ft_right.wrench.force.x
            fy_r = self.latest_ft_right.wrench.force.y
            tz_r = self.latest_ft_right.wrench.torque.z

        # مشترکہ فورسز
        fx_total = fx_l + fx_r
        fz_total = fy_l + fy_r  # Y فورس Z موoment سے مماثل ہے

        # پریشر کے مرکز کا حساب لگانا
        if abs(fx_total) > 1.0:  # کم از کم فورس تھریشولڈ
            cop_x = (tz_r - tz_l) / fx_total
        else:
            cop_x = 0.0

        if abs(fz_total) > 1.0:
            cop_y = (fx_r * 0.12 - fx_l * 0.12) / fz_total
        else:
            cop_y = 0.0

        return cop_x, cop_y


def main(args=None):
    """سینسر انٹرفیس نوڈ چلائیں۔"""
    rclpy.init(args=args)

    try:
        interface = HumanoidSensorInterface()
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

یہ کوڈ ایک مکمل ROS 2 نوڈ ہے جو سیمولیٹڈ سینسر ڈیٹا کو پڑھتا اور پروسیس کرتا ہے۔ کیمرا، LIDAR، IMU، اور فورس/ٹارک سینسرز کو سپورٹ کرتا ہے۔

## 3.6 روبوٹ ویزولائزیشن کے لیے Unity

جبکہ Gazebo فزکس سیمولیشن میں اچھا ہے، Unity ویزولائزیشن، یوزر انٹرفیس، اور فوٹوریلسٹک رینڈرنگ کے لیے بہتر رینڈرنگ کی صلاحیتیں فراہم کرتا ہے۔ ROS 2 کے ساتھ Unity کو انٹیگریٹ کرنا دونوں دنیوں کے بہترین کو فعال کرتا ہے: درست فزکس اور دلکش ویزولائزیشن۔

### ROS-Unity انٹیگریشن آرکیٹیکچر

Unity اور ROS 2 کے درمی انٹیگریشن کے لیے ایک کمیونیکیشن بریج کی ضرورت ہے۔ Unity میں ROS TCP Endpoint پیکیج ROS ٹاپکس سے ڈیٹا وصول کرتا ہے، جبکہ Unity-ROS بریج کمانڈز بھیجتا ہے اور اسٹیٹ اپڈیٹس وصول کرتا ہے۔

```csharp
// Unity کے لیے C# سکرپٹ - ROS 2 کنیکشن مینیجر
using UnityEngine;
using ROS2;
using System.Collections.Generic;

public class ROSConnectionManager : MonoBehaviour
{
    // ROS 2 نوڈ اور پبلشر ہینڈلز
    private Node ros_node;
    private Publisher<JointStateMsg> joint_state_pub;
    private Publisher<OdometryMsg> odometry_pub;
    private Subscriber<JointTrajectoryMsg> trajectory_sub;

    // جوائنٹ اسٹیٹ ذخیرہ
    private float[] joint_positions;
    private float[] joint_velocities;
    private string[] joint_names;

    // کنیکشن سیٹنگز
    private string ros_domain = "0";
    private string ros_ip = "127.0.0.1";
    private int ros_port = 9090;

    void Start()
    {
        // ROS 2 کی ابتدا کریں
        var options = new RCLdotnet().CreateNodeOptions(ros_domain);
        ros_node = new Node("unity_visualization", options);

        // پبلشرز بنائیں
        joint_state_pub = ros_node.CreatePublisher<JointStateMsg>(
            "/humanoid/joint_states"
        );
        odometry_pub = ros_node.CreatePublisher<OdometryMsg>(
            "/humanoid/odometry"
        );

        // ٹریجیکٹری کمانڈز کے لیے سبسکرائبرز بنائیں
        trajectory_sub = ros_node.CreateSubscriber<JointTrajectoryMsg>(
            "/humanoid/trajectory_command",
            TrajectoryCallback
        );

        // جوائنٹ arrays کی ابتدا کریں
        InitializeJoints();

        Debug.Log("ROS-Unity connection established");
    }

    void InitializeJoints()
    {
        // تمام ہیومینوئڈ جوائنٹ ناموں کی وضاحت کریں
        joint_names = new string[] {
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
            "left_knee", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "right_knee", "right_ankle_pitch", "right_ankle_roll",
            "waist_yaw", "waist_pitch", "waist_roll",
            "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
            "left_elbow", "left_wrist_roll", "left_wrist_pitch",
            "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
            "right_elbow", "right_wrist_roll", "right_wrist_pitch",
            "neck_pitch", "neck_yaw"
        };

        joint_positions = new float[joint_names.Length];
        joint_velocities = new float[joint_names.Length];
    }

    void Update()
    {
        // ROS کال بیکس کو پروسیس کریں
        RCLdotnet.SpinOnce(ros_node, 0.01);

        // Unity ٹرانسفارمز سے جوائنٹ پوزیشنز پڑھیں
        UpdateJointPositions();

        // جوائنٹ اسٹیٹس پبلش کریں
        PublishJointStates();
    }

    void UpdateJointPositions()
    {
        // Unity GameObjects سے جوائنٹ پوزیشنز پڑھیں
        for (int i = 0; i < joint_names.Length; i++)
        {
            var joint = GameObject.Find(joint_names[i]);
            if (joint != null)
            {
                joint_positions[i] = joint.transform.localRotation.eulerAngles.x;
                joint_velocities[i] = 0.0f;  // ڈیلٹا سے حساب لگایا جائے گا
            }
        }
    }

    void PublishJointStates()
    {
        var msg = new JointStateMsg();

        // ہیڈر ٹائم سیٹ کریں
        // نوٹ: پروڈکشن میں ROS ٹائم کا استعمال کریں گے
        msg.Header.Stamp.Sec = (int)Time.time;
        msg.Header.Stamp.Nanosec = (uint)((Time.time % 1.0) * 1e9);
        msg.Header.Frame_id = "world";

        msg.Name = joint_names;
        msg.Position = joint_positions;
        msg.Velocity = joint_velocities;
        msg.Effort = new double[joint_names.Length];  // Unity میں دستیاب نہیں

        joint_state_pub.Publish(msg);
    }

    void TrajectoryCallback(JointTrajectoryMsg msg)
    {
        // آنے والی ٹریجیکٹری کمانڈز کو پروسیس کریں
        if (msg.Points.Length > 0)
        {
            var firstPoint = msg.Points[0];
            for (int i = 0; i < joint_names.Length && i < msg.JointNames.Length; i++)
            {
                int jointIndex = System.Array.IndexOf(joint_names, msg.JointNames[i]);
                if (jointIndex >= 0)
                {
                    // Unity جوائنٹ کو ٹارگیٹ پوزیشن پر گھمائیں
                    var joint = GameObject.Find(joint_names[jointIndex]);
                    if (joint != null && firstPoint.Positions.Length > i)
                    {
                        var targetRot = Quaternion.Euler(
                            (float)(firstPoint.Positions[i] * Mathf.Rad2Deg),
                            0, 0
                        );
                        joint.transform.localRotation = targetRot;
                    }
                }
            }
        }
    }

    void OnDestroy()
    {
        // ROS وسائل کو کلین اپ کریں
        joint_state_pub.Dispose();
        odometry_pub.Dispose();
        trajectory_sub.Dispose();
        ros_node.Dispose();
    }
}
```

### ویزولائزیشن کی بہترین مشقیں

موثر ہیومینوئڈ روبوٹ ویزولائزیشن کے لیے رینڈرنگ کی معیار، کیمیرہ پوزیشننگ، اور اینیمیشن کی ہمواری پر توجہ دینی چاہیئے۔ درج ذیل گائڈلائنز پروفیشنل-کیویٹی ویزولائزیشن کو یقینی بناتی ہیں:

کیمیرہ سیٹ اپ میں متعدد دیکھنے کے زاویے شامل ہونے چاہیئے: ایک تھرڈ-پرسن فالو کیمیرہ، ہتھیلی ٹاسکس کے لیے اوور-دی-شولڈر دیکھنا، اور روبوٹ ہیڈ سے پہلے-پرسن دیکھنا۔ سینماٹوگرافک تکنیکوں کا استعمال کرتے ہوئے ہموار کیمیرہ ٹرانزیشنز دیکھنے کے تجربے کو بہتر بناتی ہیں۔

لائٹنگ کو ریئلزم اور وضاحت کے درمی توازن رکھنا چاہیئے۔ HDRI ماحولیاتی لائٹنگ قدرتی ریفلیکشنز اور ایمبیئنٹ الیومینیشن فراہم کرتی ہے۔ کی، فِل، اور رِم لائٹنگ سیٹ اپس روبوٹ کی شکل اور سلہویٹ پر زور دیتی ہیں۔ روبوٹ موشن کے جواب میں ڈائنامک لائٹنگ بصری دلچسپی شامل کرتی ہے۔

میٹریل رینڈنگ کو روبوٹ مواد کو درست طور پر پیش کرنا چاہیئے: میٹالک فینشز مناسب رفنس اور کلیئر کوٹ کے ساتھ، پلاسٹک کمپوننٹس سبسرفیس سکیٹرنگ کے ساتھ سیمی-ٹرانسلوسینسی کے لیے، اور فیبرک یا ربر موادز صحیح نارمل میپنگ کے ساتھ۔

## 3.7 سیمولیشن کا آغاز اور کنٹرول

مکمل سیمولیشن کے آغاز میں متعدد اجزاء کو کوآرڈینیٹ کرنا ضروری ہے: فزکس دنیا، روبوٹ تفصیل، سینسر پلگ انز، اور ویزولائزیشن ٹولز۔ درج ذیل ROS 2 لانچ فائل ایک جامع مثال فراہم کرتی ہے۔

```python
#!/usr/bin/env python3
"""
ہیومینوئڈ روبوٹ سیمولیشن لانچ فائل

یہ لانچ فائل Gazebo، روبوٹ تفصیل، سینسرز، اور ویزولائزیشن ٹولز سمیت مکمل سیمولیشن ماحول کو شروع کرتی ہے۔
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """مکمل سیمولیشن لانچ تفصیل تیار کریں۔"""

    # لانچ آرگیومینٹس کی وضاحت کریں
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='سیمولیشن ٹائم کا استعمال کریں'
    )

    robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='atlas_v2',
        description='روبوٹ ماڈل آئیڈینٹیفائر'
    )

    world_file = DeclareLaunchArgument(
        'world',
        default_value='humanoid_lab.sdf',
        description='Gazebo ورلڈ فائل'
    )

    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Gazebo GUI دکھائیں'
    )

    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='RViz ویزولائزیشن دکھائیں'
    )

    # پیکیج شیئرز
    pkg_share = FindPackageShare('humanoid_gazebo')
    pkg_description = FindPackageShare('humanoid_description')
    pkg_control = FindPackageShare('humanoid_control')

    # پاتھز
    urdf_file = PathJoinSubstitution([
        pkg_description, 'urdf', 'humanoid.urdf.xacro'
    ])

    world_path = PathJoinSubstitution([
        pkg_share, 'worlds', LaunchConfiguration('world')
    ])

    rviz_config = PathJoinSubstitution([
        pkg_control, 'rviz', 'humanoid_simulation.rviz'
    ])

    # لانچ تفصیل بنائیں
    ld = LaunchDescription()

    # آرگیومینٹس شامل کریں
    ld.add_action(use_sim_time)
    ld.add_action(robot_model)
    ld.add_action(world_file)
    ld.add_action(gui)
    ld.add_action(rviz)

    # Gazebo سرور
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'physics': 'dart',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    ld.add_action(gz_server)

    # Gazebo کلائنٹ
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    ld.add_action(gz_client)

    # روبوٹ اسٹیٹ پبلشر
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_file,
            'use_tf_static': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    ld.add_action(robot_state_publisher)

    # Gazebo میں روبوٹ سپاؤن کریں
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'atlas_humanoid',
            '-file', urdf_file,
            '-robot_namespace', 'humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    ld.add_action(spawn_robot)

    # جوائنٹ اسٹیٹ پبلشر (Gazebo پلگ ان)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    ld.add_action(joint_state_publisher)

    # IMU فلٹر نوڈ
    imu_filter = Node(
        package='imu_tools',
        executable='imu_filter_node',
        name='imu_filter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world_frame': 'enu',
            'orientation_stddev': 0.1,
        }],
        remappings=[
            ('imu_in', '/humanoid/imu/data'),
            ('imu_out', '/humanoid/imu/filtered')
        ],
        output='screen'
    )
    ld.add_action(imu_filter)

    # RViz ویزولائزیشن
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )
    ld.add_action(rviz_node)

    # روبوٹ کنٹرولر
    robot_controller = Node(
        package='humanoid_control',
        executable='balance_controller',
        name='balance_controller',
        parameters=[
            PathJoinSubstitution([pkg_control, 'config', 'control_params.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    ld.add_action(robot_controller)

    return ld
```

یہ کوڈ ایک مکمل ROS 2 لانچ فائل ہے جو Gazebo سیمولیشن، روبوٹ سپاؤننگ، سینسر پلگ انز، اور RViz ویزولائزیشن کو کوآرڈینیٹ کرتا ہے۔

## باب کا خلاصہ

اس باب نے ہیومینوئڈ روبوٹکس ڈویلپمنٹ کے لیے روبوٹ سیمولیشن کے ضروری پہلوؤں کا احاطہ کیا:

1. **سیمولیشن کی اہمیت**: روبوٹ سیمولیشن محفوظ الگورتھم ڈویلپمنٹ، لایت-اثر ٹیسٹنگ، اور فزیکل ہارڈویئر کے ساتھ ناممکن منظرناموں کو فعال کرتا ہے۔

2. **Gazebo کنفیگریشن**: مناسب فزکس انجنز، تھریڈ کنفیگریشنز، اور وسائل مختص کے ساتھ سیمولیشن ماحول کی تشکیل۔

3. **روبوٹ تفصیل فارمیٹس**: URDF کائنیٹکس اور بنیادی خصوصیات کے لیے معیاری روبوٹ تفصیل فراہم کرتا ہے، جبکہ SDF زیادہ جامع ماڈلنگ صلاحیتیں پیش کرتا ہے۔

4. **فزکس سیمولیشن**: جوڑ ڈائنامکس، رابطہ ماڈلنگ، اور رگر کنفیگریشن سیمولیشن میں حقیقانہ ہیومینوئڈ سلوگ کے لیے اہم ہیں۔

5. **سینسر سیمولیشن**: کیمرا، LIDAR، اور IMU سیمولیشن حقیقی ن-noise ماڈلز کے ساتھ الگورتھم کو فزیکر ڈیپلائمنٹ کے لیے تیار کرتی ہے۔

6. **Unity ویزولائزیشن**: ہائی-فیڈیلیٹی رینڈرنگ فزکس سیمولیشن کو ویزولائزیشن اور کمیونیکیشن مقاصد کے لیے پورا کرتی ہے۔

7. **آغاز اور کنٹرول**: مکمل لانچ فائلز قابل تکرار ترقیاتی ماحول کے لیے تمام سیمولیشن اجزاء کو کوآرڈینیٹ کرتی ہیں۔

### اہم تصورات

- **URDF/SDF**: روبوٹ تفصیل فارمیٹس جو کائنیٹکس، بصری، اور رابطہ خصوصیات فراہم کرتے ہیں
- **فزکس انجنز**: DART، ODE، اور Bullet مختلف درستگی/کارکردگی ٹریڈ آفس فراہم کرتے ہیں
- **سینسر پلگ انز**: Gazebo پلگ انز کانفیگریبل ن-noise کے ساتھ سینسر سلوگ کی سیمولیشن کرتے ہیں
- **ROS انٹیگریشن**: ROS 2 اور سیمولیشن ماحول کے درمی سیسلسیس انٹیگریشن
- **ویزولائزیشن**: Unity فوٹوریلسٹک رینڈرنگ فراہم کرتا ہے جو Gazebo کا پورا کرتا ہے

### ہارڈویئر ضروریات حوالہ

| جزو | کم از کم | تجویز کردہ |
|------|----------|-------------|
| CPU | 4-core @ 2.5 GHz | 8-core @ 4.0 GHz |
| RAM | 8 GB | 32 GB DDR4 |
| GPU | انٹیگریٹڈ | NVIDIA RTX 3060+ |
| اسٹوریج | 64 GB SSD | 256 GB NVMe SSD |
| فزکس ریٹ | 500 Hz | 1000 Hz |

### مزید پڑھنا

- Gazebo دستاویزات: https://gazebosim.org/docs
- ROS 2 کنٹرول: https://control.ros.org/
- Unity روبوٹکس ہب: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- DARwin-OP2 دستاویزات: https://github.com/ROBOTIS-GIT/ROBOTIS-Documents

### اگلا باب

باب 4 GPU-ایکسیلریٹڈ سیمولیشن کے لیے **Isaac Sim** کی جانچ کرتا ہے، جو پرسیپشن ماڈلز کی تربیت اور رینفورسمینٹ لرننگ کے لیے ہائی-کارکردگی سیمولیشن کو فعال کرتا ہے۔

---

**حصہ 2: ROS 2 بنیادیات** | [ہفتہ 3-5 کا جائزہ](part-2-ros2/02a-week-3-5-overview) | [حصہ 3: سیمولیشن](part-3-simulation/gazebo-unity-simulation) | [حصہ 4: NVIDIA Isaac](part-4-isaac/nvidia-isaac-platform)
