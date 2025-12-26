---
title: " Weeks 11-12: Humanoid Robot Development"
sidebar_position: 6
---

# حصہ 5: Humanoid Robot Development (انسان نما روبوٹ ڈویلپمنٹ) - ہفتہ 11-12 کا جائزہ

یہ حصہ Humanoid robot (انسان نما روبوٹ) ڈویلپمنٹ میں مہارت حاصل کرنے کے لیے ایک منظم سیکھنے کا منصوبہ فراہم کرتا ہے۔ دو ہفتوں کا نصاب Kinematics (کینیمیٹکس)، Bipedal locomotion (دو پاؤں والی حرکت)، Manipulation (ہیرا پھیری)، اور قدرتی Human-robot interaction (انسان روبوٹ تعامل) میں عملی مہارتیں بناتا ہے۔

## جائزہ

Humanoid robot development (انسان نما روبوٹ ڈویلپمنٹ) اس کورس میں سیکھی گئی مہارتوں کا اکھاڑہ ہے، جو Perception (ادراک)، Control (کنٹرول)، اور Planning (منصوبہ بندی) کو Anthropomorphic robotic systems (انسان نما روبوٹک سسٹمز) میں ضم کرتا ہے۔ یہ حصہ Bipedal locomotion (دو پاؤں والی حرکت)، Dextrous manipulation (ماہرانہ ہیرا پھیری)، اور قدرتی Human-robot interaction (انسان روبوٹ تعامل) کی منفرد چیلنجز پر مرکوز ہے جو Humanoid robots (انسان نما روبوٹس) کو دیگر روبوٹک پلیٹ فارمز سے الگ کرتے ہیں۔

Week 12 کے آخر تک، آپ ایک مکمل Humanoid controller (انسان نما کنٹرولر) لاگو کر چکے ہوں گے جو مستقل Walking (چلنا) اور بنیادی Manipulation (ہیرا پھیری) کے کاموں کے قابل ہو، اور سمجھتے ہوں گے کہ یہ سسٹمز Unitree H1 جیسے حقیقی روبوٹ پلیٹ فارمز میں کیسے ضم ہوتے ہیں۔

## Week 11: Kinematics (کینیمیٹکس) اور Locomotion (حرکت)

### سیکھنے کے مقاصد

Week 11 کے آخر تک، آپ کے پاس ہو گا:

- Humanoid kinematic chains (انسان نما کینیمیٹک چینز) کے لیے Forward kinematics (آگے کی کینیمیٹکس) کو سمجھنا اور لاگو کرنا
- Multi-DOF legs (ملٹی-ڈی او ایف لیگز) کے لیے Analytical اور Numerical methods (عددی طریقے) kullanarak Inverse kinematics (الٹی کینیمیٹکس) حل کرنا
- مستقل Bipedal walking (دو پاؤں والا چلنا) کے لیے ZMP-based balance control (زیڈ ایم پی پر مبنی بیلنس کنٹرول) لاگو کرنا
- صحیح Timing اور Foot placement (پاؤں رکھنے کے ساتھ Walking trajectories (واک کرنے کی ٹریکٹریز) پیدا کرنا
- Humanoid robot dynamics (انسان نما روبوٹ حرکیات) اور Stability margins (استحکام کی حدیں) کا تجزیہ کرنا
- Kinematics solvers (کینیمیٹکس سلورز) کو ROS 2 control systems (آر او ایس 2 کنٹرول سسٹمز) کے ساتھ ضم کرنا

### اہم مضامین

#### 1. Humanoid Kinematic Structures (انسان نما کینیمیٹکس ساختیں)

Humanoid robots (انسان نما روبوٹس) Industrial manipulators (صنعتی ہیرا پھیری کرنے والے) کے مقابلے میں منفرد Kinematic challenges (کینیمیٹکس چیلنجز) پیش کرتے ہیں۔ Kinematic chain structure (کینیمیٹکس چین ساخت) کو Bipedal base (دو پاؤں والی بنیاد) کو مدنظر رکھنا چاہیے جو Locomotion (حرکت) کے دوران حرکت کرتی ہے، اور Arms اور Legs کی Bilateral symmetry (دو طرفہ مماثلت)۔ Forward اور Inverse kinematics algorithms (کینیمیٹکس الگورتھمز) لاگو کرنے کے لیے اس Hierarchical structure (سلسلہ وار ساخت) کو سمجھنا ضروری ہے۔

معیاری Humanoid kinematic structure (انسان نما کینیمیٹکس ساخت) میں شامل ہے:

- **Pelvis/Torso (پیلوس/ٹارسو)**: پورے جسم کے لیے مرکزی Reference frame (ریفرینس فریم)
- **Legs (لیگز) (3 DOF ہر ایک)**: Hip roll/pitch, knee pitch, ankle pitch/roll
- **Arms (آرمز) (3-4 DOF ہر ایک)**: Shoulder pitch/roll, elbow, wrist
- **Head (ہیڈ) (2-3 DOF)**: Neck pitch/yaw, eye vergence

یہ ساخت Human anatomy (انسانی جسمانی ساخت) کی عکاسی کرتی ہے اور Human-designed environments (انسانوں کے بنائے ہوئے ماحول) میں کام کرنے کو فعال کرتی ہے۔

#### 2. Forward Kinematics (آگے کی کینیمیٹکس)

Forward kinematics (آگے کی کینیمیٹکس) Joint angles (مفصل زاویے) دیے گئے ہر Link کی Position اور Orientation کا حساب لگاتی ہے۔ Humanoid robots (انسان نما روبوٹس) کے لیے یہ خاصی اہم ہے:

- Manipulation (ہیرا پھیری) کے دوران End-effector position feedback (اینڈ ایفیکٹر پوزیشن فیڈبیک)
- Balance control (بیلنس کنٹرول) کے لیے Foot position estimation (پاؤں کی پوزیشن کا تخمینہ)
- Self-collision detection (خود تصادم کا پتہ لگانا) اور avoidance
- Visualization اور state reporting

Pelvis سے ہر End-effector تک Transformation (ٹرانسفارمیشن) ہر Joint کے ذریعے Homogeneous transformations (ہوموجینیئس ٹرانسفارمیشنز) کے چین کی پیروی کرتی ہے۔ Denavit-Hartenberg convention (ڈینیویٹ ہارٹنبرگ کنونشن) ان ٹرانسفارمیشنز کا حساب لگانے کے لیے ایک معیاری طریقہ فراہم کرتی ہے۔

#### 3. Legs کے لیے Inverse Kinematics (الٹی کینیمیٹکس)

Humanoid legs (انسان نما لیگز) کے لیے Inverse kinematics (الٹی کینیمیٹکس) Industrial arms (صنعتی بازو) سے زیادہ پیچیدہ ہے Balance کی ضرورت کی وجہ سے۔ اہم Considerations میں شامل ہے:

- **Redundancy Resolution (ردعمل حل)**: Obstacle avoidance یا Energy efficiency کے لیے اضافی DOF استعمال کرنا
- **Singularity Handling (منفردیت ہینڈلنگ)**: ایسی Configurations سے بچنا جہاں Jacobian منفرد ہو جاتا ہے
- **Reachability (پہنچ)**: یہ تعین کرنا کہ کیا Target position قابل حصول ہے
- **Multiple Solutions (متعدد حل)**: ممکن Configurations میں سے سب سے مناسب حل تلاش کرنا

3-DOF legs کے لیے Analytical solutions (تحلیلی حل) Frontal اور Sagittal plane components میں مسئلے کو توڑ کر موجود ہیں۔ یہ decoupling (علیحدگی) حل کو آسان بناتی ہے اور Real-time control (اصل وقت کنٹرول) کے لیے Computational efficiency (حسابی کارکردگی) بہتر کرتی ہے۔

#### 4. Bipedal Walking Control (دو پاؤں والا چلنا کنٹرول)

Humanoid robots (انسان نما روبوٹس) کے لیے Walking control (چلنا کنٹرول) متعدد Subsystems کو ملا کر کرتا ہے:

- **Footstep Planning (فٹسٹیپ پلاننگ)**: یہ طے کرنا کہ ہر پاؤں کہاں رکھنا ہے
- **ZMP Computation (زیڈ ایم پی کمپیوٹیشن)**: پورے Gait cycle کے دوران استحکام یقینی بنانا
- **Trajectory Generation (ٹریکٹری جنریشن)**: Smooth reference trajectories (ہموار حوالہ ٹریکٹریز) بنانا
- **Balance Control (بیلنس کنٹرول)**: Disturbances اور modeling errors کا معاوضہ کرنا
- **Joint Control (مفصل کنٹرول)**: منصوبہ بند ٹریکٹریز پر عمل درآمد کرنا

Zero Moment Point (ZMP) استحکام کی بنیاد ہے۔ ZMP وہ نقطہ ہے زمین پر جہاں Gravitational اور inertial forces (جاذبیت اور جمودی قوتوں) کا کل moment صفر کے برابر ہوتا ہے۔ مستقل چلنا کے لیے، ZMP کو Ground کے ساتھ رابطے میں موجود پاؤں سے بننے والے Support polygon (سپورٹ پولی گون) کے اندر رہنا چاہیے۔

#### 5. Dynamic Walking Considerations (حرکتی چلنا پر غور)

Static walking (جہاں روبوٹ Double support میں رکتا ہے) آسان ہے لیکن دھیما ہے۔ Dynamic walking (حرکتی چلنا) پورے Gait cycle کے دوران آگے کی حرکت برقرار رکھتا ہے اور درکار ہے:

- Heel strike اور toe-off کا صحیح Timing
- Swing foot trajectory planning (سوئنگ فٹ ٹریکٹری پلاننگ)
- Momentum compensation کے لیے Upper body motion
- Landing impact absorption (لینڈنگ اثر جذب)

جدید approaches Model-predictive control (ماڈل پریڈکٹو کنٹرول) استعمال کرتے ہیں متعدد Steps آگے سے منصوبہ بندی کرنے کے لیے، استحکام، رفتار، اور Energy efficiency کو بہتر بنانے کے لیے۔

### حاصل کرنے کے اہم تصورات

- **Kinematic Chains (کینیمیٹکس چینز)**: Parent-child relationships کے ساتھ Hierarchical joint structures
- **Homogeneous Transformations (ہوموجینیئس ٹرانسفارمیشنز)**: Position اور Orientation کی نمائندگی کرنے والی 4x4 matrices
- **DH Parameters**: Robot geometry کی معیاری تفصیل
- **ZMP (Zero Moment Point)**: Bipedal walking (دو پاؤں والا چلنا) کے لیے استحکام کا معیار
- **Gait Cycle**: چلنا کے phases (stance, swing, double support)
- **Support Polygon**: Ground کے ساتھ contact points کا محدب ڈھیلا
- **Jacobian**: Joint اور task space کے درمیان Velocity transformation

### مشق کے کام

1. **مشق 1: Forward Kinematics Implementation (آگے کی کینیمیٹکس لاگو کرنا)** (3 گھنٹے)
   - DH parameters کے ساتھ Humanoid leg kinematic chain (انسان نما لیگ کینیمیٹکس چین) define کرنا
   - Foot position کے لیے Forward kinematics function (آگے کی کینیمیٹکس فنکشن) لاگو کرنا
   - Velocity mapping کے لیے Jacobian matrix کمپیوٹ کرنا
   - مختلف Joint angle combinations کے ساتھ test کرنا
   - RViz میں نتائج کا VISUALIZATION کرنا

2. **مشق 2: Analytical Inverse Kinematics (تحلیلی الٹی کینیمیٹکس)** (4 گھنٹے)
   - 3-DOF leg کے لیے Analytical solution (تحلیلی حل) derive کرنا
   - Reachability checking کے ساتھ IK solver لاگو کرنا
   - متعدد solution cases کو handle کرنا
   - Edge cases اور singularities test کرنا
   - Numerical solutions سے compare کرنا

3. **مشق 3: ZMP Balance Controller (زیڈ ایم پی بیلنس کنٹرولر)** (4 گھنٹے)
   - Joint torques سے ZMP computation لاگو کرنا
   - ZMP error استعمال کرنے والا Balance controller ڈیزائن کرنا
   - استحکام کے لیے controller gains tune کرنا
   - Push recovery responses test کرنا
   - Stability margins measure کرنا

4. **مشق 4: Walking Trajectory Generator (واک ٹریکٹری جنریٹر)** (4 گھنٹے)
   - Velocity command سے Footstep plan generate کرنا
   - (parabolic lift) Swing foot trajectory لاگو کرنا
   - Double support transition logic بنانا
   - Joint reference trajectories generate کرنا
   - Balance controller کے ساتھ integrate کرنا

5. **مشق 5: Complete Walking Controller (مکمل واک کنٹرولر)** (5 گھنٹے)
   - تمام components کو مکمل controller میں ضم کرنا
   - Gait state machine لاگو کرنا
   - Realistic parameters کے ساتھ simulation میں test کرنا
   - Smooth transitions کے لیے tune کرنا
   - Walking performance metrics measure کرنا

### تخمینہ وقت کا التزام

| سرگرمی | گھنٹے |
|--------|-------|
| Reading اور tutorials | 8 |
| مشق 1: Forward Kinematics | 3 |
| مشق 2: Inverse Kinematics | 4 |
| مشق 3: ZMP Balance | 4 |
| مشق 4: Trajectory Generation | 4 |
| مشق 5: Complete Controller | 5 |
| Troubleshooting اور review | 4 |
| **کل** | **32 گھنٹے** |

## Week 12: Manipulation (ہیرا پھیری) اور Human-Robot Interaction (انسان روبوٹ تعامل)

### سیکھنے کے مقاصد

Week 12 کے آخر تک، آپ کے پاس ہو گا:

- Humanoid hands (انسان نما ہاتھوں) کے لیے Grasp planning algorithms (گراسپ پلاننگ الگورتھمز) ڈیزائن اور لاگو کرنے کی صلاحیت
- مختلف Grasp types (گراسپ اقسام) اور ان کے مناسب Applications کو سمجھنا
- Multi-modal human-robot interaction systems (ملٹی موڈ انسان روبوٹ تعامل سسٹمز) لاگو کرنا
- قدرتی Gaze اور Attention behaviors (نظر اور توجہی رویے) ڈیزائن کرنا
- Speech، gesture، اور body language (تقریر، اشارہ، اور جسمانی زبان) کو cohesive HRI کے لیے ضم کرنا
- User studies (صارفین کے مطالعے) کے ذریعے HRI quality (HRI معیار) کا جائزہ لینا

### اہم مضامین

#### 1. Robot Hands اور Manipulation (روبوٹ ہاتھ اور ہیرا پھیری)

Humanoid manipulation (انسان نما ہیرا پھیری) Human hands (انسانی ہاتھوں) کی Versatility (تعدد) کو دہرانے کی کوشش کرتی ہے۔ مخصوص کاموں کے لیے optimize کیے گئے Industrial grippers (صنعتی گریپرز) کے برعکس، Humanoid hands (انسان نما ہاتھ) کو متنوع اشیاء handle کرنے، Grasp types (گراسپ اقسام) میں adapt کرنے، اور Unstructured environments (غیر ساختہ ماحول) میں manipulation (ہیرا پھیری) کرنے کے قابل ہونا چاہیے۔

**Grasp Classification (گراسپ درجہ بندی)**:

- **Power Grasps (پاور گراسپ)**: High-force tasks کے لیے چیزیں گول میں لپیٹنا۔ مثالیں میں Cylindrical grip (ہتھیلی سے پکڑنا - ہتھوڑا پکڑنے کے لیے)، Spherical grip (گول پکڑنا - گیند پکڑنے کے لیے)، اور Ulnar grip (لکڑنے والی گرفت - چھری جیسے ٹول پکڑنے کے لیے) شامل ہیں۔
- **Precision Grasps (پریسیزن گراسپ)**: Delicate manipulation کے لیے Fingertips استعمال کرنا۔ مثالیں میں Pinch grip (انگوٹھے اور انگشت سے پکڑنا - چھوٹی چیز پکڑنے کے لیے)، Lateral pinch (-side pinch- چابی پکڑنے کے لیے)، اور Tripod grip (تین انگوں سے پکڑنا - قلم پکڑنے کے لیے) شامل ہیں۔
- **Intermediate Grasps (انٹرمیڈیٹ گراسپ)**: Force اور precision کا balance روزمرہ کاموں جیسے ٹول کے استعمال، کھانے کی چیز کی ہیرا پھیری، اور اشیاء کی منتقلی کے لیے۔

**Grasp Quality Metrics (گراسپ معیار میٹرکس)**:

- **Force Closure (فورس کلوزر)**: Arbitrary disturbance forces کا مزاحمت کرنے کی صلاحیت
- **Form Closure (فارم کلوزر)**: Motion کو روکنے والی Geometric constraints
- **Grasp Dexterity (گراسپ ڈیکسٹریٹی)**: چیز کو ہاتھ میں دوبارہ پوزیشن دینے کی صلاحیت
- **Grasp Stability (گراسپ سٹیبلٹی)**: مخصوص expected disturbances کی مزاحمت

#### 2. Grasp Planning (گراسپ پلاننگ)

Grasp planning (گراسپ پلاننگ) یہ تعین کرتی ہے کہ چیز کو پکڑنے کے لیے ہاتھ کو کس طرح posicion کرنا ہے اور forces کیسے لگانی ہیں۔ Planning process میں شامل ہے:

1. **Object Perception (چیز کا ادراک)**: Object shape، size، اور weight کا تخمینہ لگانا
2. **Grasp Selection (گراسپ کا انتخاب)**: کام کے لیے مناسب grasp type چنا
3. **Hand Configuration (ہاتھ کی ترتیب)**: مطلوبہ grasp کے لیے Joint angles کا حساب لگانا
4. **Approach Planning (قریب آنے کی منصوبہ بندی)**: موجودہ pose سے grasp pose تک Path
5. **Force Planning (قوت کی منصوبہ بندی)**: مناسب grip force لگانا

Learning-based approaches (سیکھنے پر مبنی طریقے) Visual input سے grasp success کی prediction کر سکتے ہیں، جبکہ Analytical methods (تحلیلی طریقے) Candidate grasps پر grasp quality metrics کو optimize کرتے ہیں۔

#### 3. Natural Human-Robot Interaction (قدرتی انسان روبوٹ تعامل)

Humanoid robots (انسان نما روبوٹس) کے لیے Human-robot interaction (انسان روبوٹ تعامل) کو Anthropomorphic systems (انسان نما سسٹمز) کے ساتھ interaction کرتے وقت انسانوں کی منفرد expectations کو مدنظر رکھنا چاہیے۔ اہم اصولوں میں شامل ہے:

**Social Signals (سماجی اشارات)**:

- **Gaze (نظر)**: Eye contact اور attention direction
- **Gestures (اشارات)**: Deictic (نقشہ کرنا)، emblematic (تھمبس اپ)، beat gestures
- **Posture (وضع)**: Body orientation اور stance
- **Facial Expressions (چہرے کے مماثل)**: Emotional state communication
- **Proxemics (فاصلے کا علم)**: مناسب interaction distance

**Interaction Fluency (تعامل کی روانی)**:

- مکالمے میں Turn-taking
- Response timing اور natural pacing
- Error recovery اور repair
- User preferences کو adapt کرنا

#### 4. Multimodal Interaction Architecture (ملٹی موڈ تعامل فن تعمیر)

ایک مکمل HRI system (HRI سسٹم) متعدد Communication channels (مواصلاتی چینلز) کو ضم کرتا ہے:

- **Speech Recognition اور Synthesis**: Spoken language input اور output
- **Gesture Recognition (اشارہ recognition)**: User gestures کو سمجھنا
- **Gaze Tracking (نظر ٹریکنگ)**: یہ detect کرنا کہ صارفین کہاں دیکھ رہے ہیں
- **Face Detection اور Recognition**: Users کی identify اور track کرنا
- **Body Pose Estimation (جسم کی pose کا تخمینہ)**: User posture اور gestures کو سمجھنا

سسٹم کو User intent کو سمجھنے اور مناسب responses generate کرنے کے لیے ان modalities کو fuse کرنا چاہیے۔

#### 5. Gaze اور Attention Modeling (نظر اور توجہ ماڈلنگ)

Gaze behavior (نظر کا رویہ) قدرتی interaction کے لیے بنیادی ہے۔ انسان Eye contact کا استعمال communication channels (مواصلاتی چینلز) قائم کرنے، Attention (توجہ) signal کرنے، اور Emotional state ( جذباتی حالت) convey کرنے کے لیے کرتے ہیں۔ Humanoid robot (انسان نما روبوٹ) کا gaze behavior (نظر کا رویہ) کرنا چاہیے:

- مکالمے کے دوران مناسب Eye contact
- نئے لوگوں کے آنے پر Attention shift کرنا
- User attention کے focus کی پیروی کرنا
- Internal states signal کرنا (thinking, processing, understanding)

Attention modeling (توجہ ماڈلنگ) یہ compute کرتا ہے کہ روبوٹ کو کہاں دیکھنا چاہیے:

- Detected people اور ان کا engagement level
- حالیہ speech activity اور turn-taking
- Task context اور goals
- Social norms اور politeness

#### 6. Body Language اور Gestures (جسمانی زبان اور اشارات)

Body language (جسمانی زبان) speech کو supplement کرتی ہے اور اضافی Communication channels (مواصلاتی چینلز) فراہم کرتی ہے:

- **Nodding/Shaking (سر ہلانا)**: Agreement/disagreement signals
- **Head Tilts (سر جھکانا)**: Interest، confusion، یا thoughtfulness
- **Arm Gestures (بازو کے اشارات)**: تقریر پر زور دینا، سمت indicate کرنا
- **Postural Mirroring (وضع کی عکاسی)**: Alignment کے ذریعے rapport بنانا

Implementation کے لیے natural، smooth movements پیدا کرنے کے لیے متعدد joints کا coordinated control (ہم آہنگ کنٹرول) درکار ہے۔

### حاصل کرنے کے اہم تصورات

- **Grasp Types (گراسپ اقسام)**: Power, precision، اور intermediate grasps
- **Grasp Quality (گراسپ معیار)**: Force closure, form closure, manipulability
- **Grasp Planning (گراسپ پلاننگ)**: Object analysis, candidate generation, selection
- **Social Signals (سماجی اشارات)**: Gaze, gestures, posture, proxemics
- **Attention Model (توجہ ماڈل)**: Interaction focus کا حساب لگانا
- **Multimodal Fusion (ملٹی موڈ فیوژن)**: Speech, gesture، اور gaze کو ملا کر سمجھنا
- **Body Language (جسمانی زبان)**: Expression کے لیے coordinated movement

### مشق کے کام

1. **مشق 1: Grasp Type Analysis (گراسپ قسم کا تجزیہ)** (2 گھنٹے)
   - قسم اور کام کی مناسب بود کے لحاظ سے grasps کو classify کرنا
   - مختلف اشیاء کے لیے force requirements کا تجزیہ کرنا
   - Grasp selection decision tree ڈیزائن کرنا
   - Grasp types کے درمیان trade-offs کا جائزہ لینا
   - مثالوں کے ساتھ نتائج document کرنا

2. **مشق 2: Grasp Planning Implementation (گراسپ پلاننگ لاگو کرنا)** (4 گھنٹے)
   - Grasp candidate generation لاگو کرنا
   - Grasp quality metrics کمپیوٹ کرنا
   - Task constraints کے لیے best grasp select کرنا
   - Hand configuration generate کرنا
   - Various object types کے ساتھ test کرنا

3. **مشق 3: HRI Framework Setup (HRI فریم ورک سیٹ اپ)** (3 گھنٹے)
   - Person tracking system set up کرنا
   - Attention model لاگو کرنا
   - Gaze behavior generator بنانا
   - Interaction state machine ڈیزائن کرنا
   - Multiple users کے ساتھ test کرنا

4. **مشق 4: Multimodal Interaction (ملٹی موڈ تعامل)** (4 گھنٹے)
   - Speech recognition integrate کرنا
   - Gesture recognition add کرنا
   - Multimodal intent understanding لاگو کرنا
   - Coordinated responses generate کرنا
   - Interaction flow test کرنا

5. **مشق 5: Complete HRI Integration (مکمل HRI انٹیگریشن)** (5 گھنٹے)
   - Manipulation اور interaction کو combine کرنا
   - Task-oriented dialogue لاگو کرنا
   - Body language animations add کرنا
   - User evaluation کرنا
   - Feedback کے مطابق refine کرنا

### تخمینہ وقت کا التزام

| سرگرمی | گھنٹے |
|--------|-------|
| Reading اور tutorials | 6 |
| مشق 1: Grasp Analysis | 2 |
| مشق 2: Grasp Planning | 4 |
| مشق 3: HRI Framework | 3 |
| مشق 4: Multimodal Interaction | 4 |
| مشق 5: Complete Integration | 5 |
| Troubleshooting اور review | 4 |
| **کل** | **28 گھنٹے** |

## مشترکہ ہفتہ کا شیڈول

### Week 11: Kinematics (کینیمیٹکس) اور Locomotion (حرکت)

| دن | صبح (9-12) | دوپہر (1-5) | شام (7-9) |
|-----|------------|-------------|----------|
| پیر | Kinematic theory reading | مشق 1: Forward Kinematics | کوڈ review |
| منگل | Jacobian derivation practice | مشق 2: Inverse Kinematics | پیپر reading |
| بدھ | ZMP theory study | مشق 3: ZMP Balance | Lab session |
| جمعرات | Gait analysis | مشق 4: Trajectory Generation | Documentation |
| جمعہ | Controller design | مشق 5: Complete Controller | Debugging |
| ہفتہ | Integration testing | Performance optimization | آرام |
| اتوار | Review اور reflection | Week 12 preview | آرام |

### Week 12: Manipulation (ہیرا پھیری) اور HRI

| دن | صبح (9-12) | دوپہر (1-5) | شام (7-9) |
|-----|------------|-------------|----------|
| پیر | Manipulation theory | مشق 1: Grasp Analysis | User studies review |
| منگل | Grasp planning algorithms | مشق 2: Grasp Planning | Code review |
| بدھ | HRI principles | مشق 3: HRI Framework | Reading |
| جمعرات | Multimodal systems | مشق 4: Multimodal | Documentation |
| جمعہ | Body language design | مشق 5: Complete Integration | Testing |
| ہفتہ | Final integration | User evaluation | آرام |
| اتوار | Project completion | Documentation | آرام |

## ہارڈویئر کی ضروریات

### Simulation Environment (سیمولیشن ماحول)

| Component | کم از کم | تجویز کردہ |
|-----------|----------|-----------|
| CPU | 4-core @ 2.5 GHz | 8-core @ 4.0 GHz |
| RAM | 8 GB | 16 GB |
| GPU | Intel HD 4000 | NVIDIA GTX 1060+ |
| Storage | 50 GB free | 100 GB SSD |
| ROS 2 | Humble | Humble |

### Physical Robot (Unitree H1)

| Component | Specification |
|-----------|--------------|
| Total DOF | 19 |
| Control Rate | 200 Hz |
| Communication | Ethernet, CAN |
| Power | 450W peak |
| Operating Time | ~2 hours walking |

## Deliverables (دستاویزات)

### Week 11 Deliverables

1. **Forward Kinematics Module (آگے کی کینیمیٹکس ماڈیول)**: ٹیسٹوں کے ساتھ مکمل Python implementation
2. **Inverse Kinematics Solver (الٹی کینیمیٹکس سلور)**: Reachability checking کے ساتھ Analytical solver
3. **ZMP Balance Controller (زیڈ ایم پی بیلنس کنٹرولر)**: Stability analysis کے ساتھ کام کرنے والا controller
4. **Walking Trajectory Generator (واک ٹریکٹری جنریٹر)**: Footstep اور joint trajectory generation
5. **Complete Walking Controller (مکمل واک کنٹرولر)**: State machine کے ساتھ integrated controller
6. **Documentation (دستاویزات)**: Analysis اور performance metrics کے ساتھ Technical report

### Week 12 Deliverables

1. **Grasp Planning System (گراسپ پلاننگ سسٹم)**: Quality evaluation کے ساتھ مکمل implementation
2. **HRI Framework**: Person tracking, attention model, gaze system
3. **Multimodal Integration (ملٹی موڈ انٹیگریشن)**: Speech, gesture، اور gaze fusion
4. **Body Language Generator (جسمانی زبان جنریٹر)**: Coordinated gesture animations
5. **User Evaluation (صارف جائزہ)**: Interaction testing سے نتائج
6. **Documentation (دستاویزات)**: System architecture اور API documentation

## Assessment Criteria (تشخیص معیار)

### Week 11 Assessment

| معیار | وزن | تفصیل |
|--------|------|-------|
| Forward Kinematics | 15% | درست transformation computation |
| Inverse Kinematics | 20% | درست، مؤثر IK solutions |
| Balance Controller | 25% | مستقل ZMP tracking, disturbance rejection |
| Trajectory Generation | 20% | ہموار، صحیح timing والی trajectories |
| Integration | 15% | مکمل کام کرنے والا سسٹم |
| Documentation | 5% | واضح کوڈ اور رپورٹیں |

### Week 12 Assessment

| معیار | وزن | تفصیل |
|--------|------|-------|
| Grasp Planning | 20% | Quality metrics, diverse grasp support |
| HRI Framework | 25% | Attention model, gaze behavior |
| Multimodal Fusion | 20% | Robust intent understanding |
| Body Language | 15% | Natural, coordinated animations |
| User Evaluation | 15% | معنی خیز نتائج اور تجزیہ |
| Documentation | 5% | مکمل system documentation |

## Troubleshooting Common Issues (عام مسائل کا ازالہ)

### Kinematics Issues (کینیمیٹکس مسائل)

**Problem**: IK solver دھیمے سے converge ہوتا ہے یا converge ہونے میں ناکام ہے
- Joint limit constraints چیک کریں
- Jacobian computation verify کریں
- Solution initialization from previous pose add کریں
- Matrix operations کی numerical stability consider کریں

**Problem**: چلتے ہوئے foot positions drift ہو جاتے ہیں
- Forward kinematics میں integration errors accumulate ہو رہے ہیں
- Closed-loop position feedback use کریں
- Periodic recalibration لاگو کریں
- Numerical precision issues چیک کریں

### Walking Issues (چلنا کے مسائل)

**Problem**: Gait transition کے دوران روبوٹ گر جاتا ہے
- Double support میں insufficient ZMP margin
- Transition timing parameters چیک کریں
- Joint velocity limits verify کریں
- Balance controller gains adjust کریں

**Problem**: Swing کے دوران foot scuffing ہو رہی ہے
- Swing height too low
- Trajectory smoothing insufficient
- Heel-toe transition کی timing چیک کریں
- Ground clearance parameters adjust کریں

### HRI Issues

**Problem**: Gaze قدرتی یا jittery لگ رہا ہے
- Smoothing filter too aggressive
- Missing intermediate waypoints
- Transition timing too fast
- Proper acceleration limiting لاگو کریں

**Problem**: Multimodal fusion غلط intent produce کرتا ہے
- Confidence thresholds misaligned
- Modalities conflict without proper weighting
- Temporal alignment incorrect
- Fusion algorithm implementation review کریں

## Unitree H1 کے ساتھ انٹیگریشن

### Hardware Interface (ہارڈویئر انٹرفیس)

Unitree H1 robot joint control کے لیے ایک high-level API فراہم کرتا ہے:

```bash
# روبوٹ کو فعال کریں
ros2 launch unitree_h1 bringup.launch.py

# joint status چیک کریں
ros2 topic echo /joint_states

# position commands بھیجیں
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "..."
```

### Simulation Integration (سیمولیشن انٹیگریشن)

ہارڈویئر deployment سے پہلے simulation testing کے لیے:

```python
# H1 robot model لوڈ کریں
from unitree_h1_interface import H1Robot

robot = H1Robot(use_sim=True)
robot.reset()

# joint commands بھیجیں
commands = {"left_knee": 0.5, "right_knee": 0.5}
robot.send_joint_commands(commands)

# state feedback حاصل کریں
state = robot.get_state()
print(f"Left foot position: {state.left_foot_position}")
```

یہ کوڈ Unitree H1 کے ساتھ simulation میں روبوٹ کو کنٹرول کرنے کا طریقہ دکھاتا ہے۔ ہم robot کو initialize کرتے ہیں، reset کرتے ہیں، پھر joint commands بھیجتے ہیں، اور state feedback حاصل کرتے ہیں۔

### Performance Tuning (کارکردگی ٹیوننگ)

H1 walking کے اہم پیرامیٹرز:

| Parameter | Default | Tuning Range |
|-----------|---------|--------------|
| Step length | 0.25 m | 0.15-0.35 m |
| Step frequency | 2.5 Hz | 1.5-3.0 Hz |
| Pelvis height | 0.95 m | 0.85-1.05 m |
| ZMP margin | 0.02 m | 0.01-0.05 m |

## اگلے Steps

Weeks 11-12 مکمل کرنے کے بعد، جاری رکھیں:

- **Week 13-14**: Improved locomotion کے لیے Learning-based control
- **Week 15-16**: تمام مهارات کو ملا کر Final integration project
- **Chapter 6**: Learning-based humanoid control میں ایڈوانسڈ topics
- **Part 6**: Extended HRI capabilities کے لیے Conversational AI

## اضافی وسائل

### Documentation

- [Unitree H1 Documentation](https://www.unitree.com/products/h1)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [Humanoid Robotics Literature](https://arxiv.org/list/cs.RO/recent)

### Tutorials

- [Robot Dynamics Course](https://www.roboticsbook.org/)
- [Walking Control Tutorial](https://github.com/stephane-caron/pink)
- [Grasp Planning Examples](https://github.com/robotics/graspit)

### Community

- [ROS Humanoid Forum](https://discourse.ros.org/)
- [Humanoid Research Group](https://humanoids.org/)
- [Unitree Developer Community](https://forum.unitree.com/)

---

:::tip پرو ٹپ

Walking controllers کو debug کرتے وقت، ZMP trajectory کو real-time میں visualize کریں۔ اس سے استحکام کے مسائل کا پتہ لگانے میں مدد ملتی ہے اور controller gains tune کرنے کا intuition ملتا ہے۔

:::

:::note اہم

Walking controllers کو full-speed operation سے پہلے reduced speed پر test کریں۔ ZMP margin sufficient ہونا چاہیے modeling errors اور sensor noise handle کرنے کے لیے۔

:::

:::warning احتیاط

Physical robot testing کے لیے safety observers اور emergency stop capability درکار ہے۔ ہمیشہ proper safety measures کے بغیر نئے controllers کو hardware پر test نہ کریں۔

:::

**حصہ 5: Humanoid Development (انسان نما ڈویلپمنٹ)** | [Chapter 5: Humanoid Robot Development](part-5-humanoid/humanoid-robot-development) | [حصہ 6: Conversational Robotics](part-6-conversational/conversational-robotics)
