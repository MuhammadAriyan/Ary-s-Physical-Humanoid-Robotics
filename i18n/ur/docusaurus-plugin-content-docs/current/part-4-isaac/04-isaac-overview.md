---
title: "NVIDIA Isaac پلیٹ فارم"
sidebar_position: 4
---

# باب 4: فزیکل AI کے لیے NVIDIA Isaac پلیٹ فارم

## سیکھنے کے مقاصد

اس باب کے اختتام پر، آپ یہ کر سکیں گے:
- NVIDIA Isaac ایکوسسٹم کی تعمیر اور اجزاء کو سمجھنا
- Isaac Sim کو اعلیٰ معیار کی روبوٹ سمیولیشن کے لیے RTX پاورڈ رینڈرنگ کے ساتھ کانفیگر کرنا
- GPU تیز رفتار کمپیوٹر ویژن استعمال کرتے ہوئے AI پر مبنی ادراک پائپ لائنز لاگو کرنا
- Isaac Gym استعمال کرتے ہوئے GPU پر مبنی تقویتی سیکھنے سے ہیرا پھیری کی پالیسیاں تربیت دینا
- تربیت یافتہ پالیسیوں کو جسمانی روبوٹس پر تعینات کرنے کے لیے sim-to-real منتقلی کی تکنیکیں لاگو کرنا
- ہیومینائڈ روبوٹ کنٹرول کے لیے مکمل ادراک سے عمل تک کی پائپ لائنز بنانا

## 4.1 NVIDIA Isaac ایکوسسٹم کا تعارف

NVIDIA Isaac پلیٹ فارم فزیکل AI ایپلیکیشنز تیار کرنے، ٹیسٹ کرنے، اور تعینات کرنے کے لیے ایک جامع ایکوسسٹم ہے۔ روایتی روبوٹ ڈیولپمنٹ ٹولز جو simulation، perception، اور learning کو الگ concerns کے طور پر ٹریٹ کرتے ہیں، Isaac ایک مربوط ورک فلو فراہم کرتا ہے جو concept سے deployment تک پورے ڈیولپمنٹ سائیکل کو تیز کرتا ہے۔

### NVIDIA Isaac کیا ہے؟

NVIDIA Isaac دو بنیادی components پر مشتمل ہے جو ساتھ مل کر کام کرتے ہیں: Isaac SDK اور Isaac Sim۔ Isaac SDK روبوٹ navigation، manipulation، اور perception algorithms کے لیے APIs کے ساتھ software development kit فراہم کرتا ہے۔ Isaac Sim NVIDIA Omniverse پر بنایا گیا simulation environment ہے جو photorealistic rendering، accurate physics، اور physical robot deployments کے ساتھ seamless data exchange فراہم کرتا ہے۔

پلیٹ فارم NVIDIA کی GPU computing، deep learning، اور real-time simulation کی طاقتوں کا فائدہ اٹھاتا ہے humanoid robot development کے منفرد چیلنجز کو حل کرنے کے لیے۔ Humanoid robots کو sophisticated perception کی ضرورت ہوتی ہے انسانی ماحول کو سمجھنے کے لیے، precise manipulation مختلف اشیاء کے ساتھ تعامل کرنے کے لیے، اور robust locomotion پیچیدہ علاقوں کو navigate کرنے کے لیے۔ Isaac ہر چیلنج کے لیے مخصوص ٹولز فراہم کرتا ہے جبکہ پوری development workflow میں مستقل مزاجی برقرار رکھتا ہے۔

Isaac Sim خود کو دوسرے simulation platforms سے الگ کرتا ہے کئی اہم صلاحیتوں سے۔ RTX GPU architecture real-time ray tracing ممکن بناتا ہے جو photorealistic sensor data پیدا کرتا ہے، یہ perception models کی تربیت کے لیے بہت اہم ہے جو حقیقی دنیا کی صورتحال پر عمومی بنیں گے۔ Physics simulation PhysX 5 استعمال کرتا ہے درست contact dynamics کے لیے، یہ humanoid walking اور manipulation کے لیے ضروری ہے۔ Omniverse streaming capabilities simulations کو data center infrastructure پر دور سے چلانے کی اجازت دیتی ہے، بڑے پیمانے پر متوازی تربیتی منظرناموں کو ممکن بناتے ہوئے۔

### Isaac پلیٹ فارم کی تعمیر

Isaac architecture ایک layered ڈیزائن کی پیروی کرتا ہے جو concerns کو الگ کرتا ہے جبکہ مضبوط انضمام برقرار رکھتا ہے۔ بنیاد میں Omniverse ہے، NVIDIA کا کھلا پلیٹ فارم 3D workflows اور applications بنانے کے لیے۔ Omniverse real-time rendering engine، physics simulation backbone، اور data exchange protocols فراہم کرتا ہے جن پر Isaac Sim تعمیر ہوتا ہے۔

Omniverse کے اوپر، Isaac Sim robotics کے لیے domain-specific abstractions فراہم کرتا ہے۔ Isaac Gym module GPU-accelerated reinforcement learning فراہم کرتا ہے ہزاروں متوازی ماحول کی سہولت کے ساتھ۔ Isaac Perception module computer vision algorithms فراہم کرتا ہے جو GPU execution کے لیے بہتر بنائے گئے ہیں۔ Isaac Manipulation module grasp planning اور motion generation فراہم کرتا ہے robotic arms اور hands کے لیے۔

Application layer Isaac کو ROS 2 interfaces اور custom controller bindings کے ذریعے حقیقی روبوٹ ہارڈویئر سے جوڑتا ہے۔ Simulation میں تربیت یافتہ policies کو TensorRT-optimized inference engines میں export کیا جا سکتا ہے deployment کے لیے NVIDIA Jetson یا NVIDIA AGX platforms پر۔ یہ end-to-end workflow یقینی بناتا ہے کہ simulation میں ڈیولپمنٹس براہ راست physical robot capabilities میں تبدیل ہوں۔

```python
# Example: Isaac Sim initialization and robot loading
# Udaharan: Isaac Sim initialization aur robot loading
import omni.isaac.core
from omni.isaac.core import SimulationContext
from omni.isaac.core.objects import DynamicCuboid

# Simulation context initialize karna
# Simulation context ko initialize karna
simulation_context = SimulationContext(
    stage_units_in_meters=1.0,
    physics_dt=1.0 / 60.0,
    rendering_dt=1.0 / 60.0
)

# GPU physics simulation set up karna
# GPU physics simulation ko setup karna
omni.isaac.core.utils.physx.set_physics_gpu_device(0)

# USD se humanoid robot load karna
# USD se humanoid robot ko load karna
from pxr import Usd, UsdGeom
stage = simulation_context.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Robot ko USD file se load karna
# Robot ko USD file se load karna
robot_prim_path = "/World/HumanoidRobot"
from omni.isaac.core.utils.prims import create_prim_from_usd
robot = create_prim_from_usd(
    prim_path=robot_prim_path,
    usd_path="/path/to/humanoid.usd",
    prim_type="Robot"
)
```

### Hardware Requirements اور Setup

مؤثر Isaac development کے لیے مناسب hardware کی ضرورت ہے پلیٹ فارم کی صلاحیتوں کا فائدہ اٹھانے کے لیے۔ GPU requirements کافی ہیں real-time rendering اور physics simulation کے تقاضوں کی وجہ سے۔ Development workstations کے لیے، NVIDIA RTX 3090 یا RTX 4090 interactive simulation کے لیے ضروری کارکردگی فراہم کرتا ہے۔ بڑے پیمانے پر تربیتی منظرناموں کے لیے، NVIDIA A100 یا H100 GPUs متوازی environment execution کے لیے memory اور compute capacity فراہم کرتے ہیں۔

نیچے دی گئی table مختلف development scenarios کے لیے hardware سفارشات کا خلاصہ پیش کرتی ہے:

| Scenario | GPU | Memory | Storage | Use Case |
|----------|-----|--------|---------|----------|
| Interactive Development | RTX 4090 | 24GB | 1TB NVMe | Single robot simulation, algorithm development |
| Research Development | RTX 4090 (2x) | 48GB total | 2TB NVME | Multi-robot simulation, moderate training |
| Production Training | A100 80GB | 80GB | 4TB NVME | Large-scale RL training, dataset generation |
| Edge Deployment | Jetson AGX Orin | 64GB | 1TB NVME | Robot onboard inference |

Memory requirements simulation کی پیچیدگی کے ساتھ بڑھتی ہیں۔ مکمل sensor suite کے ساتھ ایک humanoid robot simulation عام طور پر 4-8GB GPU memory کی ضرورت ہوتی ہے rendering اور physics کے لیے۔ Reinforcement learning کے لیے متوازی environments شامل کرنے سے requirement کئی گنا ہو جاتی ہے۔ Memory allocation کی محتاط منصوبہ بندی development کے دوران رکاوٹوں کو روکتی ہے۔

## 4.2 AI پر مبنی ادراک

ادراک ہیومینائڈ روبوٹ کی خودمختاری کے لیے بنیاد بناتا ہے۔ ماحول کو سمجھنا، اشیاء کا پتہ لگانا، حرکت کو ٹریک کرنا، اور مناظر کو پہچاننا سب پیچیدہ کمپیوٹر ویژن صلاحیتوں کی ضرورت ہے۔ Isaac Perception GPU تیز رفتار implementations فراہم کرتا ہے جدید ترین الگورتھم کی جو simulation میں مؤثر طریقے سے چلتے ہیں اور جسمانی روبوٹس پر تعینات ہوتے ہیں۔

### کمپیوٹر ویژن کی بنیاد

روایتی کمپیوٹر ویژن طریقے ہاتھ سے بنائی گئی خصوصیات اور الگورتھمک پائپ لائنز پر انحصار کرتے تھے۔ جدید ادراک کے نظام اس کے بجائے گہرے neural networks استعمال کرتے ہیں جو hierarchical نمائندگی براہ راست ڈیٹا سے سیکھتے ہیں۔ Isaac ان networks کو simulation پائپ لائن کے ساتھ مربوط کرتا ہے، مصنوعی ڈیٹا پر تربیت اور جسمانی ہارڈویئر پر تعیناتی کو ممکن بناتے ہوئے۔

Isaac میں ادراک پائپ لائن ماڈیولر ڈیزائن کی پیروی کرتی ہے۔ سینسر انٹرفیس کیمروں، ڈیپتھ سینسرز، اور LIDAR سے خام ڈیٹا حاصل کرتے ہیں۔ پری پروسیسنگ مراحل ڈیٹا کو معمول بناتے ہیں اور neural network inference کے لیے ان پٹس تیار کرتے ہیں۔ Detection اور segmentation networks اشیاء اور ان کی حدود کی شناخت کرتے ہیں۔ ٹریکنگ الگورتھم فریمز میں شناخت برقرار رکھتے ہیں۔ اعلیٰ سطح کے ماڈیولز متعدد سینسرز سے معلومات کو یکجا کرتے ہیں اور منظر کی سمجھ بوجھ بناتے ہیں۔

GPU تیز رفتار حقیقی وقت کے ادراک کے لیے ضروری ہے۔ جدید neural networks ہر inference کے لیے اربوں آپریشنز کی ضرورت رکھتے ہیں۔ ان networks کو CPU پر چلانے سے روبوٹ کنٹرول کے لیے درکار فریم ریٹس حاصل نہیں کیے جا سکتے۔ Isaac TensorRT optimization کا فائدہ اٹھاتا ہے RTX GPUs پر 100+ FPS inference speeds حاصل کرنے کے لیے، کنٹرول فریکوئنسیز پر ادراک کو ممکن بناتے ہوئے۔

### Object Detection اور Segmentation

Object detection سینسر ڈیٹا میں دلچسپی کی مثالوں کی شناخت کرتی ہے اور انہیں bounding boxes یا segmentation masks کے ساتھ مقامی بناتی ہے۔ ہیومینائڈ روبوٹس کے لیے، کپ، ٹولز، اور رکاوٹوں جیسی عام اشیاء کا پتہ لگانا ہیرا پھیری اور نیویگیشن کو ممکن بناتا ہے۔ Isaac مقبول detection architectures کی implementations فراہم کرتا ہے جو robotics ایپلیکیشنز کے لیے بہتر بنائی گئی ہیں۔

نیچے دی گئی مثال object detection کے لیے مکمل ادراک پائپ لائن کا مظاہرہ کرتی ہے:

```python
# Example: Isaac Perception pipeline for object detection
# Udaharan: Isaac Perception pipeline object detection کے لیے
import numpy as np
from typing import Dict, List, Tuple
from omni.isaac.sensor import CameraSensor

class IsaacPerceptionPipeline:
    """
    GPU-accelerated perception pipeline for humanoid robot perception.
    Handles camera input, neural network inference, and result post-processing.
    """

    def __init__(
        self,
        camera_prim_path: str,
        detection_model_path: str,
        segmentation_model_path: str,
        device: str = "cuda"
    ):
        self.camera_prim_path = camera_prim_path
        self.device = device

        # Initialize camera sensor from USD
        # USD se camera sensor initialize karna
        self.camera = CameraSensor(
            prim_path=camera_prim_path,
            width=1280,
            height=720,
            fov=90.0
        )

        # Load TensorRT optimized detection model
        # TensorRT optimized detection model load karna
        self.detector = self._load_trt_model(detection_model_path, "detection")

        # Load TensorRT optimized segmentation model
        # TensorRT optimized segmentation model load karna
        self.segmenter = self._load_trt_model(segmentation_model_path, "segmentation")

        # Detection confidence threshold
        # Detection confidence threshold
        self.confidence_threshold = 0.5

        # Class names for common household objects
        # Common household objects کے لیے class names
        self.class_names = [
            "person", "cup", "bottle", "bowl", "tool",
            "chair", "table", "door", "box", "misc"
        ]

    def _load_trt_model(self, model_path: str, model_type: str):
        """
        Load a TensorRT optimized model for inference.
        Models should be exported from PyTorch using torch-tensorrt.
        """
        import tensorrt as trt

        # TensorRT logger for warnings and errors
        # Warnings اور errors کے لیے TensorRT logger
        logger = trt.Logger(trt.Logger.WARNING)

        # Load engine from serialized plan
        # Serialized plan se engine load karna
        with open(model_path, "rb") as f:
            engine_data = f.read()

        runtime = trt.Runtime(logger)
        engine = runtime.deserialize_cuda_engine(engine_data)

        # Create execution context
        # Execution context create karna
        context = engine.create_execution_context()

        return {
            "engine": engine,
            "context": context,
            "model_type": model_type
        }

    def capture_and_detect(self) -> Tuple[np.ndarray, List[Dict]]:
        """
        Capture camera image and run object detection.

        Returns:
            rgb_image: H x W x 3 numpy array of RGB values
            detections: List of detection dictionaries with bbox, class, confidence
        """
        # Capture RGB image from simulation camera
        # Simulation camera se RGB image capture karna
        rgb_image = self.camera.get_rgb()

        # Preprocess for neural network input
        # Neural network input کے لیے preprocess کرنا
        input_tensor = self._preprocess_image(rgb_image)

        # Run detection inference
        # Detection inference run karna
        detections = self._run_detection_inference(input_tensor)

        return rgb_image, detections

    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for model input.
        Includes normalization, resizing, and tensor conversion.
        """
        # Resize to model input size (e.g., 640 x 640)
        # Model input size par resize karna (e.g., 640 x 640)
        resized = np.resize(image, (640, 640, 3))

        # Normalize to [0, 1]
        # [0, 1] par normalize karna
        normalized = resized.astype(np.float32) / 255.0

        # Convert HWC to CHW format
        # HWC format se CHW mein convert karna
        transposed = np.transpose(normalized, (2, 0, 1))

        # Add batch dimension
        # Batch dimension add karna
        batched = np.expand_dims(transposed, axis=0)

        return batched

    def _run_detection_inference(
        self,
        input_tensor: np.ndarray
    ) -> List[Dict]:
        """
        Run object detection inference using TensorRT.
        """
        import torch
        import cupy as cp

        # Move input to GPU
        # Input ko GPU par move karna
        gpu_input = cp.asarray(input_tensor)

        # Allocate output buffers
        # Output buffers allocate karna
        num_detections = 100
        detection_output = cp.zeros((1, num_detections, 6), dtype=cp.float32)

        # Run inference
        # Inference run karna
        context = self.detector["context"]
        context.execute_v2(
            bindings=[
                gpu_input.data_ptr(),
                detection_output.data_ptr()
            ]
        )

        # Transfer results to CPU
        # Results ko CPU par transfer karna
        output_cpu = cp.asnumpy(detection_output)

        # Parse detections
        # Detectations parse karna
        detections = []
        for i in range(num_detections):
            confidence = output_cpu[0, i, 4]
            if confidence < self.confidence_threshold:
                continue

            class_id = int(output_cpu[0, i, 5])
            bbox = output_cpu[0, i, :4]

            detections.append({
                "bbox": bbox,  # [x1, y1, x2, y2]
                "class_id": class_id,
                "class_name": self.class_names[class_id],
                "confidence": float(confidence)
            })

        return detections

    def get_segmentation_mask(self) -> np.ndarray:
        """
        Capture camera and generate segmentation mask.

        Returns:
            mask: H x W numpy array of class IDs
        """
        rgb_image = self.camera.get_rgb()
        input_tensor = self._preprocess_image(rgb_image)

        # Run segmentation inference
        # Segmentation inference run karna
        import cupy as cp

        gpu_input = cp.asarray(input_tensor)
        seg_output = cp.zeros((1, 640, 640, 10), dtype=cp.float32)

        context = self.segmenter["context"]
        context.execute_v2(
            bindings=[
                gpu_input.data_ptr(),
                seg_output.data_ptr()
            ]
        )

        # Get argmax for class labels
        # Class labels کے لیے argmax get کرنا
        mask = cp.argmax(seg_output, axis=-1)

        return cp.asnumpy(mask)
```

### گہرائی اور 3D ادراک

ہیومینائڈ روبوٹس کو صرف یہ سمجھنا نہیں ہے کہ اشیاء موجود ہیں، بلکہ یہ بھی سمجھنا ہے کہ وہ تین جہتی خلا میں کہاں ہیں۔ ڈیپتھ سینسرز اور multi-view geometry 3D معلومات فراہم کرتے ہیں جو ہیرا پھیری اور نیویگیشن کے لیے ضروری ہے۔ Isaac Sim اعلیٰ معیار کی ڈیپتھ سینسر simulation فراہم کرتا ہے جو learning پر مبنی depth estimation کے لیے تربیتی ڈیٹا پیدا کرتا ہے۔

Monocular کیمروں سے depth estimation neural networks استعمال کرتا ہے single RGB تصاویر سے dense depth maps کی پیش گوئی کرنے کے لیے۔ یہ networks RGB-D captures کے بڑے datasets سے سیکھتے ہیں اور نئے ماحول پر عمومی بنتے ہیں۔ Isaac Sim سے synthetic ڈیٹا پر تربیت حقیقی datasets کی تکمیل کر سکتی ہے ان ڈومینز کے لیے جہاں ڈیٹا اکٹھا کرنے کے مواقع محدود ہیں۔

```python
# Example: Depth estimation from RGB using neural network
# Udaharan: RGB se neural network use karke depth estimation
class DepthEstimationPipeline:
    """
    Monocular depth estimation using a neural network.
    Generates dense depth maps from single RGB images for robot perception.
    """

    def __init__(
        self,
        model_path: str,
        min_depth: float = 0.1,
        max_depth: float = 10.0
    ):
        self.min_depth = min_depth
        self.max_depth = max_depth

        # Load depth estimation model
        # Depth estimation model load karna
        self.model = self._load_depth_model(model_path)
        self.model.eval()

    def _load_depth_model(self, model_path: str):
        """Load a pre-trained depth estimation model."""
        import torch
        import torch_tensorrt

        # Model architecture (e.g., MiDaS-style)
        # Model architecture (e.g., MiDaS-style)
        model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")

        # Load trained weights
        # Trained weights load karna
        state_dict = torch.load(model_path)
        model.load_state_dict(state_dict)

        # Optimize with TensorRT for GPU inference
        # GPU inference کے لیے TensorRT سے optimize کرنا
        compiled_model = torch_tensorrt.compile(
            model,
            inputs=[
                torch_tensorrt.Input(shape=[1, 3, 384, 384])
            ],
            enabled_precisions={torch.float, torch.half}
        )

        return compiled_model

    def estimate_depth(self, rgb_image: np.ndarray) -> np.ndarray:
        """
        Estimate depth map from a single RGB image.

        Args:
            rgb_image: H x W x 3 numpy array in [0, 255] range

        Returns:
            depth_map: H x W numpy array of depth values in meters
        """
        import torch
        import cupy as cp

        # Preprocess
        # Preprocess karna
        input_tensor = self._preprocess_depth(rgb_image)

        # Move to GPU and run inference
        # GPU par move karna aur inference run karna
        with torch.no_grad():
            gpu_input = torch.tensor(input_tensor, device="cuda")
            depth_raw = self.model(gpu_input)

        # Postprocess to metric depth
        # Metric depth mein postprocess karna
        depth_metric = self._postprocess_depth(depth_raw)

        return depth_metric

    def _preprocess_depth(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for depth model input."""
        import cv2

        # Resize to model input size
        # Model input size par resize karna
        resized = cv2.resize(image, (384, 384))

        # Convert to tensor and normalize
        # Tensor mein convert karna aur normalize karna
        tensor = torch.from_numpy(resized).float().permute(2, 0, 1) / 255.0

        # Apply ImageNet normalization
        # ImageNet normalization apply karna
        mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
        std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
        normalized = (tensor - mean) / std

        # Add batch dimension
        # Batch dimension add karna
        batched = normalized.unsqueeze(0)

        return batched.numpy()

    def _postprocess_depth(self, depth_raw: torch.Tensor) -> np.ndarray:
        """Convert raw model output to metric depth."""
        import cv2

        # Remove batch dimension
        # Batch dimension hata karna
        depth_squeezed = depth_raw.squeeze().cpu().numpy()

        # Resize to original image size
        # (Assume original size is stored or passed as parameter)

        # Apply depth scaling from relative to metric
        # Relative se metric mein depth scaling apply karna
        # یہ training dataset اور model پر منحصر ہے
        depth_scaled = depth_squeezed * (self.max_depth - self.min_depth)
        depth_scaled = np.clip(depth_scaled, self.min_depth, self.max_depth)

        return depth_scaled
```

### Point Cloud Processing

Point clouds گہرائی سے براہ راست 3D geometry کی نمائندگی فراہم کرتے ہیں۔ Point clouds کو مؤثر طریقے سے process کرنے کے لیے GPU تیز رفتاری ضروری ہے کیونکہ ہر فریم میں لاکھوں نقاط پیدا ہوتے ہیں۔ Isaac cuML-تیز شدہ الگورتھم عام point cloud operations کے لیے فراہم کرتا ہے جیسے فلٹرنگ، کلسٹرنگ، اور خصوصیات کی نکاسی۔

Point cloud segmentation اشیاء کو پس منظر سے الگ کرتا ہے اور نقاط کو الگ instances میں گروپ کرتا ہے۔ ہیومینائڈ روبوٹس کے لیے، فرش، دیواروں، فرنیچر، اور اشیاء کو segment کرنا manipulation planning کے لیے منظر کی سمجھ بوجھ کو ممکن بناتا ہے۔ Learning پر مبنی segmentation networks جو Isaac Sim سے synthetic ڈیٹا پر تربیت یافتہ ہیں حقیقی دنیا کی تعیناتی پر منتقل ہو سکتے ہیں۔

## 4.3 Isaac Gym سے ہیرا پھیری

ہیرا پھیری robotic ہاتھوں اور بازوؤں کو درست طریقے سے کنٹرول کرنے کی ضرورت ہوتی ہے اشیاء کو پکڑنے اور منتقل کرنے کے لیے۔ Isaac Gym reinforcement learning استعمال کرکے manipulation policies کی تربیت کے لیے متحد ماحول فراہم کرتا ہے۔ GPU تیز رفتار simulation ہزاروں متوازی ماحول کی حمایت کرتا ہے، نمونہ مؤثر policy learning کو ممکن بناتے ہوئے۔

### Isaac Gym کی بنیادی باتیں

Isaac Gym Omniverse physics simulation کو بڑے پیمانے پر متوازی reinforcement learning کی حمایت کے لیے بڑھاتا ہے۔ روایتی طریقے متعدد simulation instances کو CPU cores پر چلاتے ہیں، single-core کی کارکردگی سے محدود۔ Isaac Gym اس کے بجائے تمام simulations کو GPU پر چلاتا ہے، جدید graphics processors کی بڑے پیمانے پر parallelism کا فائدہ اٹھاتے ہوئے۔

Architecture physics simulation کو GPU پر Python تربیتی کوڈ سے الگ کرتا ہے۔ یہ training loop کو simulation کے ساتھ ایک ہی GPU پر execute کرنے دیتا ہے، ڈیٹا منتقلی کے overhead کو کم کرتے ہوئے۔ نتیجہ یہ ہے کہ تربیت کی throughput GPU کی صلاحیت کے ساتھ بڑھتی ہے، CPU core count نہیں۔

ہیرا پھیری کے لیے Isaac Gym سیٹ اپ کرنے میں simulation ماحول کی تشکیل، روبوٹ اور object dynamics کی تعریف، اور reinforcement learning کے لیے reward function کی وضاحت شامل ہے۔ نیچے دی گئی مثال مکمل manipulation تربیتی سیٹ اپ کا مظاہرہ کرتی ہے:

```python
# Example: Isaac Gym manipulation training setup
# Udaharan: Isaac Gym manipulation training setup
import numpy as np
import torch
import gymnasium as gym
from gymnasium import spaces

class IsaacGymManipulationEnv(gym.Env):
    """
    Manipulation environment for Isaac Gym reinforcement learning.
    Robot hand learns to grasp and move objects to target positions.
    """

    def __init__(
        self,
        num_envs: int = 4096,
        device: str = "cuda",
        headless: bool = True
    ):
        self.num_envs = num_envs
        self.device = device
        self.headless = headless

        # Initialize Isaac Gym
        # Isaac Gym initialize karna
        import isaacgym
        from isaacgym import gymapi, gymtorch

        # Create the simulation
        # Simulation create karna
        self.gym = gymapi.acquire_gym()

        # Create simulation configuration
        # Simulation configuration create karna
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / 60.0  # 60 Hz control
        sim_params.num_client_threads = 0
        sim_params.use_gpu_pipeline = True
        sim_params.device_id = 0 if device == "cuda" else -1

        # Configure physics
        # Physics configure karna
        sim_params.physx.solver_type = 1  # PGS solver
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1

        # Create simulation
        # Simulation create karna
        self.sim = self.gym.create_sim(
            device_id=sim_params.device_id,
            graphics_device_id=sim_params.device_id,
            sim_type=gymapi.SIM_PHYSX,
            sim_params=sim_params
        )

        # Create environments
        # Environments create karna
        self._create_environments()

        # Define observation and action spaces
        # Observation aur action spaces define karna
        self._setup_spaces()

        # Track episode progress
        # Episode progress track karna
        self.reset_buf = torch.ones(num_envs, dtype=torch.bool, device=device)
        self.progress_buf = torch.zeros(num_envs, dtype=torch.int32, device=device)
        self.max_episode_length = 500

    def _create_environments(self):
        """Create parallel manipulation environments."""
        import isaacgym.gymapi as gymapi
        from isaacgym import gymtorch

        # Environment spacing
        # Environment spacing
        env_spacing = 0.5
        num_per_row = int(np.sqrt(self.num_envs))

        # Asset paths for robot hand and objects
        # Robot hand اور objects کے لیے asset paths
        hand_asset_path = "assets/robot_hand.urdf"
        object_asset_path = "assets/cube.urdf"

        # Load assets
        # Assets load karna
        hand_asset = self.gym.load_asset(
            self.sim,
            "assets",
            hand_asset_path,
            gymapi.AssetOptions()
        )
        object_asset = self.gym.load_asset(
            self.sim,
            "assets",
            object_asset_path,
            gymapi.AssetOptions()
        )

        # Create environments
        # Environments create karna
        self.envs = []
        self.hand_handles = []
        self.object_handles = []

        for i in range(self.num_envs):
            env = self.gym.create_env(
                self.sim,
                env_spacing * (i % num_per_row),
                env_spacing * (i // num_per_row),
                env_spacing,
                num_per_row
            )
            self.envs.append(env)

            # Create robot hand
            # Robot hand create karna
            hand_pose = gymapi.Transform()
            hand_pose.p = gymapi.Vec3(0, 0, 0.5)
            hand_handle = self.gym.create_actor(
                env, hand_asset, hand_pose, "hand", i, 1
            )
            self.hand_handles.append(hand_handle)

            # Create object to grasp
            # Grasp کے لیے object بنانا
            object_pose = gymapi.Transform()
            object_pose.p = gymapi.Vec3(
                (np.random.rand() - 0.5) * 0.2,
                (np.random.rand() - 0.5) * 0.2,
                0.05
            )
            object_handle = self.gym.create_actor(
                env, object_asset, object_pose, "object", i, 2
            )
            self.object_handles.append(object_handle)

        # Create viewer for visualization
        # Visualization کے لیے viewer create کرنا
        if not self.headless:
            self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())

    def _setup_spaces(self):
        """Configure observation and action spaces."""
        # Observation: hand pose, velocity, object pose, relative position
        # Observation: hand pose, velocity, object pose, relative position
        num_observations = 30  # Hand 7-dof pose + 6-dof vel + object 7-dof pose
        num_actions = 7  # Joint position targets for robot hand

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(num_observations,),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(num_observations,),
            dtype=np.float32
        )

    def reset(
        self,
        seed: int = None,
        options: dict = None
    ) -> Tuple[torch.Tensor, dict]:
        """Reset environments to initial state."""
        # Reset object positions
        # Object positions reset karna
        for i, env in enumerate(self.envs):
            # Randomize object position
            # Object position randomize karna
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(
                (np.random.rand() - 0.5) * 0.2,
                (np.random.rand() - 0.5) * 0.2,
                0.05
            )
            self.gym.set_rigid_body_pose(
                env,
                self.object_handles[i],
                0,
                pose
            )

        # Reset hand to starting position
        # Hand ko starting position par reset karna
        for i, env in enumerate(self.envs):
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 0.5)
            self.gym.set_rigid_body_pose(
                env,
                self.hand_handles[i],
                0,
                pose
            )

        # Get observation after reset
        # Reset ke baad observation get karna
        obs = self._get_observation()

        return obs, {}

    def step(
        self,
        actions: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict]:
        """
        Execute action and return new state, reward, done, info.

        Args:
            actions: Tensor of shape (num_envs, num_actions) with joint targets

        Returns:
            obs: New observations
            rewards: Reward for each environment
            dones: Episode termination flags
            info: Additional information
        """
        # Apply actions as joint position targets
        # Actions ko joint position targets ke tor par apply karna
        self._apply_actions(actions)

        # Step physics simulation
        # Physics simulation step karna
        for _ in range(4):  # Sub-stepping for stability
            self.gym.simulate(self.sim)

        # Step visual update
        # Visual update step karna
        self.gym.fetch_results(self.sim, True)

        # Get new observations
        # Naye observations get karna
        obs = self._get_observation()

        # Calculate rewards
        # Rewards calculate karna
        rewards = self._compute_reward()

        # Check episode termination
        # Episode termination check karna
        dones = self._check_termination()

        # Update episode progress
        # Episode progress update karna
        self.progress_buf += 1

        return obs, rewards, dones, {}

    def _apply_actions(self, actions: torch.Tensor):
        """Apply joint position targets to robot hands."""
        # Robot hands par joint position targets apply karna
        for i, env in enumerate(self.envs):
            # Apply PD control for smooth tracking
            # Smooth tracking کے لیے PD control apply کرنا
            joint_positions = actions[i].cpu().numpy()
            # In practice, use joint controller interface
            # Practice mein, joint controller interface use karna
            self.gym.set_actor_dof_position_targets(
                env,
                self.hand_handles[i],
                joint_positions
            )

    def _get_observation(self) -> torch.Tensor:
        """Collect observations from all environments."""
        # Saare environments se observations collect karna
        import torch
        from isaacgym import gymtorch

        # Stack observations from multiple sources
        # Multiple sources se observations stack karna
        observations = []

        # Get hand states
        # Hand states get karna
        hand_states = self.gym.get_actor_dof_states(
            self.envs,
            self.hand_handles,
            "joint"
        )
        observations.append(hand_states["position"])
        observations.append(hand_states["velocity"])

        # Get object states
        # Object states get karna
        object_states = self.gym.get_actor_rigid_body_states(
            self.envs,
            self.object_handles,
            "pos_quat"
        )
        observations.append(object_states["pose"])

        # Concatenate and return
        # Concatenate karna aur return karna
        return torch.cat(observations, dim=-1)

    def _compute_reward(self) -> torch.Tensor:
        """Compute dense reward for grasping task."""
        # Grasping task کے لیے dense reward compute کرنا
        import torch
        from isaacgym import gymtorch

        # Get hand and object positions
        # Hand اور object positions get کرنا
        hand_states = self.gym.get_actor_rigid_body_states(
            self.envs,
            self.hand_handles,
            "pos_quat"
        )
        object_states = self.gym.get_actor_rigid_body_states(
            self.envs,
            self.object_handles,
            "pos_quat"
        )

        hand_positions = hand_states["pose"][:, :3]
        object_positions = object_states["pose"][:, :3]

        # Distance between hand and object
        # Hand اور object کے بیچ distance
        distance = torch.norm(hand_positions - object_positions, dim=1)

        # Reward: negative distance (closer is better)
        # Reward: negative distance (قریب تر بہتر ہے)
        reward = -distance

        # Bonus for successful grasp (very close distance)
        # Successful grasp کے لیے bonus (very close distance)
        grasp_bonus = torch.where(
            distance < 0.02,
            torch.ones_like(distance) * 1.0,
            torch.zeros_like(distance)
        )
        reward += grasp_bonus

        return reward

    def _check_termination(self) -> torch.Tensor:
        """Check if episodes should terminate."""
        # Check karna ki episodes terminate honi chahiye ya nahi
        import torch

        # Max episode length exceeded
        # Max episode length exceed ho gayi
        maxed_out = self.progress_buf >= self.max_episode_length

        # Object fell off table (y position too low)
        # Object table se gir gayi (y position too low)
        object_states = self.gym.get_actor_rigid_body_states(
            self.envs,
            self.object_handles,
            "pos_quat"
        )
        object_positions = object_states["pose"][:, 1]
        fallen = object_positions < 0.01

        return torch.logical_or(maxed_out, fallen)
```

### گرفت کی منصوبہ بندی اور عملدرآمد

کامیاب ہیرا پھیری کے لیے صرف موٹر کنٹرول نہیں، بلکہ ذہین فیصلہ سازی بھی ضروری ہے کہ اشیاء کو کیسے approach کیا جائے اور پکڑا جائے۔ Grasp planning الگورتھم ممکنہ grasps کا جائزہ لیتے ہیں اور ان کو منتخب کرتے ہیں جو کامیاب ہونے کے سب سے زیادہ امکان رکھتے ہیں۔ Isaac grasp sampling الگورتھم فراہم کرتا ہے جو متنوع اشیاء کے لیے امیدوار grasps پیدا کرتے ہیں۔

Learning پر مبنی grasp planning نے تجزیاتی طریقوں سے بہتر کارکردگی دکھائی ہے نئی اشیاء کے لیے۔ Neural networks grasp کے معیار کو تجربے سے سیکھ سکتے ہیں، نادیدہ اشیاء اور مشکل configurations پر عمومی بنتے ہوئے۔ ایسے networks کی تربیت کے لیے grasp attempts کے بڑے datasets کی ضرورت ہوتی ہے، جو simulation مؤثر طریقے سے پیدا کر سکتا ہے۔

## 4.4 روبوٹ کنٹرول کے لیے Reinforcement Learning

Reinforcement learning پیچیدہ روبوٹ رویوں کے لیے ایک راستہ پیش کرتا ہے جو ہاتھ سے engineering سے مزاحمت کرتے ہیں۔ ہر حرکت کو واضح طور پر پروگرام کرنے کے بجائے، RL روبوٹس کو آزمائش اور غلطی سے سیکھنے دیتا ہے۔ Isaac Gym بڑے پیمانے پر policies کی تربیت کے لیے computational substrate فراہم کرتا ہے۔

### Robotics کے لیے RL بنیادیں

Reinforcement learning اس مسئلے کو حل کرتا ہے جس میں agent ماحول کے ساتھ تعامل سے مجموعی reward کو زیادہ سے زیادہ کرنا سیکھتا ہے۔ Agent ماحول کی حالت کا مشاہدہ کرتا ہے، actions لیتا ہے، rewards حاصل کرتا ہے، اور تجربے کی بنیاد پر اپنی policy کو update کرتا ہے۔ Robotics کے لیے، یہ نمونہ پیچیدہ sensorimotor رویوں کی learning کو ممکن بناتا ہے جو دستی طور پر design کرنا مشکل ہے۔

Robotics RL میں چیلنج نمونے کی کارکردگی اور حفاظت میں ہے۔ جسمانی روبوٹس وہ لاکھوں episodes execute نہیں کر سکتے جو simulation پر مبنی RL اکثر ضرورت رکھتا ہے۔ ہارڈویئر پر براہ راست تربیت سے نقصان کا خطرہ ہوتا ہے اور محتاط حفاظتی پابندیوں کی ضرورت ہوتی ہے۔ Isaac Sim ان چیلنجز کو حل کرتا ہے تیز، متوازی simulation فراہم کرکے جہاں policies اربوں simulated تعاملات سے سیکھ سکتی ہیں تعیناتی سے پہلے۔

Isaac Gym مقبول RL الگورتھم کے ساتھ مربوط ہوتا ہے جیسے PPO (Proximal Policy Optimization)، SAC (Soft Actor-Critic)، اور TD3 (Twin Delayed DDPG)۔ نیچے دی گئی مثال Isaac Gym کے ساتھ PPO تربیت دکھاتی ہے:

```python
# Example: PPO training for robot manipulation
# Udaharan: Robot manipulation کے لیے PPO training
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
from typing import Tuple, Dict

class PPOPolicyNetwork(nn.Module):
    """
    Actor-Critic network for PPO.
    Outputs both policy mean and value estimate.
    """

    def __init__(
        self,
        observation_dim: int,
        action_dim: int,
        hidden_dims: list = [512, 256]
    ):
        super().__init__()

        # Shared feature extractor
        # Shared feature extractor
        self.feature_net = nn.Sequential()
        in_dim = observation_dim
        for h_dim in hidden_dims:
            self.feature_net.extend([
                nn.Linear(in_dim, h_dim),
                nn.LayerNorm(h_dim),
                nn.Tanh()
            ])
            in_dim = h_dim

        # Policy head (mean of action distribution)
        # Policy head (action distribution ka mean)
        self.policy_mean = nn.Linear(hidden_dims[-1], action_dim)
        self.policy_log_std = nn.Parameter(torch.zeros(action_dim))

        # Value head
        # Value head
        self.value_head = nn.Linear(hidden_dims[-1], 1)

    def forward(
        self,
        observations: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Forward pass returning value estimate.
        For action sampling, use get_action_and_log_prob().
        """
        features = self.feature_net(observations)
        value = self.value_head(features)
        return value

    def get_action_and_log_prob(
        self,
        observations: torch.Tensor,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Sample action from policy and compute log probability.

        Returns:
            actions: Sampled action tensor
            log_probs: Log probability of sampled actions
            values: Value function estimates
        """
        features = self.feature_net(observations)

        # Get action distribution parameters
        # Action distribution parameters get karna
        mean = self.policy_mean(features)
        log_std = self.policy_log_std.expand_as(mean)
        std = torch.exp(log_std)

        # Sample action
        # Action sample karna
        if deterministic:
            actions = mean
            log_probs = torch.zeros_like(mean)
        else:
            dist = Normal(mean, std)
            actions = dist.rsample()  # Reparameterized sample
            log_probs = dist.log_prob(actions)

        # Clip actions to valid range
        # Actions ko valid range mein clip karna
        actions = torch.tanh(actions)

        # Value estimate
        # Value estimate
        values = self.value_head(features)

        return actions, log_probs, values


class PPOAgent:
    """
    PPO agent for robot control training.
    Implements clipped objective optimization.
    """

    def __init__(
        self,
        observation_dim: int,
        action_dim: int,
        lr: float = 3e-4,
        gamma: float = 0.99,
        clip_epsilon: float = 0.2,
        value_coef: float = 0.5,
        entropy_coef: float = 0.01,
        update_epochs: int = 10,
        minibatch_size: int = 4096,
        device: str = "cuda"
    ):
        self.gamma = gamma
        self.clip_epsilon = clip_epsilon
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.update_epochs = update_epochs
        self.minibatch_size = minibatch_size

        # Policy network
        # Policy network
        self.policy = PPOPolicyNetwork(
            observation_dim, action_dim
        ).to(device)

        # Optimizer
        # Optimizer
        self.optimizer = torch.optim.Adam(
            self.policy.parameters(),
            lr=lr
        )

        # Memory buffer
        # Memory buffer
        self.buffer = {
            "observations": [],
            "actions": [],
            "rewards": [],
            "dones": [],
            "log_probs": [],
            "values": []
        }

        self.device = device

    def select_action(
        self,
        observations: torch.Tensor,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Select action given current observation.
        """
        observations = observations.to(self.device)

        with torch.no_grad():
            actions, log_probs, values = self.policy.get_action_and_log_prob(
                observations, deterministic=deterministic
            )

        return actions.cpu(), log_probs.cpu(), values.cpu()

    def store_transition(
        self,
        observation: torch.Tensor,
        action: torch.Tensor,
        reward: float,
        done: bool,
        log_prob: torch.Tensor,
        value: torch.Tensor
    ):
        """Store transition in replay buffer."""
        # Replay buffer mein transition store karna
        self.buffer["observations"].append(observation)
        self.buffer["actions"].append(action)
        self.buffer["rewards"].append(reward)
        self.buffer["dones"].append(done)
        self.buffer["log_probs"].append(log_prob)
        self.buffer["values"].append(value)

    def compute_returns_and_advantages(self):
        """
        Compute discounted returns and GAE advantages.
        """
        # Convert buffer to tensors
        # Buffer ko tensors mein convert karna
        observations = torch.stack(self.buffer["observations"])
        rewards = torch.tensor(self.buffer["rewards"])
        dones = torch.tensor(self.buffer["dones"])
        old_log_probs = torch.stack(self.buffer["log_probs"])
        values = torch.stack(self.buffer["values"])

        # Compute discounted returns
        # Discounted returns compute karna
        returns = torch.zeros_like(rewards)
        R = 0
        for t in reversed(range(len(rewards))):
            if dones[t]:
                R = 0
            R = rewards[t] + self.gamma * R
            returns[t] = R

        # Compute advantages using GAE
        # GAE use karke advantages compute karna
        advantages = torch.zeros_like(rewards)
        gae = 0
        lam = 0.95  # GAE lambda
        for t in reversed(range(len(rewards))):
            if t < len(rewards) - 1:
                delta = rewards[t] + self.gamma * values[t+1] * (1 - dones[t]) - values[t]
            else:
                delta = rewards[t] - values[t]
            gae = delta + self.gamma * lam * (1 - dones[t]) * gae
            advantages[t] = gae

        # Normalize advantages
        # Advantages normalize karna
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        return observations, returns, advantages, old_log_probs

    def update(self):
        """Perform PPO update step."""
        # PPO update step perform karna
        # Compute returns and advantages
        # Returns aur advantages compute karna
        obs, returns, advantages, old_log_probs = self.compute_returns_and_advantages()

        # Flatten for minibatch sampling
        # Minibatch sampling کے لیے flatten کرنا
        flat_obs = obs.reshape(-1, *obs.shape[2:])
        flat_returns = returns.reshape(-1)
        flat_advantages = advantages.reshape(-1)
        flat_old_log_probs = old_log_probs.reshape(-1)

        # Calculate number of minibatches
        # Minibatches ki ginti karna
        num_samples = flat_obs.shape[0]
        num_batches = num_samples // self.minibatch_size

        total_policy_loss = 0
        total_value_loss = 0
        total_entropy = 0

        for _ in range(self.update_epochs):
            # Random minibatch sampling
            # Random minibatch sampling
            indices = torch.randperm(num_samples)[:self.minibatch_size]

            batch_obs = flat_obs[indices]
            batch_returns = flat_returns[indices]
            batch_advantages = flat_advantages[indices]
            batch_old_log_probs = flat_old_log_probs[indices]

            # Get new log probs and values
            # Naye log probs aur values get karna
            new_log_probs, entropy = self._get_log_probs_and_entropy(batch_obs, batch_actions)

            # Compute PPO clipped objective
            # PPO clipped objective compute karna
            ratio = torch.exp(new_log_probs - batch_old_log_probs)
            surr1 = ratio * batch_advantages
            surr2 = torch.clamp(ratio, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * batch_advantages
            policy_loss = -torch.min(surr1, surr2).mean()

            # Value loss
            # Value loss
            value_loss = F.mse_loss(values.squeeze(), batch_returns)

            # Entropy bonus
            # Entropy bonus
            entropy_loss = -entropy.mean()

            # Total loss
            # Total loss
            loss = policy_loss + self.value_coef * value_loss + self.entropy_coef * entropy_loss

            # Optimization step
            # Optimization step
            self.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 0.5)
            self.optimizer.step()

            total_policy_loss += policy_loss.item()
            total_value_loss += value_loss.item()
            total_entropy += entropy.item()

        # Clear buffer
        # Buffer clear karna
        self.buffer = {
            "observations": [],
            "actions": [],
            "rewards": [],
            "dones": [],
            "log_probs": [],
            "values": []
        }

        # Log metrics
        # Metrics log karna
        return {
            "policy_loss": total_policy_loss / self.update_epochs,
            "value_loss": total_value_loss / self.update_epochs,
            "entropy": total_entropy / self.update_epochs
        }

    def _get_log_probs_and_entropy(
        self,
        observations: torch.Tensor,
        actions: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Compute log probabilities and entropy of current policy.
        """
        features = self.policy.feature_net(observations)
        mean = self.policy.policy_mean(features)
        log_std = self.policy.policy_log_std.expand_as(mean)
        std = torch.exp(log_std)

        # Compute log prob of actions
        # Actions ka log prob compute karna
        dist = Normal(mean, std)
        log_probs = dist.log_prob(actions)

        # Clip for numerical stability
        # Numerical stability کے لیے clip کرنا
        log_probs = torch.clamp(log_probs, -20, 20)

        # Compute entropy
        # Entropy compute karna
        entropy = 0.5 + 0.5 * np.log(2 * np.pi) + torch.log(std)
        entropy = entropy.sum(dim=-1)

        return log_probs, entropy
```

### Domain Randomization

حقیقی دنیا میں منتقل ہونے والی policies کی تربیت کے لیے انہیں simulation کے دوران تبدیلی سے آشنا کرنے کی ضرورت ہے۔ Domain randomization منظم طریقے سے simulation parameters میں تبدیلی کرتا ہے جیسے lighting، object کے رنگ، friction coefficients، اور physics dynamics۔ یہ تبدیلی policies کو مضبوط سکھاتی ہے distribution shift کے خلاف جس کا وہ سامنا کریں گے جب تعینات ہوں۔

مؤثر domain randomization حقیقت پسندی اور تبدیلی کے درمیان توازن رکھتا ہے۔ کم تبدیلی سے policies simulation artifacts پر overfit ہوتی ہیں۔ زیادہ تبدیلی سے learning غیر ضروری طور پر مشکل ہو جاتی ہے۔ تحقیق تجویز کرتی ہے کہ randomization کو ان parameters پر مرکوز رکھنا چاہیے جو task کی کارکردگی اور simulation اور حقیقت کے درمیان distribution gap کو سب سے زیادہ متاثر کرتے ہیں۔

## 4.5 Sim-to-Real منتقلی کی تکنیکیں

Simulation میں تربیت یافتہ policies کو جسمانی روبوٹس پر تعینات کرنا ایک اہم چیلنج ہے۔ Sim-to-real gap physics dynamics، sensor خصوصیات، اور ماحولیاتی حالات میں فرق پر مشتمل ہے۔ کامیاب منتقلی کے لیے ان gaps کو حل کرنا محتاط نظام کی تشکیل اور تربیت کی ضرورت ہے۔

### Sim-to-Real Gap کو سمجھنا

بنیادی چیلنج یہ ہے کہ simulations جسمانی حقیقت کے تخمینہ ماڈل ہیں۔ Physics engines contact dynamics، friction، اور actuator رویے کے بارے میں آسان فرضیات کرتے ہیں۔ Rendering engines روشنی کی منتقلی اور مواد کی ظاہری شکل کا تخمینہ لگاتے ہیں۔ یہ تخمینے مل کر policies بناتے ہیں جو simulation artifacts کا فائدہ اٹھاتی ہیں، مضبوط رویے سیکھنے کے بجائے۔

Physics gaps کئی طریقوں سے ظاہر ہوتے ہیں۔ Simulators میں contact models collisions کو penalty forces یا impulse پر مبنی resolution کے ذریعے handle کرتے ہیں، دونوں حقیقی جسمانی contact کے تخمینے ہیں۔ Simulated contacts کی stiffness، damping، اور friction شاذ و نادر ہی حقیقی مواد سے بالکل میچ کرتے ہیں۔ Robot dynamics ماڈل کامل torque transmission فرض کرتے ہیں، gearbox compliance، joint friction، اور calibration errors کو نظر انداز کرتے ہوئے۔

Perception gaps simulated اور حقیقی sensor ڈیٹا کے درمیان فرق سے پیدا ہوتے ہیں۔ Simulation سے کیمرا تصاویر کے پاس مثالی noise profiles، کامل white balance، اور مستقل lens خصوصیات ہوتی ہیں۔ حقیقی کیمروں کے پاس lens aberrations، rolling shutter effects، اور exposure میں تبدیلیاں ہوتی ہیں۔ Simulation میں depth sensors مواد کی خصوصیات اور lighting conditions کے بارے میں آسان فرضیات کرتے ہیں۔

### System Identification اور Calibration

System identification تکنیکیں جسمانی نظاموں کی خصوصیات کی پیمائش کرتی ہیں اور simulation parameters کو میچ کرنے کے لیے ٹیون کرتی ہیں۔ یہ calibration عمل sim-to-real gap کو کم کرتا ہے simulation کی وفاداری بہتر بناکر۔ ہیومینائڈ روبوٹس کے لیے، system identification joint dynamics، sensor خصوصیات، اور contact خصوصیات پر مرکوز ہوتی ہے۔

Actuator characterization کمانڈ شدہ signals اور حقیقی حرکت کے درمیان تعلق کی پیمائش کرتا ہے۔ اس میں motor resistance، gearbox ratios، friction profiles، اور torque کی حدود کی شناخت شامل ہے۔ شناخت شدہ parameters simulation joint ماڈلز کی تشکیل کے لیے استعمال ہوتے ہیں، dynamic درستگی بہتر بناکر۔

Sensor calibration noise خصوصیات، biases، اور scale factors کی پیمائش کرتا ہے۔ IMU calibration accelerometer اور gyroscope biases، scale factors، اور axis alignments کا تعین کرتا ہے۔ Camera calibration intrinsics، distortion coefficients، اور robot base frames کے متعلقہ extrinsics کی پیمائش کرتا ہے۔

### Domain Adaptation

Domain adaptation تکنیکیں simulation اور حقیقی ڈیٹا کے درمیان distribution shift کو کم کرتی ہیں بغیر exact system identification کی ضرورت کے۔ یہ approaches synthetic ڈیٹا پر models کی تربیت کرتے ہیں جبکہ حقیقی distributions پر مختلف طریقہ کار کے ذریعے عمومی بنتے ہیں۔

Domain randomization متنوع synthetic ڈیٹا پر تربیت دیتا ہے policies کو distribution shift کے خلاف مضبوط بنانے کے لیے۔ اہم بصیرت یہ ہے کہ اگر حقیقی distribution تربیتی distribution کے اندر موجود ہے، تو منتقلی کامیاب ہوتی ہے۔ حقیقی تبدیلی مؤثر ہونے کے لیے randomization کو پھیلانا چاہیے۔

Domain adaptation networks domains کے درمیان نقشہ سیکھنے کے لیے سیکھتے ہیں۔ ایک encoder network حقیقی تصاویر کو synthetic تصاویر جیسا دکھانے کے لیے تبدیل کرتا ہے، یا اس کے برعکس۔ Policy تربیت پھر domain-adapted representations استعمال کرتی ہے، distribution gap کو کم کرتے ہوئے۔

Online adaptation تعیناتی کے دوران learning جاری رکھتا ہے۔ حقیقی دنیا کے تعاملات نیا تربیتی ڈیٹا فراہم کرتے ہیں جو distribution shift کے لیے درست کرتا ہے۔ اس کے لیے محتاط حفاظتی پابندیاں اور مستحکم learning rates کی ضرورت ہے policy کی خرابی کو روکنے کے لیے۔

### تعیناتی پائپ لائن

تربیت یافتہ policies کو جسمانی روبوٹس پر تعینات کرنے کے لیے تربیت یافتہ networks کو export اور optimize کرنے کی ضرورت ہے۔ Isaac PyTorch ماڈلز کو TensorRT engines میں تبدیل کرنے کے لیے ٹولز فراہم کرتا ہے مؤثر GPU inference کے لیے۔ Deployment پائپ لائن کو sensor input processing، policy inference، اور actuator command execution حقیقی وقت میں handle کرنا چاہیے۔

```python
# Example: Policy export and deployment for physical robot
# Udaharan: Physical robot کے لیے policy export اور deployment
import torch
import tensorrt as trt
import numpy as np
from typing import Dict, Tuple

class PolicyDeployer:
    """
    Deploy trained policies to physical robots using TensorRT.
    Handles model export, optimization, and inference.
    """

    def __init__(
        self,
        model: torch.nn.Module,
        input_shape: Tuple[int, ...],
        output_shape: Tuple[int, ...],
        engine_path: str = "policy.trt"
    ):
        self.model = model
        self.input_shape = input_shape
        self.output_shape = output_shape
        self.engine_path = engine_path

        # Set model to evaluation mode
        # Model ko evaluation mode par set karna
        self.model.eval()

        # TensorRT logger
        # TensorRT logger
        self.logger = trt.Logger(trt.Logger.WARNING)

    def export_to_tensorrt(
        self,
        precision: str = "fp16",
        max_batch_size: int = 1,
        max_workspace_size: int = 1 << 25
    ):
        """
        Export PyTorch model to TensorRT engine.

        Args:
            precision: "fp32", "fp16", or "int8"
            max_batch_size: Maximum batch size for inference
            max_workspace_size: Maximum workspace memory in bytes
        """
        import torch_tensorrt

        # Determine precision
        # Precision determine karna
        if precision == "fp16":
            enabled_precisions = {torch.float, torch.half}
        elif precision == "int8":
            enabled_precisions = {torch.float, torch.int8}
        else:
            enabled_precisions = {torch.float}

        # Compile model with TensorRT
        # TensorRT کے ساتھ model compile کرنا
        self.trt_module = torch_tensorrt.compile(
            self.model,
            inputs=[
                torch_tensorrt.Input(
                    shape=self.input_shape,
                    dtype=torch.float32
                )
            ],
            enabled_precisions=enabled_precisions,
            workspace_size=max_workspace_size,
            truncate_long_and_double=True
        )

        # Save engine
        # Engine save karna
        self.save_engine()

    def save_engine(self):
        """Save TensorRT engine to file."""
        # TensorRT engine ko file mein save karna
        # Get serialized engine
        # Serialized engine get karna
        engine = self.trt_module._engine

        with open(self.engine_path, "wb") as f:
            f.write(engine.serialize())

    def load_engine(self) -> trt.ICudaEngine:
        """Load TensorRT engine from file."""
        # TensorRT engine ko file se load karna
        with open(self.engine_path, "rb") as f:
            engine_data = f.read()

        runtime = trt.Runtime(self.logger)
        engine = runtime.deserialize_cuda_engine(engine_data)

        return engine

    def create_inference_session(self, engine: trt.ICudaEngine = None):
        """
        Create inference session with input/output buffers.
        """
        # Input/output buffers کے ساتھ inference session create کرنا
        if engine is None:
            engine = self.load_engine()

        self.engine = engine
        self.context = engine.create_execution_context()

        # Allocate buffers
        # Buffers allocate karna
        self.host_inputs = []
        self.gpu_inputs = []
        self.host_outputs = []
        self.gpu_outputs = []

        for i in range(engine.num_bindings):
            binding_name = engine.get_binding_name(i)
            shape = engine.get_binding_shape(i)
            dtype = trt.nptype(engine.get_binding_dtype(i))

            # Allocate host and device memory
            # Host aur device memory allocate karna
            size = trt.volume(shape)
            host_mem = np.empty(size, dtype=dtype)
            gpu_mem = torch.cuda.mem_alloc(size * dtype().itemsize)

            # Store bindings
            # Bindings store karna
            if engine.binding_is_input(binding_name):
                self.host_inputs.append(host_mem)
                self.gpu_inputs.append(gpu_mem)
            else:
                self.host_outputs.append(host_mem)
                self.gpu_outputs.append(gpu_mem)

        # Create CUDA stream
        # CUDA stream create karna
        self.stream = torch.cuda.current_stream().cuda_stream

    def infer(self, input_data: np.ndarray) -> np.ndarray:
        """
        Run inference with input data.

        Args:
            input_data: numpy array of shape input_shape

        Returns:
            output_data: numpy array of shape output_shape
        """
        # Copy input to GPU
        # Input ko GPU par copy karna
        np.copyto(self.host_inputs[0], input_data.flatten())
        torch.cuda.memcpy_async(
            self.gpu_inputs[0],
            torch.from_numpy(self.host_inputs[0]).cuda(),
            self.stream
        )

        # Bind buffers
        # Buffers bind karna
        bindings = [int(self.gpu_inputs[0]), int(self.gpu_outputs[0])]

        # Execute inference
        # Inference execute karna
        self.context.execute_async(
            bindings=bindings,
            stream_handle=self.stream
        )

        # Copy output from GPU
        # Output ko GPU se copy karna
        torch.cuda.memcpy_async(
            torch.from_numpy(self.host_outputs[0]).cuda(),
            self.gpu_outputs[0],
            self.stream
        )

        # Synchronize
        # Synchronize karna
        torch.cuda.synchronize()

        # Reshape output
        # Output reshape karna
        output = self.host_outputs[0].reshape(self.output_shape)

        return output


class RobotControlInterface:
    """
    Interface between Isaac policy and physical robot hardware.
    Handles sensor input processing and actuator command execution.
    """

    def __init__(
        self,
        deployer: PolicyDeployer,
        sensor_config: Dict,
        control_frequency: int = 100
    ):
        self.deployer = deployer
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency

        # Initialize sensor interfaces
        # Sensor interfaces initialize karna
        self.camera = self._init_camera(sensor_config["camera"])
        self.imu = self._init_imu(sensor_config["imu"])
        self.joint_encoder = self._init_joint_encoders(sensor_config["joints"])

        # Initialize actuator interface
        # Actuator interface initialize karna
        self.actuators = self._init_actuators()

        # State observation buffer
        # State observation buffer
        self.observation_history = []
        self.max_history = 10

    def _init_camera(self, config):
        """Initialize camera sensor interface."""
        # Camera sensor interface initialize karna
        # Implementation depends on specific camera hardware
        # Common interfaces: ROS 2 image transport, DirectShow, V4L2
        pass

    def _init_imu(self, config):
        """Initialize IMU sensor interface."""
        # IMU sensor interface initialize karna
        # Implementation depends on IMU hardware
        # Common interfaces: serial, I2C, SPI
        pass

    def _init_joint_encoders(self, config):
        """Initialize joint encoder interfaces."""
        pass

    def _init_actuators(self):
        """Initialize actuator interfaces for joint control."""
        pass

    def get_observation(self) -> np.ndarray:
        """
        Collect sensor data and construct observation vector.

        Returns:
            observation: Flattened observation vector for policy input
        """
        # Sensor data collect karna aur observation vector construct karna
        observations = []

        # Camera observation
        # Camera observation
        rgb = self.camera.capture()
        depth = self.camera.capture_depth()
        observations.append(self._preprocess_rgb(rgb))
        observations.append(self._preprocess_depth(depth))

        # IMU observation
        # IMU observation
        imu_reading = self.imu.read()
        observations.append(imu_reading)  # 6D acceleration + gyro

        # Joint state observation
        # Joint state observation
        joint_positions = self.joint_encoder.read_positions()
        joint_velocities = self.joint_encoder.read_velocities()
        observations.append(joint_positions)
        observations.append(joint_velocities)

        # Concatenate all observations
        # Saare observations concatenate karna
        observation = np.concatenate(observations)

        # Store in history
        # History mein store karna
        self.observation_history.append(observation)
        if len(self.observation_history) > self.max_history:
            self.observation_history.pop(0)

        return observation

    def step(self) -> Tuple[np.ndarray, bool]:
        """
        Execute one control step.

        Returns:
            observation: Current observation
            done: Whether episode should terminate
        """
        # Ek control step execute karna
        import time

        start_time = time.time()

        # Get observation
        # Observation get karna
        obs = self.get_observation()

        # Run policy inference
        # Policy inference run karna
        action = self.deployer.infer(obs)

        # Apply low-level control
        # Low-level control apply karna
        self._apply_joint_commands(action)

        # Check safety conditions
        # Safety conditions check karna
        done = self._check_safety_conditions()

        # Wait to maintain control frequency
        # Control frequency برقرار رکھنے کے لیے انتظار
        elapsed = time.time() - start_time
        if elapsed < self.control_period:
            time.sleep(self.control_period - elapsed)

        return obs, done

    def _apply_joint_commands(self, commands: np.ndarray):
        """Apply joint position/velocity/torque commands to actuators."""
        # Actuators par joint position/velocity/torque commands apply karna
        # Implementation depends on actuator interface
        # May involve CAN bus communication, EtherCAT, or custom protocols
        pass

    def _check_safety_conditions(self) -> bool:
        """Check for safety violations requiring emergency stop."""
        # Safety violations کی جانچ جو emergency stop کی ضرورت رکھتی ہیں
        # Joint limit checking
        # Joint limit checking
        joint_positions = self.joint_encoder.read_positions()
        if np.any(joint_positions < self.joint_limits["lower"]) or \
           np.any(joint_positions > self.joint_limits["upper"]):
            return True

        # Velocity limit checking
        # Velocity limit checking
        joint_velocities = self.joint_encoder.read_velocities()
        if np.any(np.abs(joint_velocities) > self.velocity_limits):
            return True

        # Force/torque checking
        # Force/torque checking
        # Additional safety checks...
        # Additional safety checks...

        return False
```

## 4.6 خلاصہ اور حصہ 5 سے تعلق

اس باب نے فزیکل AI ڈیولپمنٹ کے لیے NVIDIA Isaac پلیٹ فارم پر جامع جائزہ فراہم کیا ہے۔ آپ نے سیکھا ہے کہ کیسے Isaac Sim photorealistic رینڈرنگ اور درست physics کے ساتھ اعلیٰ معیار کی simulation کو ممکن بناتا ہے۔ Perception پائپ لائنز یہ ظاہر کرتی ہیں کہ کیسے AI پر مبنی کمپیوٹر ویژن کو روبوٹ سسٹمز میں مربوط کیا جا سکتا ہے۔ ہیرا پھیری اور reinforcement learning کے حصوں نے دکھایا ہے کہ کیسے Isaac Gym بڑے پیمانے پر parallelization کے ذریعے policy learning کو تیز کرتا ہے۔ آخر میں، sim-to-real منتقلی کی تکنیکیں simulation سے جسمانی تعیناتی تک پل فراہم کرتی ہیں۔

حصہ 5 - ہیومینائڈ روبوٹ ڈیزائن کی طرف بڑھتے ہوئے، آپ ان Isaac صلاحیتوں کو مخصوص ہیومینائڈ پلیٹ فارمز کے لیے ادراک اور کنٹرول سسٹمز تیار کرنے کے لیے لاگو کریں گے۔ یہاں simulation سیٹ اپ، policy تربیت، اور تعیناتی میں تیار کردہ مہارتیں براہ راست بعد کے ابواب میں شامل implementation چیلنجز کی حمایت کریں گی۔ Isaac کا ہیومینائڈ kinematics کے ساتھ انضمام آپ کو مکمل رویاتی نظام تیار اور جانچنے دے گا جسمانی تعیناتی سے پہلے۔

---

**اگلا باب:** حصہ 5 - ہیومینائڈ روبوٹ ڈیزائن

حصہ 5 میں، آپ اس باب سے simulation اور learning کی بنیادوں کو ہیومینائڈ روبوٹس کے لیے کنٹرول سسٹمز ڈیزائن اور لاگو کرنے میں استعمال کریں گے۔ موضوعات میں دو پیروں پر چلنے کی منصوبہ بندی، پورے جسم کے کنٹرول کی تعمیر، اور انسان-روبوٹ تعامل کے انٹرفیس شامل ہیں۔

