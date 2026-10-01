# Computer Vision Pipeline Options for LitterBot

## Executive Summary

This document analyzes different computer vision approaches for autonomous litter detection on the LitterBot platform. The CV pipeline runs on the Jetson Orin Nano and must balance **accuracy**, **speed**, and **power consumption** for real-time autonomous operation.

**Key Requirements:**
- Detect litter objects in real-time (indoor/outdoor environments)
- Classify litter type (recyclable vs trash, optionally by material)
- Provide 3D position for navigation and collection
- Run efficiently on Jetson Orin Nano (embedded platform)
- Integrate with ROS 2 navigation stack

---

## Table of Contents
1. [Object Detection Architectures](#object-detection-architectures)
2. [Camera Hardware Options](#camera-hardware-options)
3. [Depth Estimation Methods](#depth-estimation-methods)
4. [ROS 2 Integration Strategies](#ros-2-integration-strategies)
5. [Dataset and Training Options](#dataset-and-training-options)
6. [Jetson Optimization Techniques](#jetson-optimization-techniques)
7. [Recommended Pipeline](#recommended-pipeline)

---

## Object Detection Architectures

### Option 1: YOLO Family (You Only Look Once)

**Overview:**
- Single-stage detector (no region proposals)
- Directly predicts bounding boxes and class probabilities
- Industry standard for real-time object detection

**Versions:**

| Version | Release | Speed (FPS) | Accuracy (mAP) | Notes |
|---------|---------|-------------|----------------|-------|
| YOLOv5 | 2020 | ~140 FPS | 50.7 mAP | Mature, well-documented, PyTorch |
| YOLOv8 | 2023 | ~120 FPS | 53.9 mAP | Latest Ultralytics, best accuracy |
| YOLO-NAS | 2023 | ~100 FPS | 52.2 mAP | Neural Architecture Search optimized |
| YOLOv10 | 2024 | ~130 FPS | 54.5 mAP | Newest, fewer tested deployments |

**Pros:**
- **Fastest inference** among accurate detectors
- **Excellent community support** (Ultralytics, ROS 2 packages)
- **Easy to train** on custom datasets
- **Multiple model sizes** (nano, small, medium, large, extra-large)
- **TensorRT support** for Jetson optimization
- **Proven track record** in robotics applications

**Cons:**
- Less accurate than two-stage detectors (but usually sufficient)
- Struggles with very small objects (< 32×32 pixels)
- Can have more false positives than Faster R-CNN

**Best For:**
- Real-time robotics applications (like LitterBot)
- When speed matters more than perfect accuracy
- Embedded platforms (Jetson, edge devices)

**Recommended Variant:** **YOLOv8** (best balance of speed, accuracy, and support)

---

### Option 2: Faster R-CNN (Region-Based CNN)

**Overview:**
- Two-stage detector (region proposal + classification)
- Higher accuracy but slower than single-stage detectors

**Specifications:**
- Speed: ~7-15 FPS (GPU-dependent)
- Accuracy: 55-60 mAP (higher than YOLO)
- Framework: PyTorch (Detectron2), TensorFlow

**Pros:**
- **Higher accuracy** than single-stage detectors
- **Better at detecting small objects**
- **Fewer false positives**
- **Well-established in research**

**Cons:**
- **Slower inference** (~10x slower than YOLO)
- May not meet real-time requirements on Jetson
- More complex to train and deploy
- Higher power consumption

**Best For:**
- Applications where accuracy is paramount
- Offline processing or batch detection
- Stationary cameras (not mobile robots)

**Verdict for LitterBot:** **Not recommended** - too slow for real-time mobile robot

---

### Option 3: EfficientDet

**Overview:**
- Efficient architecture using EfficientNet backbone
- Good balance of speed and accuracy

**Specifications:**
- Speed: ~40 FPS (D0 variant), ~15 FPS (D4 variant)
- Accuracy: 34-55 mAP (depending on model size)
- Framework: TensorFlow/Keras

**Pros:**
- **Good accuracy-to-speed ratio**
- **Efficient architecture** (fewer parameters)
- **Scalable** (D0 to D7 variants)

**Cons:**
- **Less popular than YOLO** (fewer ROS 2 examples)
- TensorFlow-based (PyTorch more common in ROS 2)
- TensorRT conversion more complex

**Best For:**
- Projects already using TensorFlow
- When model size must be minimized

**Verdict for LitterBot:** NOTE: **Viable alternative** if YOLO doesn't work, but second choice

---

### Option 4: SSD (Single Shot Detector)

**Overview:**
- Early single-stage detector (2016)
- Predecessor to YOLO/EfficientDet

**Specifications:**
- Speed: ~50-60 FPS
- Accuracy: 25-30 mAP (lower than modern detectors)

**Pros:**
- Fast inference
- Simple architecture

**Cons:**
- **Outdated** - superseded by YOLO and EfficientDet
- Lower accuracy than modern alternatives
- Poor small object detection

**Verdict for LitterBot:** **Not recommended** - use YOLOv8 instead

---

### Option 5: Instance Segmentation (Mask R-CNN, YOLACT)

**Overview:**
- Provides pixel-level masks for each object (not just bounding boxes)
- Useful if you need precise object boundaries

**Use Cases:**
- Grasping oddly-shaped litter
- Measuring litter size
- Avoiding fragmentation when picking up

**Pros:**
- **Precise object boundaries**
- Can separate overlapping objects
- Better for manipulation tasks

**Cons:**
- **Slower than object detection** (2-5x overhead)
- More complex to train
- Higher computational cost

**Verdict for LitterBot:** NOTE: **Optional enhancement** - start with bounding boxes, upgrade if needed

---

## Camera Hardware Options

### Option 1: RGB Camera Only (e.g., USB Webcam, CSI Camera)

**Examples:**
- Logitech C920 (USB, 1080p, $50)
- Raspberry Pi Camera Module v3 (CSI, 12MP, $25)
- Jetson IMX477 Camera (CSI, 12MP, $70)

**Pros:**
- **Lowest cost** ($25-$70)
- **Simplest setup** (single camera)
- **Lightweight** (minimal hardware)

**Cons:**
- **No depth information** (need monocular depth estimation)
- Hard to judge distance to litter
- 2D bounding boxes only

**Best For:**
- Proof-of-concept / early prototypes
- Budget-constrained projects
- When depth is not critical

**Verdict for LitterBot:** NOTE: **Good starting point**, but depth needed for collection

---

### Option 2: Stereo Camera (e.g., ZED, Intel RealSense D435)

**Examples:**
- **Intel RealSense D435** (stereo + IR, 1280×720, $200)
- **ZED 2** (stereo, 1080p, $450)
- **ZED 2i** (stereo + IMU, 1080p, $600)

**Pros:**
- **Direct depth measurement** (no ML model needed)
- **Good outdoor performance** (passive stereo)
- **RGB + depth aligned**
- **Wide field of view** (ZED: 110°)
- **ROS 2 support** (official packages)

**Cons:**
- **Higher cost** ($200-$600)
- Stereo requires good lighting and texture
- Depth accuracy degrades with distance (>5m)

**Best For:**
- Mobile robots needing accurate depth
- Outdoor environments (sunlight compatible)
- When you need both detection + localization

**Verdict for LitterBot:** **Highly recommended** - Intel RealSense D435 is best value

---

### Option 3: LiDAR + Camera Fusion

**Setup:**
- 2D or 3D LiDAR (already on robot for SLAM)
- RGB camera for object detection
- Sensor fusion to combine detections with 3D points

**Examples:**
- RPLIDAR A1 (2D, $99) + USB Camera
- Velodyne Puck Lite (3D, $5000+) + USB Camera

**Pros:**
- **Already have LiDAR** for navigation (reuse hardware)
- **Accurate 3D positioning**
- Works in low light (LiDAR independent of lighting)

**Cons:**
- **Complex sensor fusion** (camera-LiDAR calibration required)
- LiDAR doesn't provide color (need camera for classification)
- Higher computational overhead

**Best For:**
- When LiDAR already required for navigation
- Outdoor/industrial environments

**Verdict for LitterBot:** NOTE: **Advanced option** - start with stereo camera, add LiDAR fusion later if needed

---

### Option 4: Event Camera (e.g., Prophesee, iniVation)

**Overview:**
- Neuromorphic cameras that only report pixel changes
- Ultra-high temporal resolution (microseconds)

**Pros:**
- No motion blur
- Extremely low latency
- Good in dynamic lighting

**Cons:**
- **Very expensive** ($1000+)
- **Limited software ecosystem**
- Requires specialized algorithms (not compatible with YOLO)
- No color information

**Verdict for LitterBot:** **Not recommended** - overkill for this application

---

## Depth Estimation Methods

### Method 1: Stereo Depth (Hardware)

**How it works:**
- Two cameras separated by baseline
- Triangulation to compute depth
- Passive (no active illumination)

**Pros:**
- No ML model needed (geometric calculation)
- Works outdoors (sunlight doesn't interfere)
- Real-time performance

**Cons:**
- Requires camera calibration
- Fails in textureless regions (e.g., white walls)
- Depth range limited (~10m for D435)

**Best For:** Real-time robotics (like LitterBot)

---

### Method 2: Monocular Depth Estimation (ML-Based)

**How it works:**
- Single RGB camera
- Deep learning model predicts depth from single image
- Trained on large datasets (NYU Depth, KITTI)

**Popular Models:**
- **MiDaS** (Intel, robust general-purpose depth)
- **DPT** (Vision Transformer-based)
- **Depth-Anything** (latest, 2024)

**Pros:**
- Only need one camera (cheap)
- Can work where stereo fails (textureless regions)

**Cons:**
- **Not real depth** (relative depth only, scale ambiguous)
- Inference overhead (adds 20-50ms per frame)
- Less accurate than stereo (10-20% error)

**Best For:**
- Budget prototypes (single camera)
- Augmenting stereo depth

**Verdict for LitterBot:** NOTE: **Backup option** if stereo camera not in budget

---

### Method 3: LiDAR Point Cloud Projection

**How it works:**
- Use LiDAR point cloud for 3D scene
- Project camera image onto point cloud
- Associate detections with 3D points

**Pros:**
- Highly accurate 3D localization
- Works in any lighting condition

**Cons:**
- Complex calibration (extrinsic camera-LiDAR)
- Sparse point cloud (gaps between points)
- Higher computational cost

**Best For:** Autonomous vehicles, outdoor robots with 3D LiDAR

---

## ROS 2 Integration Strategies

### Strategy 1: vision_msgs + Custom Detector Node

**Architecture:**
```
Camera Node (image_raw)
    ↓
YOLO Detector Node (custom)
    ↓
vision_msgs/Detection2DArray
    ↓
Litter Localization Node
    ↓
geometry_msgs/PoseStamped
    ↓
Navigation Stack
```

**Pros:**
- Full control over detector
- Standard ROS 2 message types
- Easy to debug and visualize (RViz2)

**Cons:**
- Must write custom YOLO wrapper node

**Recommended Messages:**
- `sensor_msgs/Image` - raw camera feed
- `vision_msgs/Detection2DArray` - bounding boxes + class
- `vision_msgs/Detection3DArray` - 3D localized objects
- `geometry_msgs/PoseStamped` - target litter position

---

### Strategy 2: Use Existing ROS 2 Packages

**Option A: darknet_ros (YOLO wrapper)**
- GitHub: leggedrobotics/darknet_ros
- Supports YOLOv3/v4 (older versions)
- Publishes `darknet_ros_msgs/BoundingBoxes`

**Option B: yolov8_ros (Ultralytics wrapper)**
- GitHub: mgonzs13/yolov8_ros
- Modern, supports YOLOv8
- Publishes `yolov8_msgs/DetectionArray`

**Option C: perception_pcl (3D segmentation)**
- ROS 2 official package
- For LiDAR-based segmentation
- More complex, not needed for camera-only

**Recommendation:** **yolov8_ros** (actively maintained, modern YOLO)

---

### Strategy 3: TensorRT Accelerated Pipeline

**Flow:**
```
Camera → Preprocessor → TensorRT Engine → Postprocessor → ROS 2 Publisher
```

**Steps:**
1. Train YOLOv8 in PyTorch
2. Export to ONNX format
3. Convert ONNX to TensorRT engine (Jetson-optimized)
4. Wrap TensorRT inference in ROS 2 node

**Pros:**
- **5-10x speedup** on Jetson (vs PyTorch)
- Lower latency (20-40ms per frame)
- Lower power consumption

**Cons:**
- More complex setup
- TensorRT models are hardware-specific (must rebuild for each Jetson)

**Tools:**
- **torch2trt** - PyTorch to TensorRT converter
- **tao-toolkit** - NVIDIA Transfer Learning Toolkit
- **onnx-tensorrt** - ONNX to TensorRT

**Verdict:** **Essential for production** - do this after initial prototyping

---

## Dataset and Training Options

### Option 1: TACO Dataset (Trash Annotations in Context)

**Details:**
- 1500 images, 60 litter categories
- Real-world trash in outdoor environments
- Bounding box annotations

**Pros:**
- **Free and open-source**
- Realistic litter (bottles, cans, wrappers, etc.)
- Ready for YOLO training

**Cons:**
- **Small dataset** (1500 images not enough for best accuracy)
- Limited variety (mostly urban litter)
- No depth labels

**Use Case:** Starting point for transfer learning

**Link:** http://tacodataset.org/

---

### Option 2: Custom Dataset Collection

**Approach:**
- Drive LitterBot around campus/test area
- Record video with onboard camera
- Manually annotate litter in frames
- Augment with synthetic data

**Tools:**
- **Roboflow** - annotation + augmentation platform (free tier available)
- **LabelImg** - free bounding box annotation tool
- **CVAT** - free video annotation tool

**Recommended Pipeline:**
1. Collect 500-1000 images from LitterBot's perspective
2. Annotate using Roboflow or CVAT
3. Augment (flip, rotate, brightness, blur) → 5000+ images
4. Split: 70% train, 20% val, 10% test
5. Train YOLOv8 with transfer learning (start from COCO weights)

**Pros:**
- **Tailored to your environment** (campus-specific litter)
- Matches camera perspective and lighting
- Can add custom classes (e.g., "coffee_cup", "snack_wrapper")

**Cons:**
- **Time-consuming** (10-30 sec per image annotation)
- Requires domain expertise (what counts as "litter"?)

**Recommendation:** **Combine TACO + Custom** - fine-tune TACO-trained model on your data

---

### Option 3: Synthetic Data Generation

**Approach:**
- Use Unity, Blender, or NVIDIA Isaac Sim to generate synthetic images
- Automatic annotation (ground truth from simulator)

**Pros:**
- **Unlimited data** (generate millions of images)
- Perfect annotations (no human error)
- Control lighting, backgrounds, object types

**Cons:**
- **Sim-to-real gap** (synthetic images don't look like real world)
- Time-consuming to set up
- Requires 3D models of litter objects

**Tools:**
- **NVIDIA Isaac Sim** (ROS 2 integrated, free for research)
- **Unity Perception Package** (free, good documentation)

**Recommendation:** NOTE: **Advanced technique** - use for data augmentation, not primary dataset

---

## Jetson Optimization Techniques

### 1. TensorRT Engine Conversion

**Impact:** 5-10x speedup
**Effort:** Medium

```bash
# Export YOLOv8 to ONNX
yolo export model=yolov8n.pt format=onnx

# Convert ONNX to TensorRT
trtexec --onnx=yolov8n.onnx --saveEngine=yolov8n.trt --fp16
```

**Benefits:**
- Layer fusion (reduce memory transfers)
- Kernel auto-tuning (optimize for Jetson GPU)
- Mixed precision (FP16 for speed, FP32 where needed)

---

### 2. Model Quantization (INT8)

**Impact:** 2-3x additional speedup (after TensorRT)
**Effort:** High (requires calibration dataset)

**Process:**
1. Collect 500-1000 representative images
2. Run TensorRT quantization calibration
3. Generate INT8 engine

**Trade-off:**
- Faster inference (40-50 FPS on Jetson Orin Nano)
- ~1-2% accuracy loss

---

### 3. Input Resolution Tuning

**Trade-off Matrix:**

| Resolution | FPS (YOLOv8n) | mAP | Use Case |
|------------|---------------|-----|----------|
| 320×320 | ~60 FPS | 35% | Fast, close-range |
| 416×416 | ~45 FPS | 42% | Balanced |
| 640×640 | ~30 FPS | 50% | Best accuracy |

**Recommendation:** **Start with 416×416**, tune based on testing

---

### 4. Model Size Selection

**YOLOv8 Variants:**

| Model | Parameters | Speed | Accuracy | Recommendation |
|-------|------------|-------|----------|----------------|
| YOLOv8n (nano) | 3.2M | ~60 FPS | 37% mAP | **Start here** |
| YOLOv8s (small) | 11.2M | ~45 FPS | 44% mAP | If nano insufficient |
| YOLOv8m (medium) | 25.9M | ~30 FPS | 50% mAP | Too slow for real-time |

**Recommendation:** **YOLOv8n** with TensorRT should hit 40-60 FPS on Jetson Orin Nano

---

### 5. Power Mode Configuration

**Jetson Orin Nano Power Modes:**

| Mode | Max Power | GPU Freq | CPU Cores | Use Case |
|------|-----------|----------|-----------|----------|
| 15W | 15W | 625 MHz | 6 cores | Battery operation |
| 25W | 25W | 1020 MHz | 6 cores | **Recommended** |
| MAXN | 40W | 1020 MHz | 6 cores | AC power only |

**Set power mode:**
```bash
sudo nvpmodel -m 1  # 25W mode
sudo jetson_clocks   # Lock clocks to max
```

---

## Recommended Pipeline

### Phase 1: Proof-of-Concept (Weeks 1-2)

**Goal:** Prove litter detection works

**Hardware:**
- USB webcam or Jetson CSI camera ($25-70)
- Jetson Orin Nano in 25W mode

**Software:**
1. Use **YOLOv8n** pre-trained on COCO dataset
2. Test with PyTorch (no optimization yet)
3. Detect "bottle", "cup", "can" classes from COCO
4. Publish to ROS 2 using `vision_msgs`

**Expected Performance:**
- 15-20 FPS (PyTorch, no TensorRT)
- 60-70% detection accuracy on common litter

**Validation:**
- Place known litter objects in front of camera
- Verify bounding boxes appear in RViz2

---

### Phase 2: Depth Integration (Weeks 3-4)

**Goal:** Localize litter in 3D space

**Hardware Upgrade:**
- Add **Intel RealSense D435** stereo camera ($200)

**Software:**
1. Use RealSense ROS 2 package (`realsense2_camera`)
2. Align RGB and depth frames
3. For each detected bounding box:
   - Sample depth at box center
   - Convert to 3D point in camera frame
   - Transform to robot base frame (tf2)
4. Publish `vision_msgs/Detection3DArray`

**Expected Performance:**
- 20-30 FPS (depth alignment overhead)
- 3D localization accurate to ±5cm at 1-2m range

**Validation:**
- Place litter at known distances (0.5m, 1m, 2m)
- Compare detected distance to ground truth

---

### Phase 3: Custom Training (Weeks 5-6)

**Goal:** Improve accuracy on campus-specific litter

**Process:**
1. Collect 500 images from LitterBot camera
2. Annotate using Roboflow (classes: bottle, can, wrapper, cup, etc.)
3. Combine with TACO dataset (1500 images)
4. Augment to 5000+ images
5. Fine-tune YOLOv8n (start from COCO weights)
6. Validate on held-out test set

**Expected Performance:**
- 80-90% detection accuracy on campus litter
- Same speed as Phase 1 (model size unchanged)

**Validation:**
- Drive robot around campus
- Manually verify detection accuracy (100 test cases)

---

### Phase 4: Optimization (Weeks 7-8)

**Goal:** Maximize speed for real-time operation

**Process:**
1. Export YOLOv8n to ONNX
2. Convert to TensorRT (FP16)
3. Benchmark on Jetson
4. Optionally: INT8 quantization if FP16 insufficient

**Expected Performance:**
- **40-60 FPS** (TensorRT FP16)
- **60-80 FPS** (TensorRT INT8, if needed)
- Latency: 20-30ms per frame

**Validation:**
- Stress test: drive robot at max speed, verify detection rate
- Measure power consumption (should stay under 25W)

---

### Phase 5: Multi-Object Tracking (Weeks 9-10)

**Goal:** Track litter across frames (don't re-detect same object)

**Approach:**
- Use **BoT-SORT** or **DeepSORT** tracking algorithm
- Assign unique IDs to detected litter
- Maintain track history (object persistence)

**Benefits:**
- Smoother trajectory planning (don't jitter between frames)
- Avoid re-collecting same litter
- Count unique litter items (not just detections)

**ROS 2 Integration:**
- Publish `vision_msgs/Detection3DArray` with tracking IDs
- Maintain persistent litter map in global frame

---

### Phase 6: Production Deployment (Week 11+)

**Final System:**

```
┌─────────────────────────────────────────┐
│ Intel RealSense D435 (RGB-D Camera)     │
└────────────┬────────────────────────────┘
             │ 30 FPS, 1280×720
             ↓
┌─────────────────────────────────────────┐
│ YOLOv8n TensorRT Engine (Jetson)        │
│ - FP16 precision                         │
│ - 416×416 input                          │
│ - 50 FPS inference                       │
└────────────┬────────────────────────────┘
             ↓
┌─────────────────────────────────────────┐
│ 3D Localization Node                     │
│ - Depth alignment                        │
│ - Camera → Robot transform (tf2)        │
└────────────┬────────────────────────────┘
             ↓
┌─────────────────────────────────────────┐
│ Multi-Object Tracker (BoT-SORT)         │
│ - Track persistence                      │
│ - Unique litter IDs                      │
└────────────┬────────────────────────────┘
             ↓
┌─────────────────────────────────────────┐
│ Litter Collection Planner                │
│ - Prioritize closest litter              │
│ - Plan approach path (Nav2)             │
│ - Trigger collection mechanism (STM32)  │
└─────────────────────────────────────────┘
```

**Performance Targets:**
- Detection rate: **80%+** (validated on campus)
- False positive rate: **<10%**
- Latency: **<50ms** end-to-end (camera → detection → 3D pose)
- Power: **<25W** total (Jetson + camera)

---

## Alternative Approaches

### Approach 1: Classical Computer Vision (No Deep Learning)

**Techniques:**
- Color segmentation (HSV thresholding for bright wrappers)
- Edge detection (Canny)
- Shape matching (Hough circles for bottles/cans)

**Pros:**
- No training data needed
- Fast (100+ FPS)
- Deterministic behavior

**Cons:**
- **Extremely brittle** (fails in varied lighting, backgrounds)
- Poor generalization (must hand-tune for each litter type)
- Can't classify litter types

**Verdict:** **Not recommended** for real-world autonomous system

---

### Approach 2: Vision-Language Models (e.g., CLIP, OWL-ViT)

**Concept:**
- Use models that understand natural language
- Detect "anything that looks like trash"
- No training data required (zero-shot)

**Examples:**
- **OWL-ViT** (Google, open-vocabulary detection)
- **GroundingDINO** (zero-shot object detection)

**Pros:**
- No training data needed
- Can detect novel litter types (not seen in training)
- Flexible (just change text prompt)

**Cons:**
- **Very slow** (500ms - 2s per frame)
- Requires powerful GPU (not real-time on Jetson)
- Lower accuracy than trained YOLO

**Verdict:** NOTE: **Research direction** - not ready for real-time robotics yet

---

## Cost-Benefit Analysis

| Option | Hardware Cost | Training Effort | Speed | Accuracy | Recommendation |
|--------|---------------|-----------------|-------|----------|----------------|
| RGB + YOLOv8 + Monocular Depth | $50 | Medium | ★★★★☆ | ★★★☆☆ | Budget option |
| RealSense D435 + YOLOv8 | $250 | Medium | ★★★★★ | ★★★★☆ | **Best choice** |
| LiDAR + Camera Fusion | $500+ | High | ★★★☆☆ | ★★★★★ | Advanced/future |
| Classical CV (no ML) | $50 | Low | ★★★★★ | ★☆☆☆☆ | Not robust |

---

## Next Steps

### Immediate (This Week):
1. Order Intel RealSense D435 camera ($200 on Amazon)
2. Download TACO dataset and YOLOv8 pre-trained weights
3. Set up ROS 2 workspace on Jetson

### Short-Term (Next Month):
4. Run YOLOv8 inference on sample images (verify it works)
5. Integrate RealSense with ROS 2 (`realsense2_camera` package)
6. Write simple detector node (subscribe to images, publish detections)
7. Visualize in RViz2

### Medium-Term (2-3 Months):
8. Collect custom dataset (500+ images from LitterBot perspective)
9. Fine-tune YOLOv8 on TACO + custom data
10. Convert to TensorRT for Jetson optimization
11. Integrate with Nav2 for litter approach

### Long-Term (3-6 Months):
12. Add multi-object tracking (DeepSORT)
13. Implement collection success validation (did we pick it up?)
14. Autonomous coverage path planning (lawn mower pattern)

---

## References

**Papers:**
- YOLOv8: Ultralytics YOLOv8 Documentation (2023)
- Faster R-CNN: Ren et al., NeurIPS 2015
- EfficientDet: Tan et al., CVPR 2020

**Datasets:**
- TACO: http://tacodataset.org/
- COCO: https://cocodataset.org/

**ROS 2 Packages:**
- realsense2_camera: https://github.com/IntelRealSense/realsense-ros
- yolov8_ros: https://github.com/mgonzs13/yolov8_ros
- vision_msgs: https://github.com/ros-perception/vision_msgs

**Jetson Optimization:**
- TensorRT: https://developer.nvidia.com/tensorrt
- Jetson Zoo: https://elinux.org/Jetson_Zoo

---

## Conclusion

**Recommended Pipeline for LitterBot:**

1. **Object Detection:** YOLOv8n (nano) with TensorRT FP16
2. **Camera Hardware:** Intel RealSense D435 (RGB-D)
3. **Depth Method:** Hardware stereo (from RealSense)
4. **Training Data:** TACO dataset + 500 custom images
5. **ROS 2 Integration:** Custom detector node publishing `vision_msgs`
6. **Optimization:** TensorRT on Jetson Orin Nano (25W mode)

**Expected Performance:**
- 40-60 FPS detection
- 80%+ accuracy on campus litter
- <50ms latency
- <25W power consumption

**Total Additional Cost:** ~$200 (RealSense camera only - Jetson already in system)

This pipeline balances **cost**, **performance**, and **ease of implementation** for a senior capstone project timeline.
