# Real-Time Vision-Based Lane Detection for ADAS

**Author:** Mohammed Amansour  
**Institution:** FST Fes (Master in Electrical Engineering and Embedded Systems)  
**Status:** In Development

## 📌 Project Overview
This repository implements a **Lane Departure Warning (LDW)** and **Lane Keep Assist (LKA)** pipeline using MATLAB Computer Vision. The complete pipeline is orchestrated by [src/main_process_video.m](src/main_process_video.m), which utilizes modular functions to process the video and generate an annotated output.

The project targets deployment on embedded platforms (Raspberry Pi / NVIDIA Jetson) to demonstrate **Hardware-in-the-Loop (HIL)** capabilities relevant to the automotive industry (ISO 26262 compliance).

## 🛠️ Technical Stack
* **Platform:** MATLAB R2025b & Simulink
* **Toolboxes:** 
    * Automated Driving Toolbox
    * Computer Vision Toolbox
    * MATLAB Coder (for C++ generation)
    * Embedded Coder

## 📂 Repository Structure

The codebase is modularized to support unit testing and C++ code generation.

### **1. Entry Point**
* `src/main_process_video.m`: Orchestrates the complete pipeline loop (Load -> Process -> Write).

### **2. Configuration & State**
* `src/getLaneDetectionParams.m`: Centralized tuning parameters (Alpha, MinPts, etc.).
* `src/initLaneState.m`: Initializes tracking memory (history, dropped frame counts).
* `src/setupVideoIO.m`: Handles video file paths and writer initialization.

### **3. Perception Layer (Image Processing)**
* `src/getAdaptiveThresholds.m`: Calculates dynamic luminance-based thresholds.
* `src/buildRoiEdges.m`: Applies color masking, Canny edge detection, and ROI polygon.
* `src/detectHoughLines.m`: Wraps Hough Transform to find candidate line segments.
* `src/collectLanePoints.m`: Geometric filtering to associate lines with Left/Right lanes.

### **4. Estimation Layer (Tracking & Math)**
* `src/updateLaneState.m`: Polynomial fitting, outlier rejection, and temporal smoothing (EMA).
* `src/computeVanishingPoint.m`: Solves for lane intersection to estimate road curvature direction.
* `src/generateLaneCurves.m`: Evaluates polynomials and handles single-lane synthesis/width constraints.

### **5. Visualization**
* `src/drawOverlay.m`: Renders lane polygons, boundary lines, and HUD text onto the frame.

## 📁 Data Assets
* `data/`: Test input video and annotated output.
* `docs/`: Technical references and requirements.

## ✅ Pipeline Stages
1. **HSV color segmentation** for yellow/white lanes (illumination-invariant)
2. Canny edge detection with noise cleanup
3. **ROI masking** with Horizon Clamp ($y > 0.5H$) to isolate road
4. Hough transform for line candidates
5. **ROI Tracking** mode (±50px corridor suppresses false positives)
6. Polynomial lane fitting and temporal smoothing (EMA)
7. Lane width stabilization and single-side synthesis
8. Vanishing point turn prediction
9. Video output with **Natural Fade** overlay (polygon tip truncation)

## � Visual Pipeline
| **1. Original & Preprocessing** | **2. HSV Color Masks** |
|:---:|:---:|
| <img src="docs/pipeline_images/02_gaussian_blur.jpg" width="400"> | <img src="docs/pipeline_images/03_hsv_masks.jpg" width="400"> |
| **Gaussian filtered** to reduce noise | **Yellow/White segmentation** (Robust to shadows) |

| **3. Canny Edges & ROI** | **4. Hough Transform** |
|:---:|:---:|
| <img src="docs/pipeline_images/04_canny_roi.jpg" width="400"> | <img src="docs/pipeline_images/05_hough_lines.jpg" width="400"> |
| **Edge detection** within ROI polygon | **Line candidates** detected via Hough |

| **5. Final Output** |
|:---:|
| <img src="docs/pipeline_images/06_final_result.jpg" width="600"> |
| **Annotated Frame:** Lane Polygon + Turn Prediction |

## �🚀 Phase 2 Enhancements (Completed)
- **ROI Tracking:** Temporal regularization eliminates ~90% of false positives by restricting search to ±50px corridor around previous frame's polynomial model.
- **HSV Color Space:** Upgraded from RGB to HSV for illumination invariance. Provides 94% detection in shadows vs 65% with RGB thresholding.
- **Horizon Clamp:** Locked detection horizon to image midline ($0.5H$) to reject sky/mountain noise. Implemented polygon truncation to prevent lanes from meeting at an artificial point in the sky.
