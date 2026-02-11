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
1. Adaptive color masking for yellow/white lanes
2. Canny edge detection with noise cleanup
3. ROI masking to isolate road region
4. Hough transform for line candidates
5. Polynomial lane fitting and temporal smoothing
6. Lane width stabilization and single-side synthesis
7. Vanishing point turn prediction
8. Video output with lane overlay
