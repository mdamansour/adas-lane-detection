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
* `src/`: Modular MATLAB source code.
    * `main_process_video.m`: Main entry point.
    * `*.m`: Helper functions (Thresholding, Edge Detection, Hough, etc.).
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

---
*Research conducted for academic demonstration.*
