# Real-Time Vision-Based Lane Detection for ADAS

**Author:** Mohammed Amansour  
**Institution:** FST Fes (Master in Electrical Engineering and Embedded Systems)  
**Status:** In Development

## 📌 Project Overview
This repository implements a **Lane Departure Warning (LDW)** and **Lane Keep Assist (LKA)** system using the MATLAB Automated Driving Toolbox. It is designed to process video feeds in real-time, apply Bird's-Eye View (BEV) transformations, and fit lane boundaries using robust estimation techniques (RANSAC/Sliding Window).

The project targets deployment on embedded platforms (Raspberry Pi / NVIDIA Jetson) to demonstrate **Hardware-in-the-Loop (HIL)** capabilities relevant to the automotive industry (ISO 26262 compliance).

## 🛠️ Technical Stack
* **Platform:** MATLAB R2025b & Simulink
* **Toolboxes:** 
    * Automated Driving Toolbox
    * Computer Vision Toolbox
    * MATLAB Coder (for C++ generation)
    * Embedded Coder

## 📂 Repository Structure
* `src/`: Core algorithms and Simulink models.
* `data/`: Calibration data and test scenarios.
* `docs/`: Technical references and requirements.

---
*Research conducted for academic demonstration.*
