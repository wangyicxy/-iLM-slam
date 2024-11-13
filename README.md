# -iLM-slam
SLAM Algorithms Suitable for Mining Scenarios.

This document provides setup and configuration instructions for several ROS packages used in sensor integration, localization, and mapping.

## Environment Dependencies

Ensure the following dependencies are installed:

- **OpenCV**: Default version (comes with ROS)
- **GSL**: 2.6
- **Eigen**: 3.3.4
- **Ceres**: 1.14.0
- **PCL**: 1.8 (comes with ROS)

---
## Start fusion localization algorithm
Package Name: precisedoc

### To Start:
```bash
cd /precisedoc
source devel/setup.bash
roslaunch precisedocking precise_docking.launch
```

## Visual Localization Setup Guide
Package Name: vio_precisedoc

### To Start:
```bash
cd /vio_precisedoc
source devel/setup.bash
roslaunch precisedocking precise_docking.launch
```


## QR Code Detection
This document outlines the setup and configuration instructions for visual localization using QR code detection and visual-inertial odometry (VIO) in ROS.
Package Name: `apriltag3`

### To Start:
```bash
cd /apriltag3
source devel/setup.bash
roslaunch apriltag_ros continuous_detection.launch
```

## Fusion Localization
Package Name:/fus_precisedoc

### To Start:
```bash
cd /fus_precisedoc
source devel/setup.bash
roslaunch precisedocking fus_precise_docking.launch
```


