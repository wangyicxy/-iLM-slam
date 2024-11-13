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



