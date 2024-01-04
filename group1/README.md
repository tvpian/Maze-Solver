# ENPM 809Y - RWA 3

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Authors

**Developer:** Obaid Ur Rahman

**Developer:** Rachit Thakur

**Maintainer:** Tharun V. Puthanveettil 

## Dependencies
* ROS2 Galactic

## Instructions to run the code

1. Place the package in the src folder of your colcon workspace.
2. Build the workspace using `colcon build`
3. Source the setup file using `source install/setup.bash`
4. Run the launch file using `ros2 launch simlaunch.py`


```
cd <path to colcon workspace>/src/
colcon build
source install/setup.bash
ros2 launch rwa3 rwa3.launch.py
```