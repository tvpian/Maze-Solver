# Maze Solver
## Description
In this ROS2-based implementation, a turtlebot3 performs autonomous navigation in a maze based on the cues recieved from Aruco markers placed on the walls of the maze. The turtlebot3 is equipped with two cameras. The primary camera is used to detect aruco markers to obtain the embedded directions for navigation. The secondary camera is used to detect and record the pose of the parts floating randomly in the air.

## Dependencies
- ROS2 Galactic
- ArUco marker detection requires the latest version of OpenCV.
```bash
  - pip3 uninstall opencv-python
  - pip3 uninstall opencv-contrib-python
  - pip3 install opencv-contrib-python
  - pip3 install opencv-python
```
- To install ROS packages for this demo, it is recommended to create a new
workspace.
```bash
  - mkdir -p ~/maze_ws/src
  - cd ~/maze_ws/src
  - git clone https://github.com/tvpian/Maze_Solver.git
  - rosdep install --from-paths src -y --ignore-src
  - colcon build
```

## Usage
- [Link to usage](./group1/README.md)
- Note: Considering all the packages including the group1_final cloned in the same workspace, Please follow only the instructions under the "Instructions to run the code" section in the link above.

## Demo
[demo.webm](https://github.com/tvpian/Maze_Runner/assets/41953267/9c2e4b79-4b36-48e2-995b-bce9027824f6)
