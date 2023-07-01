# TAS2 simulator-ukf
The tas-simulator-ukf package is based on ROS2 and Ubuntu 22.04. 

## Implementation
- This package can display maps and cars in rviz and gazebo. It can also enable autonomous navigation when given a goal pose. In this package, the algorithm that implements robot localization is UKF(unscented Kalman filter). 
- In here, ukf is implemented as the localization algorithm for autonomous navigation. Replace the original ekf algorithm in the [TAS2-simulator](https://gitlab.lrz.de/tas_2223/tas2-simulator) package.
- If you want to achieve autonomous navigation, it is necessary to use the [evaluate](https://gitlab.lrz.de/tas_2223/tas-project/group_10/tas-simulator/-/tree/main/evaluate) package to publish the goal pose.
- - The dataset folder should be organized as follows.
    
  ```
  tas2-simulator-ukf
   │   config
   │   launch
   |   maps
   |   models
   |   rviz
   |   CMakeLists.txt
   │   package.xml
   │   ...
  ```

## Acknowledgement
This project is not possible without multiple great opensourced codebases.
- [ ] [TAS2-simulator](https://gitlab.lrz.de/tas_2223/tas2-simulator).
- [ ] [Ros2 Humble](https://docs.ros.org/en/humble/index.html).
- [ ] [robot_localization]( https://github.com/cra-ros-pkg/robot_localization).
- [ ] [Navigation 2](https://navigation.ros.org/).