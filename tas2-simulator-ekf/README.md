# TAS2 simulator-ekf
The tas-simulator-ekf package is based on ROS2 and Ubuntu 22.04. 

## Implementation
- This package can display maps and cars in rviz and gazebo. It can also enable autonomous navigation when given a goal pose. In this package, the algorithm that implements robot localization is EKF(extended Kalman filter). This package is almost the same as the one in [TAS2-simulator](https://gitlab.lrz.de/tas_2223/tas2-simulator).
- If you want to achieve autonomous navigation, it is necessary to use the [evaluate](https://gitlab.lrz.de/tas_2223/tas-project/group_10/tas-simulator/-/tree/main/evaluate) package to publish the goal pose.

- The dataset folder should be organized as follows.
    
  ```
  tas2-simulator-ekf
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
