# TAS2 simulator-ekf
The evaluate package is based on ROS2 and Ubuntu 22.04. 

## Implementation
- This package can evaluate filters, in this project they are the evaluation of UKF(unscented Kalman filter) and EKF(extended Kalman filter).
- This package calculates the odometry error before and after the filter. Position error and orientation error.
- The results can be visualized in real time while the error is being calculated.
- The dataset folder should be organized as follows.
    
  ```
  evaluate
  │   data
  │   test    
  │   resource
  └───evaluate
      │   for_evaluate.py
      │   eval_all.py
      │   pub_goal.py
      │   pub_goal2.py
      │   save_data.py
      │   ...

  ```


- The results of the evaluation and the visualization of the error curves:

  ![](./../image/eval1.gif)

- The results of comparing the two different algorithms:

  ![](./../image/eval2.gif)


## Acknowledgement
This project is not possible without multiple great opensourced codebases.
- [ ] [TAS2-simulator](https://gitlab.lrz.de/tas_2223/tas2-simulator).
- [ ] [Ros2 Humble](https://docs.ros.org/en/humble/index.html).
- [ ] [robot_localization]( https://github.com/cra-ros-pkg/robot_localization).
- [ ] [Navigation 2](https://navigation.ros.org/).