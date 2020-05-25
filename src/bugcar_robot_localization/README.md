# Bugcar Robot Localization PACKAGE
## What is this repo?
This repo contains the source of the original [ROS robot_localization](https://github.com/cra-ros-pkg/robot_localization), and more important, the config and launch files for Bugcar's robot_localization package
## What would I do if I had a bug?
Check closed issuses first, then if your problem had not been asked, raised a new one!
### Common bugs:
 - **TF jumps or drifts:**
    - If the tf drifts, but the odom topics do not experience erractic jumps, then the robot_localization is trying to focalize the origin of the odom frame with respect to the map frame
    - Make sure that only map_ekf and odom_ekf publish tf via:
      **~$ rostopic info tf**
    - The covariance matrices of all observation sources (imu, odom, gps) are diagonal
 - **Utm frame is not in the same branch as odom frame**
    - Make sure that the roboclaw always publish **odom/wheel** topic even when first initialize.
    - Increase **delay** param in navsat_transform_node
## Important
- Understand the REP aka ROS' standard for reference frames, orientation and units!!!
- Make use of *rqt_graph* (for check relationship between ROS nodes), *rostopic*, *rosnode*, and *rosrun tf view_frames*
- The folder is named bugcar_robot_localization. However, the package name **is still `robot_localization`**. This misunderstand can happen when trying to run the package
## How to tune robot_localization?
Reference to [robot_localization wiki](http://docs.ros.org/melodic/api/robot_localization/html/index.html)
- To set topics for data subscripting and publising: edit *remap* in *launch/bugcar_robot_localization.launch*
- To configure params: *params/dual_ekf_navsat.yaml*
- **Important: do not forget to set a suitable "datum" param in "navsat_transform" node that is at the origin of map frame_id**
## Ideal relationship between tf frames
![alt text](https://github.com/tranqkhue/bugcar/blob/master/src/bugcar_robot_localization/doc/images/ideal_tf_frame.jpg)
## What is "dual_ekf_navsat"?
According to [ROS REP 105](https://www.ros.org/reps/rep-0105.html), there are 3 main Coordinate Frames:
- *base_link*: the vehicle itself, with x-axis toward, y-axis to the left, and z-axis updown
- *odom*: [Ref](https://answers.ros.org/question/237295/confused-about-coordinate-frames-can-someone-please-explain/) "It is the frame where the odometry is used to update the pose of the robot. In this frame, there is no correction or jumps in the robot's pose, it is continually updated based on the data coming from the motor's encoders (sometimes fused with accelerometer and gyro)." The odometry is integral-ed to get the position, which is relative (as +C in integral equations) as the origin of this frame is where the robot_localization package starts. Odom frame is used in short-term as it will be smooth and continuous (due to nature of high data input). However, they have two limitations: the frame is not aligned to any fixed geographical frame, and odometry data will drift over time.
- *map*: It's the frame that is fixed over time, and the tf between this frame and other frames should be by absolute sensors (for example, GPS data is absolute on Earth's geographical coordinates, or compass is fixed to Earth's magnetic coordinates)
We need TWO **INDEPENDENT** EKF for *base_link->odom* and *base_link->map*. We may only need one *base_link->map* as our robot is working outdoor and there is usually GPS signal, but due to compatibility with ROS' REP and ROS' ecosystem (motion planners...), we need those two. 
## Sensor fusion vs odom/wheel only:
Green line is from odom/wheel, red/yellow line is from map_se/odom_se EKF
![alt text](https://github.com/tranqkhue/bugcar/blob/master/src/bugcar_robot_localization/doc/screen_shot_1.png)
