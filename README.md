# State_Estimator

A standalone ROS Noetic package for Unscented Kalman Filter state estimation and Extended Kalman Filter, using a car dynamics model with a LuGre tyre model. This repository isolates the state estimation pipeline for independent compilation, testing, and offline playback via rosbag.

I would like to express my gratitude to Nuno Alexandre, who pioneered the development of the car model used in this repository. Additionally, while the Extended Kalman Filter includes custom modifications, the foundational concept and original codebase were not authored by me. I want to thank the original developers for the robust design of this state estimator, as well as everyone involved in its maintenance.

## Dependencies
To successfully build and run this package, ensure the following dependencies are installed or present in your catkin workspace:

**System / External Libraries:**
* `roscpp`
* `Eigen3` 
* `tf2` 

**Standard ROS Packages:**
* `std_msgs`
* `sensor_msgs`
* `geometry_msgs`
* `nav_msgs`

**Custom Workspace Packages:**
* `common_msgs`
* `node_metrics_monitor`
  
## How to Run

Open a terminal:

1. **cd <path_to_your_workspace>**
2. **catkin_make --source <path_to_the_repository>**
3. **source devel/setup.bash**
4. **roslaunch state_estimation state_estimation.launch**
5. **rosbag play <name_of_your_file>.bag** in another terminal
