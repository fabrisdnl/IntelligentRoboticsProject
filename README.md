# Group 09

## Members
* **Andrea Campagnol**: [andrea.campagnol.1@studenti.unipd.it](mailto:andrea.campagnol.1@studenti.unipd.it)

* **Daniele Fabris** [daniele.fabris.5@studenti.unipd.it](mailto:daniele.fabris.5@studenti.unipd.it)

# ASSIGNMENT 1

## Setting up the environment 

Clone this repository inside your `catkin_ws/src/` folder.
Make sure to have the `tiago_iaslab_simulation` folder inside the same folder.

## Commands to run the simulation

* ### Local machine 
    - Open at least 5 terminal windows
    - In each window run: 
        ```
            cd ~/path_to_catkin_ws
        ```

* ### VLab
    - Open at least 5 terminal windows
    - In each window run: 
        ```
        start_tiago
        source /opt/ros/noetic/setup.bash
        source /tiago_public_ws/devel/setup.bash
        source ~/catkin_ws/devel/setup.bash
        ```

#### 1. Terminal window
```
cd ~/catkin_ws
catkin build
```

#### 2. Terminal window
```
roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library
```
Wait until the startup is completed.

#### 3. Terminal window
```
roslaunch tiago_iaslab_simulation navigation.launch
```

#### 4. Terminal window
```
rosrun assignment_1 client_tiago XP YP ZP XQ YQ ZQ WQ
```
Coordinates Example: 11 1.2 0.0 0.0 0.0 -0.3 0.5

#### 5. Terminal window
```
rosrun assignment_1 server_tiago ALGORITHM
```
ALGORITHM argument can be: `detection` or `detectionCV`

## Optional windows and results visualization

### Access to Tiago live camera feed

#### 6. Terminal window (Optional: access to tiago camera)
```
rosrun assignment_1 tiago_vision
```

### Visualize distance/obstacles map

The detectCV algorithm generates a distance map, a background map and an obstacle map in order to visualize and better understand the laser scan data and detect obstacles. 
To visualize them remove the comments to the `cv::imwrite(...)` in `detectCV.cpp` file.
The outputs can be found in you home folder. 

## Known Problems

- Sometimes the simulation does not properly initialize, and laserscan is not shown. Just kill and restart. 
- Sometimes the openCV modules cause some problem, in particular dealing with the GUI. Just kill and restart. 

# ASSIGNMENT 2

## Setting up the environment 

Clone this repository inside your `catkin_ws/src/` folder.
Make sure to have the `apriltag`, `apriltag_ros`, `gazebo_ros_link_attacher` and `tiago_iaslab_simulation` folders inside the same folder.
If you run the simulation in Vlab `apriltag` and `apriltag_ros` are not required. 

## Commands to run the simulation

* ### Local machine 
    - Open a terminal windows and run: 
        ```
        cd ~/path_to_catkin_ws
        roslaunch assignment_2 assignment_2.launch
        ```

* ### VLab
    - Open a terminal windows and run:  
        ```
        start_tiago
        source /opt/ros/noetic/setup.bash
        source /tiago_public_ws/devel/setup.bash
        source ~/catkin_ws/devel/setup.bash
        roslaunch assignment_2 assignment_2.launch
        ```

# Suggestions

We suggest to run `rqt_console` and `rqt` to follow step by step the arm action pipeline and visualize the camera data.
In `rqt` -> `Plugins` -> `Visualization` -> `Image View`


