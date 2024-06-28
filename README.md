# Formula Student Driverless: Perception using lidar on EUFS sim
This repository aims to acheive 3D coordinates of cones placed on racetrack in Edinburg University Formula Student Simulator using a lidar pointcloud.
Lidar used is: Velodyne VLP-16R (5 layers) <br />
ROS version: ROS Noetic <br />
Ubuntu: 20.04 <br />


### Raw output of lidar
<img src="./images/rawpcd.png" alt="mc" width="400"/>

### After ground removal
<img src="./images/nogroundpcd.png" alt="mc" width="400"/>

### Final Coordinates
<img src="./images/coordinatespcd.png" alt="mc" width="400"/>
<img src="./images/coordinates.png" alt="mc" width="400"/> <br />

## Installation

### Downloading EUFS simulator 
1. cd catkin_ws/src
2. git clone https://github.com/eufsa/eufs_sim
3. sudo apt-get install ros-noetic-ackermann-msgs
4. ⁠sudo apt-get install ros-noetic-twist-mux
5. ⁠sudo apt-get install ros-noetic-joy
6. sudo apt-get install ⁠ros-noetic-controller-manager 
7. sudo apt-get install ⁠ros-noetic-velodyne-simulator 
8. ⁠sudo apt-get install ros-noetic-effort-controllers 
9. ⁠sudo apt-get install ros-noetic-velocity-controllers 
10. ⁠sudo apt-get install ros-noetic-joint-state-controller 
11. sudo apt-get install ⁠ros-noetic-gazebo-ros-control
12. ⁠sudo apt-get install ros-noetic-hector-gazebo-plugins
13. ⁠cd .. 
14. ⁠catkin_make
15. ⁠gedit src/eufs_sim/robot_control/nodes/twist_to_ackermannDrive.py
16. ⁠change the first line from python to python3

Now you should be able to do: <br />
roslaunch eufs_gazebo small_track.launch <br />
and see the car in its environment. <br />

In another terminal window, you can also do <br />
roslaunch robot_control rqt_robot_control.launch <br />
to control the car using slider joystick <br />

### Setting up this lidar package
1. sudo apt update
2. sudo apt-get install ros-noetic-pcl-conversions
3. sudo apt-get install ros-noetic-pcl-ros
4. sudo apt-get install python3-pip
5. pip3 install open3d
6. pip3 install cython
7. sudo apt update
8. sudo apt install python3-pcl
9. go to location: /usr/lib/python3/dist-packages. Then copy the folders “pcl” and “python_pcl-0.3.egg-info”. Go to Home then .local/lib/python3.8/site-packages. Delete the folder named “pcl”. Copy here the two folders from before. Restart the pc.
10. cd catkin_ws/src
11. git clone https://github.com/harshsrivastava0/lidar.git
12. chmod +x lidar/ground_removal/ransac3d_main.py
13. chmod +x lidar/clustering/cluster_extraction_dbscan.py
14. chmod +x lidar/pose_estimation/cone_pose.py
15. Make sure to change the relative paths given in lidar/clustering/cluster_extraction_dbscan.py and lidar/pose_estimation/cone_pose.py according to your own pc.
16. cd ..
17. catkin_make

Now after launching a track in EUFS simulator. Start an empty terminal and launch: <br />
roslaunch ground_removal main.launch <br />

The final coordinates of the cones are published on the topic /cone_pose (IMPORTANT: coordinates are given assuming lidar faces left) <br />
You can view the coordinated using: <br />
rostopic echo /cone_pose <br />

To publish this, two custom message types arrofarr and arr have been used. The structure for which is present in lidar/clustering/msg
