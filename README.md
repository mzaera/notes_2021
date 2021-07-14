HUSKY SIMULATION ON MY PC
--------------------------

In each terminal:

    cd catkin_ws
    source devel/setup.bash


Terminal 1 (launch simulation):

    roslaunch agriculture_launcher bringup.launch


Terminal 2 (move the robot):
By rostopic pub:

    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -5.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

By teleop:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

By husky_nav pkg:

    rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'map'
    pose:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.707
        w: 0.707"

Terminal 3 (run rviz and open the configuration file):

    rosrun rviz rviz -d ~/catkin_ws/src/agriculture_sim/src/rviz/rviz_husky_config.rviz


HUSKY SIMULATION ON DOCKER
--------------------------

Start docker:

    docker start ingeniarius-simulators

In each terminal:

    docker exec -it ingeniarius-simulators bash
    cd agriculture_sim/
    source devel/setup.bash

Terminal 1 (launch simulation).

Terminal 2 (move the robot).

Terminal 3 (run rviz and open the configuration file).


USEFUL LINKS
------------

Docker:

[Docker instalation guide](https://hub.docker.com/r/ingeniarius/ingeniarius-simulators)

Gazebo:

[Git MAS-UAV pkg](https://github.com/Rezenders/mas_uav)

[Git Husky pkg](https://github.com/husky/husky)

[Agriculture envairoment](https://www.clearpathrobotics.com/assets/guides/kinetic/husky/additional_sim_worlds.html)

RTABMAP:

[ RTAB-Map Webpage](http://introlab.github.io/rtabmap/)

[ RTAB-Map Forum](http://official-rtab-map-forum.67519.x6.nabble.com/)

[ROS Wiki RTAB-Map](http://wiki.ros.org/rtabmap_ros)

Robot Localization:

[Robot_localization GIT](https://github.com/cra-ros-pkg/robot_localization)

[Robot_localization Wiki](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)

[Robot_localization Params](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html)

[Data robot_localization](http://docs.ros.org/en/indigo/api/robot_localization/html/preparing_sensor_data.html)

Octomap:

[Octomap_server Wiki](http://wiki.ros.org/octomap_server)

Others:

[Covariance matrices](https://manialabs.wordpress.com/2012/08/06/covariance-matrices-with-a-practical-example/)

[Quaternions and Euler angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

[Quaternions and Euler angles Online Transform](https://quaternions.online/)

CHANGES DONE
------------

* /home/developer/agriculture_sim/src/configurations/robot_localization/navsat_transform.yaml

Line 4 aprox.
    *Adding an yaw offset correcting that way the wrong orientation on rviz and robot_localization pkg*

    yaw_offset: 1.570796

* /home/developer/agriculture_sim/src/configurations/robot_localization/ekf_global.yaml
    *Increase the rejection threshold (to inf) to fix the wrongs lectures while rotatting*

        odom0_pose_rejection_threshold: 50
        odom0_twist_rejection_threshold: 50
        imu0_pose_rejection_threshold: 50                 
        imu0_twist_rejection_threshold: 50               
        imu0_linear_acceleration_rejection_threshold: 50  

* /home/developer/agriculture_sim/src/configurations/robot_localization/ekf_local.yaml
    *Increase the rejection threshold (to inf) to fix the wrongs lectures while rotatting*

        odom0_pose_rejection_threshold: 50
        odom0_twist_rejection_threshold: 50
        imu0_pose_rejection_threshold: 50                 
        imu0_twist_rejection_threshold: 50               
        imu0_linear_acceleration_rejection_threshold: 50 

* /home/developer/agriculture_sim/src/agriculture_launcher/bringup.launch

    *Add: octomap_server launch file, husky_navigation launch file and the new rtabmap launch file*

```bash
<launch>
    <include file="$(find cpr_agriculture_gazebo)/launch/agriculture_world.launch">
      <arg name="platform" value="husky" />
    </include> 

    <include file="$(find agriculture_launcher)/robot_localization/localization_local.launch"/>
    <include file="$(find agriculture_launcher)/robot_localization/localization_global.launch"/>

    <include file="$(find agriculture_launcher)/rtabmap/rtabmap_simulation_husky.launch"/>

    <include file="$(find agriculture_launcher)/octomap/octomap_server_start.launch"/>  
    <include file="$(find husky_navigation)/launch/move_base.launch">
        <arg name="no_static_map" value="true"/>
    </include>

</launch>

```

* /home/developer/agriculture_sim/src/agriculture_launcher

*Generate octomap launch file*
    
    mkdir octomap
    cd octomap

*Generate a file called "octomap_server_start.launch"*

    <launch>
        <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
            <param name="resolution" value="0.05" />
            
            <!-- fixed map frame (set to 'map' if SLAM or localization running!) -->
            <param name="frame_id" type="string" value="map" />
            
            <!-- maximum range to integrate (speedup!) -->
            <param name="sensor_model/max_range" value="5.0" />
            
            <!-- data source to integrate (PointCloud2) -->
            <remap from="cloud_in" to="/rtabmap/octomap_occupied_space" />
        
        </node>
    </launch>

* /home/developer/agriculture_sim/src/agriculture_launcher/rtabmap

*Generate a file called "rtabmap_simulation_husky.launch". You can obtain it from this repo*


* /home/developer/agriculture_sim/src/rviz

*Copy from this repo the file called "rviz_husky_config.rviz"*

GIT COMMANDS
------------

Obtain the repo:

    git clone https://github.com/mzaera/notes_2021

Update the repo:

    git add --all
    git commit -m "readme"
    git push

DOCKER COMMANDS
--------------
* Start docker:

```bash
docker start ingeniarius-simulators
```
* Stop the docker container (my pc terminal):

```bash
docker container stop ingeniarius-simulators
```

* Delate and rekame the docker:
(run.bash on the folder)

```bash
docker container stop ingeniarius-simulators
docker container rm ingeniarius-simulators
docker pull ingeniarius/ingeniarius-simulators
sudo chmod +x run.bash
./run.bash
```

* Rename docker container:
```bash
docker rename ingeniarius-simulators ingeniarius-mzX
```

FILE FROM PC TO DOCKER
----------------------

*(on the docker terminal)*

    mkdir marti

*Get the path of a folder:*
    
    pwd

*To obtain the container ID:*
*(on my pc terminal)*

    docker container ls -a

*General command:*

    docker cp  /host/local/path/file <containerId>:/file/path/in/container/

*My command:*

    docker cp /home/mzaera/Documents/map_test_1.rviz ea379b3ce13d:/home/developer/marti

FILE FROM DOCKER TO PC
----------------------

*(on my pc terminal)*

*General command:*

    docker cp <containerId>:/file/path/in/container/file /host/local/path/

*My command:*

    docker cp 304712a249f8:/home/developer/marti/map_test_1.rviz /home/mzaera/Documents/
    docker cp 304712a249f8:/home/developer/agriculture_sim/src/configurations/robot_localization/navsat_transform.yaml /home/mzaera/Documents/

PKG FROM DOCKER TO PC
---------------------
Create a Catkin Workspace:
    
    cd
    source /opt/ros/melodic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

Copy the folder from docker to catkin_ws/src:

    docker cp <containerId>:/file/path/in/container/ /host/local/path/

    docker cp 304712a249f8:/home/developer/agriculture_sim /home/mzaera/catkin_ws/src

Install the dependencies:

    rosdep install --from-paths src --ignore-src --rosdistro melodic -y
    catkin_make

Install the missing pkgs:

    sudo apt install ros-melodic-octomap-server
    sudo apt install ros-melodic-rtabmap-ros

INSTALL SUBLIME
---------------

Install sublime

    wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
    sudo apt-get install apt-transport-https
    echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
    sudo apt-get update
    sudo apt-get install sublime-text

TELEOP
------

Install:

    sudo apt install ros-melodic-teleop-twist-keyboard

Use (normal):

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Use (remapping):

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=name_of_the_new_topic

EXTRA NOTES
-----------

Find where is a pkg:

    rospack find <name_of_the_pkg> 

See the especific params:

    rtabmap --params | grep Name/

Root mode start:

    sudo su

Root mode end:

    exit

INFO/EXTRA LIST
---------------

* DynamicParams:

    Odometry parameters:

        left wheel radius multiplier: 1
        right wheel radius multiplier: 1
        wheel separation multiplier: 1.875

    Publication parameters:

        Publish executed velocity command: disabled
        Publication rate: 50
        Publish frame odom on tf: disabled


ODOM INPUTS
-----------

* rostopic echo -n1 /rtabmap/odom_rgbd_icp

*Using ICP config on RTABMAP*

```bash
header: 
  seq: 43
  stamp: 
    secs: 45
    nsecs: 456000000
  frame_id: "odom"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  covariance: [9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0]
---
```
* rostopic echo -n1 /rtabmap/odom_rgbd_icp

*Using RGBD config on RTABMAP (maybe the pointcloud rotation comes from w=1.0)*

```bash
header: 
  seq: 1
  stamp: 
    secs: 43
    nsecs: 287000000
  frame_id: "odom"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0 
  covariance: [19998.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 19998.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 19998.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 19998.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 19998.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 19998.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0]
---
```


* rostopic echo -n1 /ekf/gps_converted_odom

```bash
header: 
  seq: 179
  stamp: 
    secs: 50
    nsecs: 367000000
  frame_id: "map"
child_frame_id: ''
pose: 
  pose: 
    position: 
      x: 0.198646071833
      y: -7.19143222272
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [1.0000000000000002e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0000000000000002e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

* rostopic echo -n1 /imu/data

```bash
header: 
  seq: 531
  stamp: 
    secs: 51
    nsecs: 660000000
  frame_id: "base_link"
orientation: 
  x: 0.00418615480244
  y: 0.00507176713806
  z: 0.0327327948792
  w: 0.999442503311
orientation_covariance: [2.6030820491461885e-07, 0.0, 0.0, 0.0, 2.6030820491461885e-07, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: 0.00648587704519
  y: -0.0216170857248
  z: -0.00175590506428
angular_velocity_covariance: [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
linear_acceleration: 
  x: -0.0942494459401
  y: 0.0858529754922
  z: 9.79793773383
linear_acceleration_covariance: [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
---
```

ROSNODE LIST
------------

* /base_controller_spawner
* /ekf_global
* /ekf_local
* /gazebo
* /gazebo_gui
* /navsat_transform_node
* /realsense_to_laserscan
* /robot_state_publisher
* /rosout
* /rtabmap/icp_odometry
* /rtabmap/rtabmap
* /rviz_1625154693621032645
* /twist_marker_server
* /twist_mux


INICIAL PARAMETERS
------------------
 * /ekf_global/base_link_frame: base_link
 * /ekf_global/debug: False
 * /ekf_global/frequency: 30
 * /ekf_global/imu0: imu/data
 * /ekf_global/imu0_config: [False, False, Fa...
 * /ekf_global/imu0_differential: False
 * /ekf_global/imu0_linear_acceleration_rejection_threshold: 0.8
 * /ekf_global/imu0_nodelay: False
 * /ekf_global/imu0_pose_rejection_threshold: 0.8
 * /ekf_global/imu0_queue_size: 5
 * /ekf_global/imu0_relative: False
 * /ekf_global/imu0_remove_gravitational_acceleration: True
 * /ekf_global/imu0_twist_rejection_threshold: 0.8
 * /ekf_global/initial_estimate_covariance: [1.0, 0, 0, 0, 0,...
 * /ekf_global/map_frame: map
 * /ekf_global/odom0: rtabmap/odom_rgbd...
 * /ekf_global/odom0_config: [True, True, True...
 * /ekf_global/odom0_differential: True
 * /ekf_global/odom0_nodelay: True
 * /ekf_global/odom0_pose_rejection_threshold: 5
 * /ekf_global/odom0_queue_size: 5
 * /ekf_global/odom0_relative: False
 * /ekf_global/odom0_twist_rejection_threshold: 1
 * /ekf_global/odom1: ekf/gps_converted...
 * /ekf_global/odom1_config: [True, True, True...
 * /ekf_global/odom1_differential: False
 * /ekf_global/odom1_nodelay: True
 * /ekf_global/odom1_queue_size: 2
 * /ekf_global/odom1_relative: False
 * /ekf_global/odom_frame: odom
 * /ekf_global/print_diagnostics: True
 * /ekf_global/process_noise_covariance: [1.0, 0, 0, 0, 0,...
 * /ekf_global/publish_tf: True
 * /ekf_global/sensor_timeout: 0.1
 * /ekf_global/transform_time_offset: 0.05
 * /ekf_global/transform_timeout: 0.0
 * /ekf_global/two_d_mode: True
 * /ekf_global/use_control: False
 * /ekf_global/world_frame: map
 * /ekf_local/base_link_frame: base_link
 * /ekf_local/debug: False
 * /ekf_local/frequency: 30
 * /ekf_local/imu0: imu/data
 * /ekf_local/imu0_config: [False, False, Fa...
 * /ekf_local/imu0_differential: False
 * /ekf_local/imu0_linear_acceleration_rejection_threshold: 0.8
 * /ekf_local/imu0_nodelay: False
 * /ekf_local/imu0_pose_rejection_threshold: 0.8
 * /ekf_local/imu0_queue_size: 5
 * /ekf_local/imu0_relative: False
 * /ekf_local/imu0_remove_gravitational_acceleration: True
 * /ekf_local/imu0_twist_rejection_threshold: 0.8
 * /ekf_local/initial_estimate_covariance: ['1e-9', 0, 0, 0,...
 * /ekf_local/map_frame: map
 * /ekf_local/odom0: rtabmap/odom_rgbd...
 * /ekf_local/odom0_config: [False, False, Fa...
 * /ekf_local/odom0_differential: False
 * /ekf_local/odom0_nodelay: False
 * /ekf_local/odom0_pose_rejection_threshold: 5
 * /ekf_local/odom0_queue_size: 5
 * /ekf_local/odom0_relative: False
 * /ekf_local/odom0_twist_rejection_threshold: 1
 * /ekf_local/odom_frame: odom
 * /ekf_local/print_diagnostics: True
 * /ekf_local/process_noise_covariance: ['1e-3', 0, 0, 0,...
 * /ekf_local/publish_tf: True
 * /ekf_local/sensor_timeout: 0.1
 * /ekf_local/transform_time_offset: 0.05
 * /ekf_local/transform_timeout: 0.0
 * /ekf_local/two_d_mode: True
 * /ekf_local/use_control: False
 * /ekf_local/world_frame: odom
 * /gazebo/enable_ros_network: True
 * /husky_joint_publisher/publish_rate: 50
 * /husky_joint_publisher/type: joint_state_contr...
 * /husky_velocity_controller/angular/z/has_acceleration_limits: True
 * /husky_velocity_controller/angular/z/has_velocity_limits: True
 * /husky_velocity_controller/angular/z/max_acceleration: 6.0
 * /husky_velocity_controller/angular/z/max_velocity: 2.0
 * /husky_velocity_controller/base_frame_id: base_link
 * /husky_velocity_controller/cmd_vel_timeout: 0.25
 * /husky_velocity_controller/enable_odom_tf: False
 * /husky_velocity_controller/estimate_velocity_from_position: False
 * /husky_velocity_controller/left_wheel: ['front_left_whee...
 * /husky_velocity_controller/linear/x/has_acceleration_limits: True
 * /husky_velocity_controller/linear/x/has_velocity_limits: True
 * /husky_velocity_controller/linear/x/max_acceleration: 3.0
 * /husky_velocity_controller/linear/x/max_velocity: 1.0
 * /husky_velocity_controller/pose_covariance_diagonal: [0.001, 0.001, 0....
 * /husky_velocity_controller/publish_rate: 50
 * /husky_velocity_controller/right_wheel: ['front_right_whe...
 * /husky_velocity_controller/twist_covariance_diagonal: [0.001, 0.001, 0....
 * /husky_velocity_controller/type: diff_drive_contro...
 * /husky_velocity_controller/velocity_rolling_window_size: 2
 * /husky_velocity_controller/wheel_radius_multiplier: 1.0
 * /husky_velocity_controller/wheel_separation_multiplier: 1.875
 * /navsat_transform_node/broadcast_utm_transform: True
 * /navsat_transform_node/broadcast_utm_transform_as_parent_frame: True
 * /navsat_transform_node/datum: [49.8999999596, 8...
 * /navsat_transform_node/delay: 3
 * /navsat_transform_node/frequency: 30
 * /navsat_transform_node/magnetic_declination_radians: 0.0349066
 * /navsat_transform_node/publish_filtered_gps: False
 * /navsat_transform_node/transform_timeout: 0.1
 * /navsat_transform_node/use_odometry_yaw: True
 * /navsat_transform_node/wait_for_datum: True
 * /navsat_transform_node/yaw_offset: 1.570796
 * /navsat_transform_node/zero_altitude: True
 * /octomap_server/frame_id: map
 * /octomap_server/resolution: 0.05
 * /octomap_server/sensor_model/max_range: 500.0
 * /realsense_to_laserscan/angle_increment: 0.005
 * /realsense_to_laserscan/angle_max: 0.759218224618
 * /realsense_to_laserscan/angle_min: -0.759218224618
 * /realsense_to_laserscan/concurrency_level: 1
 * /realsense_to_laserscan/max_height: 1.0
 * /realsense_to_laserscan/min_height: 0.05
 * /realsense_to_laserscan/range_max: 8.0
 * /realsense_to_laserscan/range_min: 0.105
 * /realsense_to_laserscan/scan_time: 0.3333
 * /realsense_to_laserscan/target_frame: base_link
 * /realsense_to_laserscan/tolerance: 1.0
 * /realsense_to_laserscan/use_inf: True
 * /rosdistro: melodic
 * /rosversion: 1.14.11
 * /rtabmap/icp_odometry/config_path: 
 * /rtabmap/icp_odometry/expected_update_rate: 0.0
 * /rtabmap/icp_odometry/frame_id: base_link
 * /rtabmap/icp_odometry/ground_truth_base_frame_id: 
 * /rtabmap/icp_odometry/ground_truth_frame_id: 
 * /rtabmap/icp_odometry/guess_frame_id: 
 * /rtabmap/icp_odometry/guess_min_rotation: 0.0
 * /rtabmap/icp_odometry/guess_min_translation: 0.0
 * /rtabmap/icp_odometry/max_update_rate: 0.0
 * /rtabmap/icp_odometry/odom_frame_id: odom
 * /rtabmap/icp_odometry/publish_tf: False
 * /rtabmap/icp_odometry/queue_size: 10
 * /rtabmap/icp_odometry/scan_cloud_max_points: 0
 * /rtabmap/icp_odometry/wait_for_transform_duration: 0.2
 * /rtabmap/icp_odometry/wait_imu_to_init: False
 * /rtabmap/rtabmap/Grid/FromDepth: True
 * /rtabmap/rtabmap/Mem/IncrementalMemory: true
 * /rtabmap/rtabmap/Mem/InitWMWithAllNodes: false
 * /rtabmap/rtabmap/Optimizer/Iterations: 100
 * /rtabmap/rtabmap/approx_sync: True
 * /rtabmap/rtabmap/config_path: 
 * /rtabmap/rtabmap/database_path: ~/.ros/rtabmap.db
 * /rtabmap/rtabmap/frame_id: base_link
 * /rtabmap/rtabmap/gen_scan: False
 * /rtabmap/rtabmap/ground_truth_base_frame_id: 
 * /rtabmap/rtabmap/ground_truth_frame_id: 
 * /rtabmap/rtabmap/landmark_angular_variance: 9999.0
 * /rtabmap/rtabmap/landmark_linear_variance: 0.0001
 * /rtabmap/rtabmap/latch: True
 * /rtabmap/rtabmap/map_always_update: True
 * /rtabmap/rtabmap/map_cleanup: False
 * /rtabmap/rtabmap/map_frame_id: map
 * /rtabmap/rtabmap/odom_frame_id: odom
 * /rtabmap/rtabmap/odom_sensor_sync: False
 * /rtabmap/rtabmap/odom_tf_angular_variance: 1.0
 * /rtabmap/rtabmap/odom_tf_linear_variance: 1.0
 * /rtabmap/rtabmap/publish_tf: False
 * /rtabmap/rtabmap/queue_size: 10
 * /rtabmap/rtabmap/scan_cloud_max_points: 0
 * /rtabmap/rtabmap/subscribe_depth: False
 * /rtabmap/rtabmap/subscribe_odom_info: True
 * /rtabmap/rtabmap/subscribe_rgb: False
 * /rtabmap/rtabmap/subscribe_rgbd: False
 * /rtabmap/rtabmap/subscribe_scan: False
 * /rtabmap/rtabmap/subscribe_scan_cloud: True
 * /rtabmap/rtabmap/subscribe_scan_descriptor: False
 * /rtabmap/rtabmap/subscribe_stereo: False
 * /rtabmap/rtabmap/subscribe_user_data: False
 * /rtabmap/rtabmap/wait_for_transform_duration: 0.2
 * /twist_mux/locks: [{'topic': 'e_sto...
 * /twist_mux/topics: [{'topic': 'joy_t...
 * /use_sim_time: True



COMPLETE PARAMETERS LIST RTABMAP
--------------------------------

* /rtabmap/BRIEF/Bytes
* /rtabmap/BRISK/Octaves
* /rtabmap/BRISK/PatternScale
* /rtabmap/BRISK/Thresh
* /rtabmap/Bayes/FullPredictionUpdate
* /rtabmap/Bayes/PredictionLC
* /rtabmap/Bayes/VirtualPlacePriorThr
* /rtabmap/Db/TargetVersion
* /rtabmap/DbSqlite3/CacheSize
* /rtabmap/DbSqlite3/InMemory
* /rtabmap/DbSqlite3/JournalMode
* /rtabmap/DbSqlite3/Synchronous
* /rtabmap/DbSqlite3/TempStore
* /rtabmap/FAST/CV
* /rtabmap/FAST/Gpu
* /rtabmap/FAST/GpuKeypointsRatio
* /rtabmap/FAST/GridCols
* /rtabmap/FAST/GridRows
* /rtabmap/FAST/MaxThreshold
* /rtabmap/FAST/MinThreshold
* /rtabmap/FAST/NonmaxSuppression
* /rtabmap/FAST/Threshold
* /rtabmap/FREAK/NOctaves
* /rtabmap/FREAK/OrientationNormalized
* /rtabmap/FREAK/PatternScale
* /rtabmap/FREAK/ScaleNormalized
* /rtabmap/GFTT/BlockSize
* /rtabmap/GFTT/K
* /rtabmap/GFTT/MinDistance
* /rtabmap/GFTT/QualityLevel
* /rtabmap/GFTT/UseHarrisDetector
* /rtabmap/GMS/ThresholdFactor
* /rtabmap/GMS/WithRotation
* /rtabmap/GMS/WithScale
* /rtabmap/GTSAM/Optimizer
* /rtabmap/Grid/3D
* /rtabmap/Grid/CellSize
* /rtabmap/Grid/ClusterRadius
* /rtabmap/Grid/DepthDecimation
* /rtabmap/Grid/DepthRoiRatios
* /rtabmap/Grid/FlatObstacleDetected
* /rtabmap/Grid/FootprintHeight
* /rtabmap/Grid/FootprintLength
* /rtabmap/Grid/FootprintWidth
* /rtabmap/Grid/FromDepth
* /rtabmap/Grid/GroundIsObstacle
* /rtabmap/Grid/MapFrameProjection
* /rtabmap/Grid/MaxGroundAngle
* /rtabmap/Grid/MaxGroundHeight
* /rtabmap/Grid/MaxObstacleHeight
* /rtabmap/Grid/MinClusterSize
* /rtabmap/Grid/MinGroundHeight
* /rtabmap/Grid/NoiseFilteringMinNeighbors
* /rtabmap/Grid/NoiseFilteringRadius
* /rtabmap/Grid/NormalK
* /rtabmap/Grid/NormalsSegmentation
* /rtabmap/Grid/PreVoxelFiltering
* /rtabmap/Grid/RangeMax
* /rtabmap/Grid/RangeMin
* /rtabmap/Grid/RayTracing
* /rtabmap/Grid/Scan2dUnknownSpaceFilled
* /rtabmap/Grid/ScanDecimation
* /rtabmap/GridGlobal/AltitudeDelta
* /rtabmap/GridGlobal/Eroded
* /rtabmap/GridGlobal/FootprintRadius
* /rtabmap/GridGlobal/FullUpdate
* /rtabmap/GridGlobal/MaxNodes
* /rtabmap/GridGlobal/MinSize
* /rtabmap/GridGlobal/OccupancyThr
* /rtabmap/GridGlobal/ProbClampingMax
* /rtabmap/GridGlobal/ProbClampingMin
* /rtabmap/GridGlobal/ProbHit
* /rtabmap/GridGlobal/ProbMiss
* /rtabmap/GridGlobal/UpdateError
* /rtabmap/Icp/CCFilterOutFarthestPoints
* /rtabmap/Icp/CCMaxFinalRMS
* /rtabmap/Icp/CCSamplingLimit
* /rtabmap/Icp/CorrespondenceRatio
* /rtabmap/Icp/DownsamplingStep
* /rtabmap/Icp/Epsilon
* /rtabmap/Icp/Force4DoF
* /rtabmap/Icp/Iterations
* /rtabmap/Icp/MaxCorrespondenceDistance
* /rtabmap/Icp/MaxRotation
* /rtabmap/Icp/MaxTranslation
* /rtabmap/Icp/OutlierRatio
* /rtabmap/Icp/PMConfig
* /rtabmap/Icp/PMMatcherEpsilon
* /rtabmap/Icp/PMMatcherIntensity
* /rtabmap/Icp/PMMatcherKnn
* /rtabmap/Icp/PointToPlane
* /rtabmap/Icp/PointToPlaneGroundNormalsUp
* /rtabmap/Icp/PointToPlaneK
* /rtabmap/Icp/PointToPlaneLowComplexityStrategy
* /rtabmap/Icp/PointToPlaneMinComplexity
* /rtabmap/Icp/PointToPlaneRadius
* /rtabmap/Icp/RangeMax
* /rtabmap/Icp/RangeMin
* /rtabmap/Icp/Strategy
* /rtabmap/Icp/VoxelSize
* /rtabmap/ImuFilter/ComplementaryBiasAlpha
* /rtabmap/ImuFilter/ComplementaryDoAdpativeGain
* /rtabmap/ImuFilter/ComplementaryDoBiasEstimation
* /rtabmap/ImuFilter/ComplementaryGainAcc
* /rtabmap/ImuFilter/MadgwickGain
* /rtabmap/ImuFilter/MadgwickZeta
* /rtabmap/KAZE/Diffusivity
* /rtabmap/KAZE/Extended
* /rtabmap/KAZE/NOctaveLayers
* /rtabmap/KAZE/NOctaves
* /rtabmap/KAZE/Threshold
* /rtabmap/KAZE/Upright
* /rtabmap/Kp/BadSignRatio
* /rtabmap/Kp/ByteToFloat
* /rtabmap/Kp/DetectorStrategy
* /rtabmap/Kp/DictionaryPath
* /rtabmap/Kp/FlannRebalancingFactor
* /rtabmap/Kp/GridCols
* /rtabmap/Kp/GridRows
* /rtabmap/Kp/IncrementalDictionary
* /rtabmap/Kp/IncrementalFlann
* /rtabmap/Kp/MaxDepth
* /rtabmap/Kp/MaxFeatures
* /rtabmap/Kp/MinDepth
* /rtabmap/Kp/NNStrategy
* /rtabmap/Kp/NewWordsComparedTogether
* /rtabmap/Kp/NndrRatio
* /rtabmap/Kp/Parallelized
* /rtabmap/Kp/RoiRatios
* /rtabmap/Kp/SubPixEps
* /rtabmap/Kp/SubPixIterations
* /rtabmap/Kp/SubPixWinSize
* /rtabmap/Kp/TfIdfLikelihoodUsed
* /rtabmap/Marker/CornerRefinementMethod
* /rtabmap/Marker/Dictionary
* /rtabmap/Marker/Length
* /rtabmap/Marker/MaxDepthError
* /rtabmap/Marker/MaxRange
* /rtabmap/Marker/MinRange
* /rtabmap/Marker/VarianceAngular
* /rtabmap/Marker/VarianceLinear
* /rtabmap/Mem/BadSignaturesIgnored
* /rtabmap/Mem/BinDataKept
* /rtabmap/Mem/CompressionParallelized
* /rtabmap/Mem/CovOffDiagIgnored
* /rtabmap/Mem/DepthAsMask
* /rtabmap/Mem/GenerateIds
* /rtabmap/Mem/ImageCompressionFormat
* /rtabmap/Mem/ImageKept
* /rtabmap/Mem/ImagePostDecimation
* /rtabmap/Mem/ImagePreDecimation
* /rtabmap/Mem/IncrementalMemory
* /rtabmap/Mem/InitWMWithAllNodes
* /rtabmap/Mem/IntermediateNodeDataKept
* /rtabmap/Mem/LaserScanDownsampleStepSize
* /rtabmap/Mem/LaserScanNormalK
* /rtabmap/Mem/LaserScanNormalRadius
* /rtabmap/Mem/LaserScanVoxelSize
* /rtabmap/Mem/LocalizationDataSaved
* /rtabmap/Mem/MapLabelsAdded
* /rtabmap/Mem/NotLinkedNodesKept
* /rtabmap/Mem/RawDescriptorsKept
* /rtabmap/Mem/RecentWmRatio
* /rtabmap/Mem/ReduceGraph
* /rtabmap/Mem/RehearsalIdUpdatedToNewOne
* /rtabmap/Mem/RehearsalSimilarity
* /rtabmap/Mem/RehearsalWeightIgnoredWhileMoving
* /rtabmap/Mem/STMSize
* /rtabmap/Mem/SaveDepth16Format
* /rtabmap/Mem/StereoFromMotion
* /rtabmap/Mem/TransferSortingByWeightId
* /rtabmap/Mem/UseOdomFeatures
* /rtabmap/Mem/UseOdomGravity
* /rtabmap/ORB/EdgeThreshold
* /rtabmap/ORB/FirstLevel
* /rtabmap/ORB/Gpu
* /rtabmap/ORB/NLevels
* /rtabmap/ORB/PatchSize
* /rtabmap/ORB/ScaleFactor
* /rtabmap/ORB/ScoreType
* /rtabmap/ORB/WTA_K
* /rtabmap/Optimizer/Epsilon
* /rtabmap/Optimizer/GravitySigma
* /rtabmap/Optimizer/Iterations
* /rtabmap/Optimizer/LandmarksIgnored
* /rtabmap/Optimizer/PriorsIgnored
* /rtabmap/Optimizer/Robust
* /rtabmap/Optimizer/Strategy
* /rtabmap/Optimizer/VarianceIgnored
* /rtabmap/PyDetector/Cuda
* /rtabmap/PyDetector/Path
* /rtabmap/PyMatcher/Cuda
* /rtabmap/PyMatcher/Iterations
* /rtabmap/PyMatcher/Model
* /rtabmap/PyMatcher/Path
* /rtabmap/PyMatcher/Threshold
* /rtabmap/RGBD/AngularSpeedUpdate
* /rtabmap/RGBD/AngularUpdate
* /rtabmap/RGBD/CreateOccupancyGrid
* /rtabmap/RGBD/Enabled
* /rtabmap/RGBD/GoalReachedRadius
* /rtabmap/RGBD/GoalsSavedInUserData
* /rtabmap/RGBD/LinearSpeedUpdate
* /rtabmap/RGBD/LinearUpdate
* /rtabmap/RGBD/LocalBundleOnLoopClosure
* /rtabmap/RGBD/LocalImmunizationRatio
* /rtabmap/RGBD/LocalRadius
* /rtabmap/RGBD/LoopClosureIdentityGuess
* /rtabmap/RGBD/LoopClosureReextractFeatures
* /rtabmap/RGBD/LoopCovLimited
* /rtabmap/RGBD/MarkerDetection
* /rtabmap/RGBD/MaxLocalRetrieved
* /rtabmap/RGBD/MaxLoopClosureDistance
* /rtabmap/RGBD/MaxOdomCacheSize
* /rtabmap/RGBD/NeighborLinkRefining
* /rtabmap/RGBD/NewMapOdomChangeDistance
* /rtabmap/RGBD/OptimizeFromGraphEnd
* /rtabmap/RGBD/OptimizeMaxError
* /rtabmap/RGBD/PlanAngularVelocity
* /rtabmap/RGBD/PlanLinearVelocity
* /rtabmap/RGBD/PlanStuckIterations
* /rtabmap/RGBD/ProximityAngle
* /rtabmap/RGBD/ProximityBySpace
* /rtabmap/RGBD/ProximityByTime
* /rtabmap/RGBD/ProximityMaxGraphDepth
* /rtabmap/RGBD/ProximityMaxPaths
* /rtabmap/RGBD/ProximityOdomGuess
* /rtabmap/RGBD/ProximityPathFilteringRadius
* /rtabmap/RGBD/ProximityPathMaxNeighbors
* /rtabmap/RGBD/ProximityPathRawPosesUsed
* /rtabmap/RGBD/ScanMatchingIdsSavedInLinks
* /rtabmap/RGBD/StartAtOrigin
* /rtabmap/Reg/Force3DoF
* /rtabmap/Reg/RepeatOnce
* /rtabmap/Reg/Strategy
* /rtabmap/Rtabmap/ComputeRMSE
* /rtabmap/Rtabmap/CreateIntermediateNodes
* /rtabmap/Rtabmap/DetectionRate
* /rtabmap/Rtabmap/ImageBufferSize
* /rtabmap/Rtabmap/ImagesAlreadyRectified
* /rtabmap/Rtabmap/LoopGPS
* /rtabmap/Rtabmap/LoopRatio
* /rtabmap/Rtabmap/LoopThr
* /rtabmap/Rtabmap/MaxRetrieved
* /rtabmap/Rtabmap/MemoryThr
* /rtabmap/Rtabmap/PublishLastSignature
* /rtabmap/Rtabmap/PublishLikelihood
* /rtabmap/Rtabmap/PublishPdf
* /rtabmap/Rtabmap/PublishRAMUsage
* /rtabmap/Rtabmap/PublishStats
* /rtabmap/Rtabmap/RectifyOnlyFeatures
* /rtabmap/Rtabmap/SaveWMState
* /rtabmap/Rtabmap/StartNewMapOnGoodSignature
* /rtabmap/Rtabmap/StartNewMapOnLoopClosure
* /rtabmap/Rtabmap/StatisticLogged
* /rtabmap/Rtabmap/StatisticLoggedHeaders
* /rtabmap/Rtabmap/StatisticLogsBufferedInRAM
* /rtabmap/Rtabmap/TimeThr
* /rtabmap/Rtabmap/WorkingDirectory
* /rtabmap/SIFT/ContrastThreshold
* /rtabmap/SIFT/EdgeThreshold
* /rtabmap/SIFT/NFeatures
* /rtabmap/SIFT/NOctaveLayers
* /rtabmap/SIFT/RootSIFT
* /rtabmap/SIFT/Sigma
* /rtabmap/SURF/Extended
* /rtabmap/SURF/GpuKeypointsRatio
* /rtabmap/SURF/GpuVersion
* /rtabmap/SURF/HessianThreshold
* /rtabmap/SURF/OctaveLayers
* /rtabmap/SURF/Octaves
* /rtabmap/SURF/Upright
* /rtabmap/Stereo/DenseStrategy
* /rtabmap/Stereo/Eps
* /rtabmap/Stereo/Iterations
* /rtabmap/Stereo/MaxDisparity
* /rtabmap/Stereo/MaxLevel
* /rtabmap/Stereo/MinDisparity
* /rtabmap/Stereo/OpticalFlow
* /rtabmap/Stereo/SSD
* /rtabmap/Stereo/WinHeight
* /rtabmap/Stereo/WinWidth
* /rtabmap/StereoBM/BlockSize
* /rtabmap/StereoBM/Disp12MaxDiff
* /rtabmap/StereoBM/MinDisparity
* /rtabmap/StereoBM/NumDisparities
* /rtabmap/StereoBM/PreFilterCap
* /rtabmap/StereoBM/PreFilterSize
* /rtabmap/StereoBM/SpeckleRange
* /rtabmap/StereoBM/SpeckleWindowSize
* /rtabmap/StereoBM/TextureThreshold
* /rtabmap/StereoBM/UniquenessRatio
* /rtabmap/StereoSGBM/BlockSize
* /rtabmap/StereoSGBM/Disp12MaxDiff
* /rtabmap/StereoSGBM/MinDisparity
* /rtabmap/StereoSGBM/Mode
* /rtabmap/StereoSGBM/NumDisparities
* /rtabmap/StereoSGBM/P1
* /rtabmap/StereoSGBM/P2
* /rtabmap/StereoSGBM/PreFilterCap
* /rtabmap/StereoSGBM/SpeckleRange
* /rtabmap/StereoSGBM/SpeckleWindowSize
* /rtabmap/StereoSGBM/UniquenessRatio
* /rtabmap/SuperPoint/Cuda
* /rtabmap/SuperPoint/ModelPath
* /rtabmap/SuperPoint/NMS
* /rtabmap/SuperPoint/NMSRadius
* /rtabmap/SuperPoint/Threshold
* /rtabmap/VhEp/Enabled
* /rtabmap/VhEp/MatchCountMin
* /rtabmap/VhEp/RansacParam1
* /rtabmap/VhEp/RansacParam2
* /rtabmap/Vis/BundleAdjustment
* /rtabmap/Vis/CorFlowEps
* /rtabmap/Vis/CorFlowIterations
* /rtabmap/Vis/CorFlowMaxLevel
* /rtabmap/Vis/CorFlowWinSize
* /rtabmap/Vis/CorGuessMatchToProjection
* /rtabmap/Vis/CorGuessWinSize
* /rtabmap/Vis/CorNNDR
* /rtabmap/Vis/CorNNType
* /rtabmap/Vis/CorType
* /rtabmap/Vis/DepthAsMask
* /rtabmap/Vis/EpipolarGeometryVar
* /rtabmap/Vis/EstimationType
* /rtabmap/Vis/FeatureType
* /rtabmap/Vis/ForwardEstOnly
* /rtabmap/Vis/GridCols
* /rtabmap/Vis/GridRows
* /rtabmap/Vis/InlierDistance
* /rtabmap/Vis/Iterations
* /rtabmap/Vis/MaxDepth
* /rtabmap/Vis/MaxFeatures
* /rtabmap/Vis/MeanInliersDistance
* /rtabmap/Vis/MinDepth
* /rtabmap/Vis/MinInliers
* /rtabmap/Vis/MinInliersDistribution
* /rtabmap/Vis/PnPFlags
* /rtabmap/Vis/PnPRefineIterations
* /rtabmap/Vis/PnPReprojError
* /rtabmap/Vis/RefineIterations
* /rtabmap/Vis/RoiRatios
* /rtabmap/Vis/SubPixEps
* /rtabmap/Vis/SubPixIterations
* /rtabmap/Vis/SubPixWinSize
* /rtabmap/g2o/Baseline
* /rtabmap/g2o/Optimizer
* /rtabmap/g2o/PixelVariance
* /rtabmap/g2o/RobustKernelDelta
* /rtabmap/g2o/Solver
* /rtabmap/is_rtabmap_paused
* /rtabmap/rgbd_odometry/approx_sync
* /rtabmap/rgbd_odometry/config_path
* /rtabmap/rgbd_odometry/expected_update_rate
* /rtabmap/rgbd_odometry/frame_id
* /rtabmap/rgbd_odometry/ground_truth_base_frame_id
* /rtabmap/rgbd_odometry/ground_truth_frame_id
* /rtabmap/rgbd_odometry/guess_frame_id
* /rtabmap/rgbd_odometry/guess_min_rotation
* /rtabmap/rgbd_odometry/guess_min_translation
* /rtabmap/rgbd_odometry/keep_color
* /rtabmap/rgbd_odometry/max_update_rate
* /rtabmap/rgbd_odometry/odom_frame_id
* /rtabmap/rgbd_odometry/publish_tf
* /rtabmap/rgbd_odometry/queue_size
* /rtabmap/rgbd_odometry/subscribe_rgbd
* /rtabmap/rgbd_odometry/wait_for_transform_duration
* /rtabmap/rgbd_odometry/wait_imu_to_init
* /rtabmap/rtabmap/Mem/IncrementalMemory
* /rtabmap/rtabmap/Mem/InitWMWithAllNodes
* /rtabmap/rtabmap/approx_sync
* /rtabmap/rtabmap/config_path
* /rtabmap/rtabmap/database_path
* /rtabmap/rtabmap/frame_id
* /rtabmap/rtabmap/gen_scan
* /rtabmap/rtabmap/ground_truth_base_frame_id
* /rtabmap/rtabmap/ground_truth_frame_id
* /rtabmap/rtabmap/landmark_angular_variance
* /rtabmap/rtabmap/landmark_linear_variance
* /rtabmap/rtabmap/map_frame_id
* /rtabmap/rtabmap/odom_frame_id
* /rtabmap/rtabmap/odom_sensor_sync
* /rtabmap/rtabmap/odom_tf_angular_variance
* /rtabmap/rtabmap/odom_tf_linear_variance
* /rtabmap/rtabmap/publish_tf
* /rtabmap/rtabmap/queue_size
* /rtabmap/rtabmap/scan_cloud_max_points
* /rtabmap/rtabmap/subscribe_depth
* /rtabmap/rtabmap/subscribe_odom_info
* /rtabmap/rtabmap/subscribe_rgb
* /rtabmap/rtabmap/subscribe_rgbd
* /rtabmap/rtabmap/subscribe_scan
* /rtabmap/rtabmap/subscribe_scan_cloud
* /rtabmap/rtabmap/subscribe_scan_descriptor
* /rtabmap/rtabmap/subscribe_stereo
* /rtabmap/rtabmap/subscribe_user_data
* /rtabmap/rtabmap/wait_for_transform_duration
* 

COMPLETE PARAMETERS LIST EKF
----------------------------

* /ekf_local/base_link_frame
* /ekf_local/debug
* /ekf_local/frequency
* /ekf_local/imu0
* /ekf_local/imu0_config
* /ekf_local/imu0_differential
* /ekf_local/imu0_linear_acceleration_rejection_threshold
* /ekf_local/imu0_nodelay
* /ekf_local/imu0_pose_rejection_threshold
* /ekf_local/imu0_queue_size
* /ekf_local/imu0_relative
* /ekf_local/imu0_remove_gravitational_acceleration
* /ekf_local/imu0_twist_rejection_threshold
* /ekf_local/initial_estimate_covariance
* /ekf_local/map_frame
* /ekf_local/odom0
* /ekf_local/odom0_config
* /ekf_local/odom0_differential
* /ekf_local/odom0_nodelay
* /ekf_local/odom0_pose_rejection_threshold
* /ekf_local/odom0_queue_size
* /ekf_local/odom0_relative
* /ekf_local/odom0_twist_rejection_threshold
* /ekf_local/odom_frame
* /ekf_local/print_diagnostics
* /ekf_local/process_noise_covariance
* /ekf_local/publish_tf
* /ekf_local/sensor_timeout
* /ekf_local/transform_time_offset
* /ekf_local/transform_timeout
* /ekf_local/two_d_mode
* /ekf_local/use_control
* /ekf_local/world_frame
* /ekf_global/base_link_frame
* /ekf_global/debug
* /ekf_global/frequency
* /ekf_global/imu0
* /ekf_global/imu0_config
* /ekf_global/imu0_differential
* /ekf_global/imu0_linear_acceleration_rejection_threshold
* /ekf_global/imu0_nodelay
* /ekf_global/imu0_pose_rejection_threshold
* /ekf_global/imu0_queue_size
* /ekf_global/imu0_relative
* /ekf_global/imu0_remove_gravitational_acceleration
* /ekf_global/imu0_twist_rejection_threshold
* /ekf_global/initial_estimate_covariance
* /ekf_global/map_frame
* /ekf_global/odom0
* /ekf_global/odom0_config
* /ekf_global/odom0_differential
* /ekf_global/odom0_nodelay
* /ekf_global/odom0_pose_rejection_threshold
* /ekf_global/odom0_queue_size
* /ekf_global/odom0_relative
* /ekf_global/odom0_twist_rejection_threshold
* /ekf_global/odom1
* /ekf_global/odom1_config
* /ekf_global/odom1_differential
* /ekf_global/odom1_nodelay
* /ekf_global/odom1_queue_size
* /ekf_global/odom1_relative
* /ekf_global/odom_frame
* /ekf_global/print_diagnostics
* /ekf_global/process_noise_covariance
* /ekf_global/publish_tf
* /ekf_global/sensor_timeout
* /ekf_global/transform_time_offset
* /ekf_global/transform_timeout
* /ekf_global/two_d_mode
* /ekf_global/use_control
* /ekf_global/world_frame* 
