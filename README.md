HUSKY SIMULATION
----------------

Start docker:

    docker start ingeniarius-simulators

In each terminal:

    docker exec -it ingeniarius-simulators bash
    cd agriculture_sim/
    source devel/setup.bash


Terminal 1 (launch simulation):

    roslaunch agriculture_launcher bringup.launch


Terminal 2 (move the robot):

    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -5.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'


Terminal 3 (run rviz and open the configuration file):

    rosrun rviz rviz -d ~/marti/map_test_1.rviz


TEMP COMMANDS
------------

    rosservice call /rtabmap/reset "{}"

    rosservice call /rtabmap/trigger_new_map "{}"


EXTRA NOTES
-----------

Find where is a pkg:

    rospack find <name_of_the_pkg> 

See the icp params:

    rtabmap --params | grep Icp/

Root mode start:

    sudo su

Root mode end:

    exit

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

* Send the rviz config file from my pc to the docker:

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
    

* Send the rviz config file from the docker to my pc:

*(on my pc terminal)*

*General command:*

    docker cp <containerId>:/file/path/in/container/file /host/local/path/

*My command:*

    docker cp 304712a249f8:/home/developer/marti/map_test_1.rviz /home/mzaera/Documents/
    docker cp 304712a249f8:/home/developer/agriculture_sim/src/configurations/robot_localization/navsat_transform.yaml /home/mzaera/Documents/


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

GIT COMMANDS
------------

Obtain the repo:

    git clone https://github.com/mzaera/notes_2021

Update the repo:

    git add --all
    git commit -m "readme"
    git push


CHANGES DONE
------------

* /home/developer/agriculture_sim/src/agriculture_launcher/rtabmap/rtabmap_simulator.launch

Line 97 aprox.
    *Using rgbd odom seems to add an error (scans rotating while husky moves) so we use icp insted*
    *Also error about number of of points in the two input datasets differs (seems not to afect)*    

    <arg name="icp_odometry"             default="true"/>         <!-- Launch rtabmap icp odometry node -->

Line 98 aprox.
    *Just a change of name easier to identify as we are using icp now*

    <arg name="odom_topic"               default="odom_rgbd_icp"/>          <!-- Odometry topic name -->

* /home/developer/agriculture_sim/src/configurations/robot_localization/ekf_global.yaml

Line 30 aprox.
    *Just a change of name easier to identify as we are using icp now*

    odom0: rtabmap/odom_rgbd_icp

* /home/developer/agriculture_sim/src/configurations/robot_localization/ekf_local.yaml

Line 30 aprox.
    *Just a change of name easier to identify as we are using icp now*

    odom0: rtabmap/odom_rgbd_icp


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


INFO/EXTRA LIST
---------------

* [ INFO] [1625138739.271218554, 41.039000000]: imu plugin missing <xyzOffset>, defaults to 0s
[ INFO] [1625138739.272245560, 41.039000000]: imu plugin missing <rpyOffset>, defaults to 0s

* [pcl::NormalEstimationOMP::compute] Both radius (1.000000) and K (5) defined! Set one of them to zero first and then re-run compute ().
[pcl::concatenateFields] The number of points in the two input datasets differs

* DynamicParams:

    Odometry parameters:

        left wheel radius multiplier: 1
        right wheel radius multiplier: 1
        wheel separation multiplier: 1.875

    Publication parameters:

        Publish executed velocity command: disabled
        Publication rate: 50
        Publish frame odom on tf: disabled


WARNINGS LIST
-------------

* Warning [parser_urdf.cc:1119] multiple inconsistent <gravity> exists due to fixed joint reduction overwriting previous value [true] with [false].


* Warning [parser.cc:950] XML Element[vertical_fov], child of element[camera] not defined in SDF. Ignoring[vertical_fov]. You may have an incorrect SDF file, or an sdformat version that doesn't support this element.


* [ WARN] [1624981315.525914322, 41.562000000]: Setting "Grid/FromDepth" parameter to false (default true) as "subscribe_scan", "subscribe_scan_cloud" or "gen_scan" is true. The occupancy grid map will be constructed from laser scans. To get occupancy grid map from cloud projection, set "Grid/FromDepth" to true. To suppress this warning, add <param name="Grid/FromDepth" type="string" value="false"/>

* [ WARN] [1624981315.670153842, 41.562000000]: There is no image subscription, bag-of-words loop closure detection will be disabled...


* [WARN] [1624981338.851268, 41.562000]: Controller Spawner couldn't find the expected controller_manager ROS interface.


* [ WARN] [1624981340.567380170, 41.563000000]: Failed to meet update rate! Took 41.563000000000002387
[ WARN] [1624981340.570003059, 41.563000000]: Failed to meet update rate! Took 41.563000000000002387
[ WARN] [1624981340.574535775, 41.565000000]: Failed to meet update rate! Took 41.531666667000003201
[ WARN] [1624981340.575426327, 41.567000000]: Failed to meet update rate! Took 41.53366666699999854



* [ WARN] [1624981385.775020023, 49.004000000]: Odometry: Detected not valid consecutive stamps (previous=48.904000s new=48.904000s). New stamp should be always greater than previous stamp. This new data is ignored.

* [ WARN] [1625042581.356276872, 281.039000000]: odometry: Could not get transform from base_link to front_realsense_gazebo (stamp=280.761000) after 0.200000 seconds ("wait_for_transform_duration"=0.200000)! Error="canTransform: source_frame front_realsense_gazebo does not exist.. canTransform returned after 0.2 timeout was 0.2."


* [ WARN] [1625148019.783057694, 40.730000000]: IcpOdometry: Transferring value 0.05 of "Icp/VoxelSize" to ros parameter "scan_voxel_size" for convenience. "Icp/VoxelSize" is set to 0.
[ WARN] [1625148019.783580811, 40.730000000]: IcpOdometry: Transferring value 5 of "Icp/PointToPlaneK" to ros parameter "scan_normal_k" for convenience.
[ WARN] [1625148019.784069661, 40.730000000]: IcpOdometry: Transferring value 1.0 of "Icp/PointToPlaneRadius" to ros parameter "scan_normal_radius" for convenience.


* [ WARN] [1625215117.373219580, 55.754000000]: We are receiving imu data (buffer=1), but cannot interpolate imu transform at time 55.640000. IMU won't be added to graph.


* [WARN] [1625228090.214656, 41.891000]: Controller Spawner couldn't find the expected controller_manager ROS interface.


ERROR
-----

All the errors seems to be solved automaticaly after a few seconds

* [ERROR] [1624981808.355031649, 40.651000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/front_left_wheel
[ERROR] [1624981808.356011145, 40.651000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/front_right_wheel
[ERROR] [1624981808.356927444, 40.651000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/rear_left_wheel
[ERROR] [1624981808.357667160, 40.651000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/rear_right_wheel



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

* /agriculture_geom: <?xml version="1....
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
 * /ekf_global/odom0: rtabmap/rgbd_odom
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
 * /ekf_local/odom0: rtabmap/rgbd_odom
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
 * /navsat_transform_node/yaw_offset: 0.0
 * /navsat_transform_node/zero_altitude: True
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
 * /robot_description: <?xml version="1....
 * /rosdistro: melodic
 * /rosversion: 1.14.6
 * /rtabmap/rgbd_odometry/approx_sync: True
 * /rtabmap/rgbd_odometry/config_path: 
 * /rtabmap/rgbd_odometry/expected_update_rate: 0.0
 * /rtabmap/rgbd_odometry/frame_id: base_link
 * /rtabmap/rgbd_odometry/ground_truth_base_frame_id: 
 * /rtabmap/rgbd_odometry/ground_truth_frame_id: 
 * /rtabmap/rgbd_odometry/guess_frame_id: 
 * /rtabmap/rgbd_odometry/guess_min_rotation: 0.0
 * /rtabmap/rgbd_odometry/guess_min_translation: 0.0
 * /rtabmap/rgbd_odometry/keep_color: False
 * /rtabmap/rgbd_odometry/max_update_rate: 0.0
 * /rtabmap/rgbd_odometry/odom_frame_id: odom
 * /rtabmap/rgbd_odometry/publish_tf: False
 * /rtabmap/rgbd_odometry/queue_size: 10
 * /rtabmap/rgbd_odometry/subscribe_rgbd: False
 * /rtabmap/rgbd_odometry/wait_for_transform_duration: 0.2
 * /rtabmap/rgbd_odometry/wait_imu_to_init: False
 * /rtabmap/rtabmap/Mem/IncrementalMemory: true
 * /rtabmap/rtabmap/Mem/InitWMWithAllNodes: false
 * /rtabmap/rtabmap/approx_sync: True
 * /rtabmap/rtabmap/config_path: 
 * /rtabmap/rtabmap/database_path: ~/.ros/rtabmap.db
 * /rtabmap/rtabmap/frame_id: base_link
 * /rtabmap/rtabmap/gen_scan: False
 * /rtabmap/rtabmap/ground_truth_base_frame_id: 
 * /rtabmap/rtabmap/ground_truth_frame_id: 
 * /rtabmap/rtabmap/landmark_angular_variance: 9999.0
 * /rtabmap/rtabmap/landmark_linear_variance: 0.0001
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
