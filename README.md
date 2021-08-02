# TRANSPORTER

Run full_system.launch:

```bash
ssh solarcleano@192.168.8.70
```

```bash
ssh vinebot_0@192.168.8.238
```
*Exit to ssh crt+d*

```bash
cd catkin_ws
source devel/setup.bash
```
```bash
roslaunch transporter_launcher full_system.launch
```

```bash
roslaunch vinebot_launcher full_system.launch
```
Run rviz:
*New terminal ROS_MASTER_URI » 192.168.8.X*

```bash
rosrun rviz rviz -d /home/mzaera/Documents/rviz_config/transporter_config.rviz
```


## Initial Configs
*NO cakin_make if some ccp file is modified should do catkin build*

### SSH

```bash
ssh solarcleano@192.168.8.70
```
### File folders

+ Other locations & connect:

```bash
sftp://solarcleano@192.168.8.70
```

And right click + "Add Bookmark".

### Bash

```bash
gedit ~/.bashrc
```

```bash
# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
  # We have color support; assume it's compliant with Ecma-48
  # (ISO/IEC-6429). (Lack of such support is extremely rare, and such
  # a case would tend to support setf rather than setaf.)
  color_prompt=yes
    else
  color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

source /opt/ros/melodic/setup.bash

#--------------

MY_IP=$(hostname -I | awk '{print $1}')
echo 'Current IP: '$MY_IP

# Debug config: IP of cleaner_1 Ingeniarius/Ingeniarius-5G/Ingeniarius-4G
# export ROS_MASTER_URI=http://192.168.8.203:11311
# export ROS_IP=$MY_IP


# Debug config: IP of server Ingeniarius/Ingeniarius-5G/Ingeniarius-4G
DEFINED_ROS_MASTER_NAMES=('localhost' 'no_name' 'no_name' 'no_name' 'no_name' 'no_name' 'transporter')
DEFINED_ROS_MASTER_URIS=($MY_IP '192.168.8.200' '192.168.8.203' '192.168.8.15' '192.168.8.207' '192.168.8.28' '192.168.8.70')

echo '---------- Available ROS_MASTER_URIs ----------'

for i in ${!DEFINED_ROS_MASTER_URIS[@]}; do
    if [ $i == 0 ]; then
        printf '\e[1m'
        printf '%d \t %s \t %s \t (default)\n' $i ${DEFINED_ROS_MASTER_URIS[$i]} ${DEFINED_ROS_MASTER_NAMES[$i]}
        printf '\e[0m'
    else
        printf '%d \t %s \t %s\n' $i ${DEFINED_ROS_MASTER_URIS[$i]} ${DEFINED_ROS_MASTER_NAMES[$i]}
    fi

done 

echo 'Select index and press [ENTER]:'
read idx

# Checks if idx is empty and if is valid
if [ -z "$idx" ]
then
    echo 'Setting default IP as ROS_MASTER_URI.'
    idx=0
else
    if (( $idx < 0 || $idx > $i ))
    then
        echo 'Selected option '$idx' is not defined!'
        echo 'Setting up default IP.'
        idx=0
    fi
fi

echo 'Defined ROS_MASTER_URI »»» '${DEFINED_ROS_MASTER_URIS[$idx]}



export ROS_MASTER_URI=http://${DEFINED_ROS_MASTER_URIS[$idx]}:11311
export ROS_IP=$MY_IP


#--------------

## General BUILD ros ws
source ~/catkin_ws/devel/setup.bash
ROS_WORKSPACE=~/catkin_ws

# Editor
export EDITOR='code -w'
```

# HUSKY SIMULATION
## My PC

* In each terminal:

```bash
cd catkin_ws
source devel/setup.bash
```

* Terminal 1 (launch simulation):

```bash
roslaunch agriculture_launcher bringup.launch
```

```bash
roslaunch agriculture_launcher inspect_bringup.launch
```

* Terminal 2 (move the robot):

Also can be done by adding 2D navigation markers on Rviz.

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -5.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

```bash
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
```

## Docker


* Start docker:

```bash
docker start ingeniarius-simulators
```

* In each terminal:

```bash
docker exec -it ingeniarius-simulators bash
cd agriculture_sim/
source devel/setup.bash
```

* Terminal 1 (launch simulation).

* Terminal 2 (move the robot).


# USEFUL LINKS

## Docker:

[Docker instalation guide](https://hub.docker.com/r/ingeniarius/ingeniarius-simulators)

## Gazebo:

[Git MAS-UAV pkg](https://github.com/Rezenders/mas_uav)

[Git Husky pkg](https://github.com/husky/husky)

[Agriculture envairoment](https://www.clearpathrobotics.com/assets/guides/kinetic/husky/additional_sim_worlds.html)

## RTABMAP:

[ RTAB-Map Webpage](http://introlab.github.io/rtabmap/)

[ RTAB-Map Forum](http://official-rtab-map-forum.67519.x6.nabble.com/)

[ROS Wiki RTAB-Map](http://wiki.ros.org/rtabmap_ros)

## Robot Localization:

[Robot_localization GIT](https://github.com/cra-ros-pkg/robot_localization)

[Robot_localization Wiki](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)

[Robot_localization Params](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html)

[Data robot_localization](http://docs.ros.org/en/indigo/api/robot_localization/html/preparing_sensor_data.html)

## Octomap:

[Octomap_server Wiki](http://wiki.ros.org/octomap_server)

## Others:

[Covariance matrices](https://manialabs.wordpress.com/2012/08/06/covariance-matrices-with-a-practical-example/)

[Quaternions and Euler angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

[Quaternions and Euler angles Online Transform](https://quaternions.online/)

# CHANGES DONE

## Bringup.launch

Path: /agriculture_sim/src/agriculture_launcher/bringup.launch

```bash
<launch>

    <include file="$(find cpr_agriculture_gazebo)/launch/agriculture_world.launch">
      <arg name="platform"          value="husky" />
    </include> 

    <include file="$(find agriculture_launcher)/robot_localization/localization_local.launch"/>
    <include file="$(find agriculture_launcher)/robot_localization/localization_global.launch"/>

    <include file="$(find husky_navigation)/launch/move_base.launch">
      <arg name="no_static_map"     value="true"/>
    </include>

    <include file="$(find agriculture_launcher)/rtabmap/rtabmap_simulation_husky.launch">
      <arg name="node_start_delay"  value="10.0"/>
      <arg name="max_range"         value="20.0"/>
      <arg name="max_obst"          value="3.5"/>
    </include> 

    <include file="$(find agriculture_launcher)/rviz/rviz_rtabmap.launch">  
      <arg name="node_start_delay" value="10.0"/>
    </include> 

</launch>
```

## Rtabmap_simulation_husky.launch

Path: /agriculture_sim/src/agriculture_launcher/rtabmap

Generate a file called "rtabmap_simulation_husky.launch"

```bash
<launch>       
     
  <param  name="use_sim_time"                       type="bool"   value="true"/>
  <arg    name="args"                               default="-- delete_db_on_start -d"/>                 
  
  <arg    name="node_start_delay"                   default="0.0"/>                 
                         
  <arg    name="max_range"                          default="25.0"/>                 
  <arg    name="max_obst"                           default="100.0"/>                 

                            
  <group ns="rtabmap">

    <!--RGB-D Odometry-->
    <node name="rgbd_odometry" pkg="rtabmap_ros" type="rgbd_odometry" output="log" args="$(arg args)" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@'" >

      <!--Normal Params-->
      <param name="frame_id"                    type="string" value="base_link"/>
      <param name="odom_frame_id"               type="string" value="odom"/>
      <param name="publish_tf"                  type="bool"   value="false"/>
      <param name="wait_for_transform_duration" type="double" value="0.5"/>
      <param name="initial_pose"                type="string" value=""/>
      <param name="queue_size"                  type="int"    value="100"/>
      <param name="publish_null_when_lost"      type="bool"   value="true"/>
      <param name="ground_truth_frame_id"       type="string" value=""/>
      <param name="ground_truth_base_frame_id"  type="string" value=""/>
      <param name="guess_frame_id"              type="string" value=""/>
      <param name="guess_min_translation"       type="double" value="0.0"/>
      <param name="guess_min_rotation"          type="double" value="0.0"/>
      <param name="config_path"                 type="string" value=""/>
  
      <param name="approx_sync"                 type="bool"   value="true"/>
      <param name="rgbd_cameras"                type="int"    value="1"/>
      <param name="subscribe_rgbd"              type="bool"   value="false"/>

      <!--Extra Params-->
      <param name="wait_imu_to_init"            type="bool"   value="false"/>
      <param name="keep_color"                  type="bool"   value="false"/>

      <!--Remaps-->
      <remap from="rgb/image"       to="/realsense/color/image_raw"/>
      <remap from="depth/image"     to="/realsense/depth/image_rect_raw"/>
      <remap from="rgb/camera_info" to="/realsense/color/camera_info"/>
      <remap from="odom"            to="/rtabmap/rgbd_odom"/>
      <remap from="imu"             to="/imu/data"/>
    </node>

    <!--RTABMAP-->
    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="$(arg args)" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@'" >

      <!-- Normal Params-->
      <param name="subscribe_depth"                 type="bool"   value="true"/>
      <param name="subscribe_scan"                  type="bool"   value="false"/>
      <param name="subscribe_scan_cloud"            type="bool"   value="true"/>
      <param name="subscribe_stereo"                type="bool"   value="false"/>
      <param name="subscribe_rgbd"                  type="bool"   value="false"/>
      <param name="frame_id"                        type="string" value="base_link"/>
      <param name="map_frame_id"                    type="string" value="map"/>
      <param name="odom_frame_id"                   type="string" value="map"/> <!--(odom) map ??? -->
      <param name="odom_tf_linear_variance"         type="double" value="1.0"/> <!--1.0 when simulation else 0.0005-->
      <param name="odom_tf_angular_variance"        type="double" value="1.0"/> <!--1.0 when simulaton else 0.0005-->
      <param name="queue_size"                      type="int"    value="100"/>
      <param name="publish_tf"                      type="bool"   value="false"/>
      <param name="tf_delay"                        type="double" value="0.05"/>
      <param name="wait_for_transform"              type="bool"   value="true"/>
      <param name="wait_for_transform_duration"     type="double" value="0.5"/>
      <param name="config_path"                     type="string" value=""/>
      <param name="database_path"                   type="string" value="~/.ros/rtabmap.db"/>
      <param name="gen_scan"                        type="bool"   value="false"/>
      <param name="gen_scan_max_depth"              type="double" value="4.0"/>
      <param name="approx_sync"                     type="bool"   value="true"/>
      <param name="rgbd_cameras"                    type="int"    value="1"/>
      <param name="use_action_for_goal"             type="bool"   value="true"/>
      <param name="odom_sensor_sync"                type="bool"   value="false"/>
      <param name="gen_depth"                       type="bool"   value="false"/> 
      <param name="gen_depth_decimation"            type="int"    value="1"/>
      <param name="gen_depth_fill_holes_size"       type="int"    value="0"/>
      <param name="gen_depth_fill_iterations"       type="double" value="0.1"/>      
      <param name="gen_depth_fill_holes_error"      type="int"    value="1"/>
      <param name="map_filter_radius"               type="double" value="0.0"/>      
      <param name="map_filter_angle"                type="double" value="30.0"/>      
      <param name="map_cleanup"                     type="bool"   value="true"/> 
      <param name="latch"                           type="bool"   value="true"/>
      <param name="map_always_update"               type="bool"   value="false"/>
      <param name="map_empty_ray_tracing"           type="bool"   value="true"/>
      <param name="cloud_output_voxelized"          type="bool"   value="false"/> 
      <param name="cloud_subtract_filtering"                  type="bool" value="true"/> 
      <param name="cloud_subtract_filtering_min_neighbors"    type="int"  value="5"/> 

      <!--Extra Params-->
      <param name="Grid/3D"                         type="bool"   value="true"/>      
      <param name="Grid/CellSize"                   type="double" value="0.1"/>
      <param name="Grid/FromDepth"                  type="bool"   value="false"/>
      <param name="Grid/NoiseFilteringMinNeighbors" type="int"    value="0"/>
      <param name="Grid/NoiseFilteringRadius"       type="double" value="0.0"/>
      <param name="Grid/ClusterRadius"              type="int"    value="1"/> 
      <param name="Grid/RayTracing"                 type="bool"   value="true"/> 
      <param name="Grid/MapFrameProjection"         type="bool"   value="true"/>
      <param name="Grid/PreVoxelFiltering"          type="bool"   value="false"/>

      <param name="Grid/RangeMax"                   type="double" value="$(arg max_range)"/> 

      <param name="Grid/NormalsSegmentation"        type="bool"   value="true"/>
      <param name="Grid/MaxObstacleHeight"          type="double" value="$(arg max_obst)"/>  
      <param name="Grid/MaxGroundHeight"            type="double" value="0.15"/> 
      <param name="Grid/MinGroundHeight"            type="double" value="0.0"/> 
      <param name="Grid/MaxGroundAngle"             type="int"    value="40"/>

      <param name="Kp/DetectorStrategy"             type="string" value="0"/>

      <param name="Mem/NotLinkedNodesKept"          type="string" value="false"/>

      <param name="OdomF2M/ScanSubtractRadius"      type="double" value="0.1"/> 
      <param name="OdomF2M/ScanMaxSize"             type="int"    value="0"/>

      <param name="Optimizer/Iterations"            type="int"    value="100"/> 

      <param name="Reg/Force3DoF"                   type="string" value="true"/>
      <param name="Reg/Strategy"                    type="string" value="1"/>

      <param name="RGBD/NeighborLinkRefining"       type="bool"   value="true"/> 
      <param name="RGBD/OptimizeMaxError"           type="string" value="3.0"/>
      <param name="RGBD/OptimizeFromGraphEnd"       type="string" value="false"/>
      <param name="RGBD/AngularUpdate"              type="string" value="0.1"/>
      <param name="RGBD/LinearUpdate"               type="string" value="0.1"/>
      <param name="RGBD/LoopClosureReextractFeatures" type="string" value="true"/>

      <param name="SURF/HessianThreshold"           type="string" value="300"/>

      <param name="Vis/MinInliers"                  type="string" value="25"/>
      <param name="Vis/FeatureType"                 type="string" value="0"/>

      <!--Remaps-->
      <remap from="rgb/image"              to="/realsense/color/image_raw"/>
      <remap from="depth/image"            to="/realsense/depth/image_rect_raw"/>
      <remap from="rgb/camera_info"        to="/realsense/color/camera_info"/>
      <remap from="scan_cloud"             to="/points"/>
      <remap from="scan_descriptor"        to="/scan_descriptor"/>
      <remap from="user_data"              to="/user_data"/>
      <remap from="user_data_async"        to="/user_data_async"/>
      <remap from="tag_detections"         to="/tag_detections"/>
      <remap from="odom"                   to="rtabmap/rgbd_odom"/>
      <remap from="imu"                    to="/imu/data"/>
    </node>
  
  </group> 

</launch>
```

## Rviz_rtabmap_config.rviz & Rviz_husky.launch

Path: /agriculture_sim/src/agriculture_launcher/rviz

Copy from this repo the file called "rviz_rtabmap_config.rviz" on the generated rviz folder and add the launch file rviz_husky.launch.

```bash
mkdir rviz
cd rviz
```

```bash
<launch>
  <arg  name="node_start_delay"  default="0.0"/>                 
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find agriculture_launcher)/rviz/rviz_rtabmap_config.rviz" required="true" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@'"/>
</launch>

```

## Inspect_bringup.launch

Path: /agriculture_sim/src/agriculture_launcher

Generate a file called inspect_bringup.launch".

```bash
<launch>
    
    <include file="$(find cpr_inspection_gazebo)/launch/inspection_world.launch">
      <arg name="platform" value="husky" />
    </include> 

    <include file="$(find agriculture_launcher)/robot_localization/localization_local.launch"/>
    <include file="$(find agriculture_launcher)/robot_localization/localization_global.launch"/>

    <include file="$(find husky_navigation)/launch/move_base.launch">
      <arg name="no_static_map" value="true"/>
    </include>

    <include file="$(find agriculture_launcher)/rtabmap/rtabmap_simulation_husky.launch">
      <arg name="node_start_delay"  value="20.0"/>
      <arg name="max_range"         value="20.0"/>
      <arg name="max_obst"          value="500.0"/>
    </include> 

    <include file="$(find agriculture_launcher)/rviz/rviz_rtabmap.launch">  
      <arg name="node_start_delay" value="10.0"/>
    </include> 

</launch>
```

## Inspection_world.launch

Path: /agriculture_sim/src/cpr_gazebo/cpr_inspection_gazebo/launch/inspection_world.launch

Modify the spawn point of the husky. Line 5 - 8.

```bash
<arg name="x" default="-17.177975"/>
<arg name="y" default="7.264329"/>
<arg name="z" default="1.01229"/>
<arg name="yaw" default="0.2" />
```

## Inspection_world.world

Path: /agriculture_sim/src/cpr_gazebo/cpr_inspection_gazebo/worlds/inspection_world.world

Turn off the shadows. Line 23.

```bash
<shadows>0</shadows>
```

## PROVISIONAL SOLUTION: navsat_transform.yaml

Path: /agriculture_sim/src/configurations/robot_localization/navsat_transform.yaml

### 2D mode

```bash
frequency: 30
delay: 3
magnetic_declination_radians: 0.0349066 #check!!
yaw_offset: 1.570796
zero_altitude: true
broadcast_utm_transform: true
broadcast_utm_transform_as_parent_frame: true
publish_filtered_gps: false
use_odometry_yaw: true
wait_for_datum: true
#datum: [41.2206659318, -8.52751781305, 0.0]
datum: [49.8999999596, 8.90000090445, -0.00463541982361] #simulation
transform_timeout: 0.1
```

### 3D mode

```bash
frequency: 30
delay: 3
magnetic_declination_radians: 0.0349066 #check!!
yaw_offset: 1.570796
zero_altitude: false
broadcast_utm_transform: true
broadcast_utm_transform_as_parent_frame: true
publish_filtered_gps: false
use_odometry_yaw: true
wait_for_datum: true
#datum: [41.2206659318, -8.52751781305, 0.0]
datum: [49.8999999596, 8.90000090445, -0.00463541982361] #simulation
transform_timeout: 0.1
```

## PROVISIONAL SOLUTION: ekf_global.yaml

Path: /agriculture_sim/src/configurations/robot_localization/ekf_global.yaml

### 2D mode

```bash
frequency: 30
sensor_timeout: 0.1
two_d_mode: true
transform_time_offset: 0.05
transform_timeout: 0.0
#predict_to_current_time: true 
print_diagnostics: true
debug: false
publish_tf: true

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: map

# odom0: example
# odom0_config:  [x,        y,        z,
#                 roll,     pitch,    yaw,
#                 vel_x,    vel_y,    vel_z,
#                 vel_roll, vel_pitc, vel_yaw,
#                 acc_x,    acc_y,    acc_z]

odom0: rtabmap/rgbd_odom
odom0_config: [true, true, true,
               true, false, false,
               false, false, false,
               false, false, true,
               false, false, false]

odom0_queue_size: 5
odom0_nodelay: true
odom0_differential: true
odom0_relative: false
odom0_pose_rejection_threshold: 50.0
odom0_twist_rejection_threshold: 50.0

odom1: ekf/gps_converted_odom
odom1_config: [ true,  true,  true,
                false, false, false,
                false, false, false,
                false, false, false,
                false, false, false]
odom1_queue_size: 2
odom1_nodelay: true
odom1_differential: false
odom1_relative: false
        
imu0: imu/data
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
imu0_nodelay: false
imu0_differential: false
imu0_relative: false
imu0_queue_size: 5
imu0_pose_rejection_threshold: 50.0                 # Note the difference in parameter names
imu0_twist_rejection_threshold: 50.0                #
imu0_linear_acceleration_rejection_threshold: 50.0  #

# [ADVANCED] Some IMUs automatically remove acceleration due to gravity, and others don't. If yours doesn't, please set
# this to true, and *make sure* your data conforms to REP-103, specifically, that the data is in ENU frame.
imu0_remove_gravitational_acceleration: true

use_control: false

process_noise_covariance:  [1.0,  0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    1.0,  0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    1e-3, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0.3,  0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0.3,  0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0.01, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0.5,   0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0.5,   0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0.1,  0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0.3,  0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.3,  0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.3,  0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.3,  0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.3,  0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.3]

initial_estimate_covariance:   [1.0,  0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    1.0,  0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    1.0,  0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    1.0,  0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    1.0,  0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    1.0,  0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    1.0,  0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    1.0,   0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1.0,   0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1.0,   0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     1.0,  0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    1.0,  0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    1.0]
```

### 3D mode

```bash
frequency: 30
sensor_timeout: 0.1
two_d_mode: false #TRUE = 0 --> Z, roll, pitch, their respective velocities, and Z acceleration 
transform_time_offset: 0.05
transform_timeout: 0.0
print_diagnostics: true
debug: false
publish_tf: true

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: map

# odom0: example
# odom0_config:  [x,        y,        z,
#                 roll,     pitch,    yaw,
#                 vel_x,    vel_y,    vel_z,
#                 vel_roll, vel_pitc, vel_yaw,
#                 acc_x,    acc_y,    acc_z]

odom0: rtabmap/rgbd_odom
odom0_config: [false, false, false,
               false, false, false,
               false, false, false,
               false, false, false,
               false, false, false]
odom0_queue_size: 10
odom0_nodelay: true
odom0_differential: true
odom0_relative: false
odom0_pose_rejection_threshold: 50
odom0_twist_rejection_threshold: 50

odom1: ekf/gps_converted_odom
odom1_config: [ true,  true,  true,
                false, false, false,
                false, false, false,
                false, false, false,
                false, false, false]
odom1_nodelay: true
odom1_differential: false
odom1_relative: false
        
imu0: imu/data
imu0_config: [false, false, false,
              true, true, true,
              false, false, false,
              true, true, true,
              true, true, true]
imu0_nodelay: false
imu0_differential: false
imu0_relative: false
imu0_queue_size: 10
imu0_pose_rejection_threshold: 50                 
imu0_angular_velocity_rejection_threshold: 50               
imu0_linear_acceleration_rejection_threshold: 50  

# [ADVANCED] Some IMUs automatically remove acceleration due to gravity, and others don't. If yours doesn't, please set
# this to true, and *make sure* your data conforms to REP-103, specifically, that the data is in ENU frame.
imu0_remove_gravitational_acceleration: true

use_control: false

process_noise_covariance:  [1.0,  0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    1.0,  0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    1e-3, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0.3,  0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0.3,  0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0.01, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0.5,   0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0.5,   0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0.1,  0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0.3,  0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.3,  0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.3,  0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.3,  0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.3,  0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.3]

initial_estimate_covariance:   [1.0,  0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    1.0,  0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    1.0,  0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    1.0,  0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    1.0,  0,    0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    1.0,  0,    0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    1.0,  0,     0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    1.0,   0,     0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1.0,   0,     0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1.0,   0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     1.0,  0,    0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    1.0,  0,
                                0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    1.0]
```

## PROVISIONAL SOLUTION: ekf_local.yaml

Path: /agriculture_sim/src/configurations/robot_localization/ekf_local.yaml

### 2D mode

```bash
frequency: 30
sensor_timeout: 0.1
two_d_mode: true
transform_time_offset: 0.05
transform_timeout: 0.0
#predict_to_current_time: true 
print_diagnostics: true
debug: false
publish_tf: true

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom

# odom0: example
# odom0_config:  [x,        y,        z,
#                 roll,     pitch,    yaw,
#                 vel_x,    vel_y,    vel_z,
#                 vel_roll, vel_pitc, vel_yaw,
#                 acc_x,    acc_y,    acc_z]

odom0: rtabmap/rgbd_odom
odom0_config: [false, false, false,
               true,  false, false,
               false, false, false,
               false, false, true,
               false, false, false]
odom0_queue_size: 5
odom0_nodelay: false
odom0_differential: false
odom0_relative: false
odom0_pose_rejection_threshold: 50.0
odom0_twist_rejection_threshold: 50.0

imu0: imu/data
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
imu0_nodelay: false
imu0_differential: false
imu0_relative: false
imu0_queue_size: 5
imu0_pose_rejection_threshold: 50.0                 # Note the difference in parameter names
imu0_twist_rejection_threshold: 50.0                #
imu0_linear_acceleration_rejection_threshold: 50.0  #


# [ADVANCED] Some IMUs automatically remove acceleration due to gravity, and others don't. If yours doesn't, please set
# this to true, and *make sure* your data conforms to REP-103, specifically, that the data is in ENU frame.
imu0_remove_gravitational_acceleration: true

use_control: false

process_noise_covariance:  [1e-3, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    1e-3, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    1e-3, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    1e-3, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    1e-3, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    1e-3, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0.5,   0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0.5,   0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0.1,  0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0.3,  0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.3,  0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.3,  0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.3,  0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.3,  0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.3]

initial_estimate_covariance: [1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    1.0,  0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    1.0,  0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    1.0,  0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    1.0,  0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    1.0,  0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    1.0,   0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1.0,   0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1.0,   0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     1.0,  0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    1.0,  0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    1.0]

```

### 3D mode

```bash
frequency: 30
sensor_timeout: 0.1
two_d_mode: false #TRUE = 0 --> Z, roll, pitch, their respective velocities, and Z acceleration 
transform_time_offset: 0.05
transform_timeout: 0.0
print_diagnostics: true
debug: false
publish_tf: true

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom

# odom0: example
# odom0_config:  [x,        y,        z,
#                 roll,     pitch,    yaw,
#                 vel_x,    vel_y,    vel_z,
#                 vel_roll, vel_pitc, vel_yaw,
#                 acc_x,    acc_y,    acc_z]

odom0: rtabmap/rgbd_odom
odom0_config: [false, false, false,
               false,  false, false,
               false, false, false,
               false, false, false,
               false, false, false]
odom0_queue_size: 10
odom0_nodelay: false
odom0_differential: false
odom0_relative: false

odom0_pose_rejection_threshold: 50
odom0_twist_rejection_threshold: 50

imu0: imu/data
imu0_config: [false, false, false,
              true, true, true,
              false, false, false,
              true, true, true,
              true, true, true]
imu0_nodelay: false
imu0_differential: false
imu0_relative: false
imu0_queue_size: 10

imu0_pose_rejection_threshold: 50                 
imu0_angular_velocity_rejection_threshold: 50                
imu0_linear_acceleration_rejection_threshold: 50  

# [ADVANCED] Some IMUs automatically remove acceleration due to gravity, and others don't. If yours doesn't, please set
# this to true, and *make sure* your data conforms to REP-103, specifically, that the data is in ENU frame.
imu0_remove_gravitational_acceleration: true

use_control: false

process_noise_covariance:  [1e-3, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    1e-3, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    1e-3, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    1e-3, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    1e-3, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    1e-3, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0.5,   0,     0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0.5,   0,    0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0.1,  0,    0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0.3,  0,    0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.3,  0,    0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.3,  0,    0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.3,  0,    0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.3,  0,
                            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.3]

initial_estimate_covariance: [1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    1.0,  0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    1.0,  0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    1.0,  0,    0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    1.0,  0,    0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    1.0,  0,     0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    1.0,   0,     0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1.0,   0,     0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1.0,   0,    0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     1.0,  0,    0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    1.0,  0,
                              0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    1.0]

```


# TERMINAL COMMANDS

## Git commands

* Obtain the repo:

```bash
git clone https://github.com/mzaera/notes_2021
```
* Update the repo:

```bash
git add --all
git commit -m "readme"
git push
```

## Docker commands

* Start docker:

```bash
docker start ingeniarius-simulators
```
* Stop the docker container (my pc terminal):

```bash
docker container stop ingeniarius-simulators
```

* Delate and rekame the docker:
    *(run.bash on the folder)*

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

## File from PC to Docker


* Get the path of a folder:
    
```bash
pwd
```

* To obtain the container ID:
    *(on my pc terminal)*

```bash
docker container ls -a
```

* General command:

```bash
docker cp  /host/local/path/file <containerId>:/file/path/in/container/
```

## File from Docker to PC


* General command:

```bash
docker cp <containerId>:/file/path/in/container/file /host/local/path/
```

## Full pkg from Docker to PC

* Create a Catkin Workspace:

```bash  
cd
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
* Copy the folder from docker to catkin_ws/src:

```bash
docker cp <containerId>:/file/path/in/container/ /host/local/path/
```
* Install the dependencies:

```bash
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
sudo apt-get install libpcap-dev
catkin_make
```
* Install the missing pkgs:

```bash
sudo apt-get install ros-melodic-octomap-rviz-plugins
sudo apt install ros-melodic-rtabmap-ros
```




## Install Sublime


* Install sublime

```bash
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
```

## Teleop Twist Keyboard

* Install:

```bash
sudo apt install ros-melodic-teleop-twist-keyboard
```
* Use (normal):

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
* Use (remapping):

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=name_of_the_new_topic
```

## Extra notes

* Find where is a pkg:

```bash
rospack find <name_of_the_pkg> 
```

* See the especific params:

```bash
rtabmap --params | grep Name/
```

* Root mode start:

```bash
sudo su
```

* Root mode end:

```bash
exit
```


# ODOM INPUTS

NOT updated!

* rostopic echo -n1 /rtabmap/rgbd_odom

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

# ROSNODE LIST


```bash
/base_controller_spawner
/ekf_global
/ekf_local
/gazebo
/gazebo_gui
/move_base
/navsat_transform_node
/octomap_server
/realsense_to_laserscan
/robot_state_publisher
/rosout
/rostopic_8124_1626769758575
/rtabmap/rgbd_odometry
/rtabmap/rtabmap
/rviz
/twist_marker_server
/twist_mux
```

# ROSTOPIC LIST


```bash
/clicked_point
/clock
/cmd_vel
/diagnostics
/e_stop
/ekf/global_odom
/ekf/gps_converted_odom
/ekf/local_odom
/free_cells_vis_array
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/gps/fix
/gps/fix/position/parameter_descriptions
/gps/fix/position/parameter_updates
/gps/fix/status/parameter_descriptions
/gps/fix/status/parameter_updates
/gps/fix/velocity/parameter_descriptions
/gps/fix/velocity/parameter_updates
/gps/vel
/ground_truth/state
/husky_velocity_controller/cmd_vel
/husky_velocity_controller/odom
/husky_velocity_controller/parameter_descriptions
/husky_velocity_controller/parameter_updates
/imu/data
/imu/data/accel/parameter_descriptions
/imu/data/accel/parameter_updates
/imu/data/bias
/imu/data/rate/parameter_descriptions
/imu/data/rate/parameter_updates
/imu/data/yaw/parameter_descriptions
/imu/data/yaw/parameter_updates
/initialpose
/joint_states
/joy_teleop/cmd_vel
/move_base/DWAPlannerROS/cost_cloud
/move_base/DWAPlannerROS/global_plan
/move_base/DWAPlannerROS/local_plan
/move_base/DWAPlannerROS/parameter_descriptions
/move_base/DWAPlannerROS/parameter_updates
/move_base/DWAPlannerROS/trajectory_cloud
/move_base/NavfnROS/plan
/move_base/cancel
/move_base/current_goal
/move_base/feedback
/move_base/global_costmap/costmap
/move_base/global_costmap/costmap_updates
/move_base/global_costmap/footprint
/move_base/global_costmap/inflation/parameter_descriptions
/move_base/global_costmap/inflation/parameter_updates
/move_base/global_costmap/obstacles_laser/parameter_descriptions
/move_base/global_costmap/obstacles_laser/parameter_updates
/move_base/global_costmap/parameter_descriptions
/move_base/global_costmap/parameter_updates
/move_base/goal
/move_base/local_costmap/costmap
/move_base/local_costmap/costmap_updates
/move_base/local_costmap/footprint
/move_base/local_costmap/inflation/parameter_descriptions
/move_base/local_costmap/inflation/parameter_updates
/move_base/local_costmap/obstacles_laser/parameter_descriptions
/move_base/local_costmap/obstacles_laser/parameter_updates
/move_base/local_costmap/parameter_descriptions
/move_base/local_costmap/parameter_updates
/move_base/parameter_descriptions
/move_base/parameter_updates
/move_base/result
/move_base/status
/move_base_simple/goal
/occupied_cells_vis_array
/octomap_binary
/octomap_full
/octomap_point_cloud_centers
/octomap_server/parameter_descriptions
/octomap_server/parameter_updates
/odom
/points
/projected_map
/projected_map_updates
/realsense/color/camera_info
/realsense/color/image_raw
/realsense/color/image_raw/compressed
/realsense/color/image_raw/compressed/parameter_descriptions
/realsense/color/image_raw/compressed/parameter_updates
/realsense/color/image_raw/compressedDepth
/realsense/color/image_raw/compressedDepth/parameter_descriptions
/realsense/color/image_raw/compressedDepth/parameter_updates
/realsense/color/image_raw/theora
/realsense/color/image_raw/theora/parameter_descriptions
/realsense/color/image_raw/theora/parameter_updates
/realsense/depth/camera_info
/realsense/depth/color/points
/realsense/depth/image_rect_raw
/realsense/parameter_descriptions
/realsense/parameter_updates
/realsense/scan
/rosout
/rosout_agg
/rtabmap/cloud_ground
/rtabmap/cloud_map
/rtabmap/cloud_obstacles
/rtabmap/global_path
/rtabmap/global_path_nodes
/rtabmap/global_pose
/rtabmap/goal
/rtabmap/goal_node
/rtabmap/goal_out
/rtabmap/goal_reached
/rtabmap/gps/fix
/rtabmap/grid_map
/rtabmap/grid_prob_map
/rtabmap/info
/rtabmap/initialpose
/rtabmap/labels
/rtabmap/landmarks
/rtabmap/local_grid_empty
/rtabmap/local_grid_ground
/rtabmap/local_grid_obstacle
/rtabmap/local_path
/rtabmap/local_path_nodes
/rtabmap/localization_pose
/rtabmap/mapData
/rtabmap/mapGraph
/rtabmap/mapPath
/rtabmap/octomap_binary
/rtabmap/octomap_empty_space
/rtabmap/octomap_full
/rtabmap/octomap_global_frontier_space
/rtabmap/octomap_grid
/rtabmap/octomap_ground
/rtabmap/octomap_obstacles
/rtabmap/octomap_occupied_space
/rtabmap/odom_info
/rtabmap/odom_info_lite
/rtabmap/odom_last_frame
/rtabmap/odom_local_map
/rtabmap/odom_local_scan_map
/rtabmap/odom_rgbd_image
/rtabmap/proj_map
/rtabmap/rgbd_odom
/rtabmap/scan_map
/scan
/set_pose
/tf
/tf_static
/twist_marker_server/cmd_vel
/twist_marker_server/feedback
/twist_marker_server/update
/twist_marker_server/update_full
/user_data_async
```

# PARAMETERS LIST RTABMAP

All the paramatres except the rtabmap (generals) and rgbd_odometry params. Those mising paramatres are: [ROS Wiki RTAB-Map](http://wiki.ros.org/rtabmap_ros) .

* BRIEF
```bash
Param: BRIEF/Bytes = "32"                                  [Bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.]
```
* BRISK
```bash
Param: BRISK/Octaves = "3"                                 [Detection octaves. Use 0 to do single scale.]
Param: BRISK/PatternScale = "1"                            [Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.]
Param: BRISK/Thresh = "30"                                 [FAST/AGAST detection threshold score.]
```
* Bayes
```bash
Param: Bayes/FullPredictionUpdate = "false"                [Regenerate all the prediction matrix on each iteration (otherwise only removed/added ids are updated).]
Param: Bayes/PredictionLC = "0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23" [Prediction of loop closures (Gaussian-like, here with sigma=1.6) - Format: {VirtualPlaceProb, LoopClosureProb, NeighborLvl1, NeighborLvl2, ...}.]
Param: Bayes/VirtualPlacePriorThr = "0.9"                  [Virtual place prior]
```
* Db
```bash
Param: Db/TargetVersion = ""                               [Target database version for backward compatibility purpose. Only Major and minor versions are used and should be set (e.g., 0.19 vs 0.20 or 1.0 vs 2.0). Patch version is ignored (e.g., 0.20.1 and 0.20.3 will generate a 0.20 database).]
```
* DbSqlite3
```bash
Param: DbSqlite3/CacheSize = "10000"                       [Sqlite cache size (default is 2000).]
Param: DbSqlite3/InMemory = "false"                        [Using database in the memory instead of a file on the hard disk.]
Param: DbSqlite3/JournalMode = "3"                         [0=DELETE, 1=TRUNCATE, 2=PERSIST, 3=MEMORY, 4=OFF (see sqlite3 doc : "PRAGMA journal_mode")]
Param: DbSqlite3/Synchronous = "0"                         [0=OFF, 1=NORMAL, 2=FULL (see sqlite3 doc : "PRAGMA synchronous")]
Param: DbSqlite3/TempStore = "2"                           [0=DEFAULT, 1=FILE, 2=MEMORY (see sqlite3 doc : "PRAGMA temp_store")]
```
* FAST
```bash
Param: BRISK/Thresh = "30"                                 [FAST/AGAST detection threshold score.]
Param: FAST/CV = "0"                                       [Enable FastCV implementation if non-zero (and RTAB-Map is built with FastCV support). Values should be 9 and 10.]
Param: FAST/Gpu = "false"                                  [GPU-FAST: Use GPU version of FAST. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.]
Param: FAST/GpuKeypointsRatio = "0.05"                     [Used with FAST GPU.]
Param: FAST/GridCols = "0"                                 [Grid cols (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.]
Param: FAST/GridRows = "0"                                 [Grid rows (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.]
Param: FAST/MaxThreshold = "200"                           [Maximum threshold. Used only when FAST/GridRows and FAST/GridCols are set.]
Param: FAST/MinThreshold = "7"                             [Minimum threshold. Used only when FAST/GridRows and FAST/GridCols are set.]
Param: FAST/NonmaxSuppression = "true"                     [If true, non-maximum suppression is applied to detected corners (keypoints).]
Param: FAST/Threshold = "20"                               [Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.]
Param: Kp/DetectorStrategy = "8"                           [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
Param: Vis/FeatureType = "8"                               [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
```
* FREAK
```bash
Param: FREAK/NOctaves = "4"                                [Number of octaves covered by the detected keypoints.]
Param: FREAK/OrientationNormalized = "true"                [Enable orientation normalization.]
Param: FREAK/PatternScale = "22"                           [Scaling of the description pattern.]
Param: FREAK/ScaleNormalized = "true"                      [Enable scale normalization.]
```
* GFTT
```bash
Param: GFTT/BlockSize = "3"                                []
Param: GFTT/K = "0.04"                                     []
Param: GFTT/MinDistance = "7"                              []
Param: GFTT/QualityLevel = "0.001"                         []
Param: GFTT/UseHarrisDetector = "false"                    []
Param: Kp/DetectorStrategy = "8"                           [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
Param: Vis/FeatureType = "8"                               [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
```
* GMS
```bash
Param: GMS/ThresholdFactor = "6.0"                         [The higher, the less matches.]
Param: GMS/WithRotation = "false"                          [Take rotation transformation into account.]
Param: GMS/WithScale = "false"                             [Take scale transformation into account.]
```
* Grid
```bash
Param: Grid/3D = "true"                                    [A 3D occupancy grid is required if you want an OctoMap (3D ray tracing). Set to false if you want only a 2D map, the cloud will be projected on xy plane. A 2D map can be still generated if checked, but it requires more memory and time to generate it. Ignored if laser scan is 2D and "Grid/FromDepth" is false.]
Param: Grid/CellSize = "0.05"                              [Resolution of the occupancy grid.]
Param: Grid/ClusterRadius = "0.1"                          [[Grid/NormalsSegmentation=true] Cluster maximum radius.]
Param: Grid/DepthDecimation = "4"                          [[Grid/DepthDecimation=true] Decimation of the depth image before creating cloud. Negative decimation is done from RGB size instead of depth size (if depth is smaller than RGB, it may be interpolated depending of the decimation value).]
Param: Grid/DepthRoiRatios = "0.0 0.0 0.0 0.0"             [[Grid/FromDepth=true] Region of interest ratios [left, right, top, bottom].]
Param: Grid/FlatObstacleDetected = "true"                  [[Grid/NormalsSegmentation=true] Flat obstacles detected.]
Param: Grid/FootprintHeight = "0.0"                        [Footprint height used to filter points over the footprint of the robot. Footprint length and width should be set.]
Param: Grid/FootprintLength = "0.0"                        [Footprint length used to filter points over the footprint of the robot.]
Param: Grid/FootprintWidth = "0.0"                         [Footprint width used to filter points over the footprint of the robot. Footprint length should be set.]
Param: Grid/FromDepth = "true"                             [Create occupancy grid from depth image(s), otherwise it is created from laser scan.]
Param: Grid/GroundIsObstacle = "false"                     [[Grid/3D=true] Ground segmentation (Grid/NormalsSegmentation) is ignored, all points are obstacles. Use this only if you want an OctoMap with ground identified as an obstacle (e.g., with an UAV).]
Param: Grid/MapFrameProjection = "false"                   [Projection in map frame. On a 3D terrain and a fixed local camera transform (the cloud is created relative to ground), you may want to disable this to do the projection in robot frame instead.]
Param: Grid/MaxGroundAngle = "45"                          [[Grid/NormalsSegmentation=true] Maximum angle (degrees) between points normal to grounds normal to label it as ground. Points with higher angle difference are considered as obstacles.]
Param: Grid/MaxGroundHeight = "0.0"                        [Maximum ground height (0=disabled). Should be set if "Grid/NormalsSegmentation" is false.]
Param: Grid/MaxObstacleHeight = "0.0"                      [Maximum obstacles height (0=disabled).]
Param: Grid/MinClusterSize = "10"                          [[Grid/NormalsSegmentation=true] Minimum cluster size to project the points.]
Param: Grid/MinGroundHeight = "0.0"                        [Minimum ground height (0=disabled).]
Param: Grid/NoiseFilteringMinNeighbors = "5"               [Noise filtering minimum neighbors.]
Param: Grid/NoiseFilteringRadius = "0.0"                   [Noise filtering radius (0=disabled). Done after segmentation.]
Param: Grid/NormalK = "20"                                 [[Grid/NormalsSegmentation=true] K neighbors to compute normals.]
Param: Grid/NormalsSegmentation = "true"                   [Segment ground from obstacles using point normals, otherwise a fast passthrough is used.]
Param: Grid/PreVoxelFiltering = "true"                     [Input cloud is downsampled by voxel filter (voxel size is "Grid/CellSize") before doing segmentation of obstacles and ground.]
Param: Grid/RangeMax = "5.0"                               [Maximum range from sensor. 0=inf.]
Param: Grid/RangeMin = "0.0"                               [Minimum range from sensor.]
Param: Grid/RayTracing = "false"                           [Ray tracing is done for each occupied cell, filling unknown space between the sensor and occupied cells. If Grid/3D=true, RTAB-Map should be built with OctoMap support, otherwise 3D ray tracing is ignored.]
Param: Grid/Scan2dUnknownSpaceFilled = "false"             [Unknown space filled. Only used with 2D laser scans. Use Grid/RangeMax to set maximum range if laser scan max range is to set.]
Param: Grid/ScanDecimation = "1"                           [[Grid/FromDepth=false] Decimation of the laser scan before creating cloud.]
```
* GridGlobal
```bash
Param: GridGlobal/AltitudeDelta = "0"                      [Assemble only nodes that have the same altitude of +-delta meters of the current pose (0=disabled). This is used to generate 2D occupancy grid based on the current altitude (e.g., multi-floor building).]
Param: GridGlobal/Eroded = "false"                         [Erode obstacle cells.]
Param: GridGlobal/FootprintRadius = "0.0"                  [Footprint radius (m) used to clear all obstacles under the graph.]
Param: GridGlobal/FullUpdate = "true"                      [When the graph is changed, the whole map will be reconstructed instead of moving individually each cells of the map. Also, data added to cache wont be released after updating the map. This process is longer but more robust to drift that would erase some parts of the map when it should not.]
Param: GridGlobal/MaxNodes = "0"                           [Maximum nodes assembled in the map starting from the last node (0=unlimited).]
Param: GridGlobal/MinSize = "0.0"                          [Minimum map size (m).]
Param: GridGlobal/OccupancyThr = "0.5"                     [Occupancy threshold (value between 0 and 1).]
Param: GridGlobal/ProbClampingMax = "0.971"                [Probability clamping maximum (value between 0 and 1).]
Param: GridGlobal/ProbClampingMin = "0.1192"               [Probability clamping minimum (value between 0 and 1).]
Param: GridGlobal/ProbHit = "0.7"                          [Probability of a hit (value between 0.5 and 1).]
Param: GridGlobal/ProbMiss = "0.4"                         [Probability of a miss (value between 0 and 0.5).]
Param: GridGlobal/UpdateError = "0.01"                     [Graph changed detection error (m). Update map only if poses in new optimized graph have moved more than this value.]
```
* Icp
```bash
Param: Icp/CCFilterOutFarthestPoints = "false"             [If true, the algorithm will automatically ignore farthest points from the reference, for better convergence.]
Param: Icp/CCMaxFinalRMS = "0.2"                           [Maximum final RMS error.]
Param: Icp/CCSamplingLimit = "50000"                       [Maximum number of points per cloud (they are randomly resampled below this limit otherwise).]
Param: Icp/CorrespondenceRatio = "0.1"                     [Ratio of matching correspondences to accept the transform.]
Param: Icp/DownsamplingStep = "1"                          [Downsampling step size (1=no sampling). This is done before uniform sampling.]
Param: Icp/Epsilon = "0"                                   [Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.]
Param: Icp/Force4DoF = "false"                             [Limit ICP to x, y, z and yaw DoF. Available if Icp/Strategy > 0.]
Param: Icp/Iterations = "30"                               [Max iterations.]
Param: Icp/MaxCorrespondenceDistance = "0.05"              [Max distance for point correspondences.]
Param: Icp/MaxRotation = "0.78"                            [Maximum ICP rotation correction accepted (rad).]
Param: Icp/MaxTranslation = "0.2"                          [Maximum ICP translation correction accepted (m).]
Param: Icp/OutlierRatio = "0.85"                           [Outlier ratio used with Icp/Strategy>0. For libpointmatcher, this parameter set TrimmedDistOutlierFilter/ratio for convenience when configuration file is not set. For CCCoreLib, this parameter set the "finalOverlapRatio". The value should be between 0 and 1.]
Param: Icp/PMConfig = ""                                   [Configuration file (*.yaml) used by libpointmatcher. Note that data filters set for libpointmatcher are done after filtering done by rtabmap (i.e., Icp/VoxelSize, Icp/DownsamplingStep), so make sure to disable those in rtabmap if you want to use only those from libpointmatcher. Parameters Icp/Iterations, Icp/Epsilon and Icp/MaxCorrespondenceDistance are also ignored if configuration file is set.]
Param: Icp/PMMatcherEpsilon = "0.0"                        [KDTreeMatcher/epsilon: approximation to use for the nearest-neighbor search. For convenience when configuration file is not set.]
Param: Icp/PMMatcherIntensity = "false"                    [KDTreeMatcher:  among nearest neighbors, keep only the one with the most similar intensity. This only work with Icp/PMMatcherKnn>1.]
Param: Icp/PMMatcherKnn = "1"                              [KDTreeMatcher/knn: number of nearest neighbors to consider it the reference. For convenience when configuration file is not set.]
Param: Icp/PointToPlane = "false"                          [Use point to plane ICP.]
Param: Icp/PointToPlaneGroundNormalsUp = "0.0"             [Invert normals on ground if they are pointing down (useful for ring-like 3D LiDARs). 0 means disabled, 1 means only normals perfectly aligned with -z axis. This is only done with 3D scans.]
Param: Icp/PointToPlaneK = "5"                             [Number of neighbors to compute normals for point to plane if the cloud doesnt have already normals.]
Param: Icp/PointToPlaneLowComplexityStrategy = "1"         [If structural complexity is below Icp/PointToPlaneMinComplexity: set to 0 to so that the transform is automatically rejected, set to 1 to limit ICP correction in axes with most constraints (e.g., for a corridor-like environment, the resulting transform will be limited in y and yaw, x will taken from the guess), set to 2 to accept "as is" the transform computed by PointToPoint.]
Param: Icp/PointToPlaneMinComplexity = "0.02"              [Minimum structural complexity (0.0=low, 1.0=high) of the scan to do PointToPlane registration, otherwise PointToPoint registration is done instead and strategy from Icp/PointToPlaneLowComplexityStrategy is used. This check is done only when Icp/PointToPlane=true.]
Param: Icp/PointToPlaneRadius = "1.0"                      [Search radius to compute normals for point to plane if the cloud doesnt have already normals.]
Param: Icp/RangeMax = "0"                                  [Maximum range filtering (0=disabled).]
Param: Icp/RangeMin = "0"                                  [Minimum range filtering (0=disabled).]
Param: Icp/Strategy = "0"                                  [ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).]
Param: Icp/VoxelSize = "0.05"                              [Uniform sampling voxel size (0=disabled).]

```
* ImuFilter
```bash
Param: ImuFilter/ComplementaryBiasAlpha = "0.01"           [Bias estimation gain parameter, belongs in [0, 1].]
Param: ImuFilter/ComplementaryDoAdpativeGain = "true"      [Parameter whether to do adaptive gain or not.]
Param: ImuFilter/ComplementaryDoBiasEstimation = "true"    [Parameter whether to do bias estimation or not.]
Param: ImuFilter/ComplementaryGainAcc = "0.01"             [Gain parameter for the complementary filter, belongs in [0, 1].]
Param: ImuFilter/MadgwickGain = "0.1"                      [Gain of the filter. Higher values lead to faster convergence but more noise. Lower values lead to slower convergence but smoother signal, belongs in [0, 1].]
Param: ImuFilter/MadgwickZeta = "0.0"                      [Gyro drift gain (approx. rad/s), belongs in [-1, 1].]
```
* KAZE
```bash
Param: KAZE/Diffusivity = "1"                              [Diffusivity type: 0=DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER.]
Param: KAZE/Extended = "false"                             [Set to enable extraction of extended (128-byte) descriptor.]
Param: KAZE/NOctaveLayers = "4"                            [Default number of sublevels per scale level.]
Param: KAZE/NOctaves = "4"                                 [Maximum octave evolution of the image.]
Param: KAZE/Threshold = "0.001"                            [Detector response threshold to accept keypoint.]
Param: KAZE/Upright = "false"                              [Set to enable use of upright descriptors (non rotation-invariant).]
```
* Kp
```bash
Param: Kp/BadSignRatio = "0.5"                             [Bad signature ratio (less than Ratio x AverageWordsPerImage = bad).]
Param: Kp/ByteToFloat = "false"                            [For Kp/NNStrategy=1, binary descriptors are converted to float by converting each byte to float instead of converting each bit to float. When converting bytes instead of bits, less memory is used and search is faster at the cost of slightly less accurate matching.]
Param: Kp/DetectorStrategy = "8"                           [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
Param: Kp/DictionaryPath = ""                              [Path of the pre-computed dictionary]
Param: Kp/FlannRebalancingFactor = "2.0"                   [Factor used when rebuilding the incremental FLANN index (see "Kp/IncrementalFlann"). Set <=1 to disable.]
Param: Kp/GridCols = "1"                                   [Number of columns of the grid used to extract uniformly "Kp/MaxFeatures / grid cells" features from each cell.]
Param: Kp/GridRows = "1"                                   [Number of rows of the grid used to extract uniformly "Kp/MaxFeatures / grid cells" features from each cell.]
Param: Kp/IncrementalDictionary = "true"                   []
Param: Kp/IncrementalFlann = "true"                        [When using FLANN based strategy, add/remove points to its index without always rebuilding the index (the index is built only when the dictionary increases of the factor "Kp/FlannRebalancingFactor" in size).]
Param: Kp/MaxDepth = "0"                                   [Filter extracted keypoints by depth (0=inf).]
Param: Kp/MaxFeatures = "500"                              [Maximum features extracted from the images (0 means not bounded, <0 means no extraction).]
Param: Kp/MinDepth = "0"                                   [Filter extracted keypoints by depth.]
Param: Kp/NNStrategy = "1"                                 [kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4]
Param: Kp/NewWordsComparedTogether = "true"                [When adding new words to dictionary, they are compared also with each other (to detect same words in the same signature).]
Param: Kp/NndrRatio = "0.8"                                [NNDR ratio (A matching pair is detected, if its distance is closer than X times the distance of the second nearest neighbor.)]
Param: Kp/Parallelized = "true"                            [If the dictionary update and signature creation were parallelized.]
Param: Kp/RoiRatios = "0.0 0.0 0.0 0.0"                    [Region of interest ratios [left, right, top, bottom].]
Param: Kp/SubPixEps = "0.02"                               [See cv::cornerSubPix().]
Param: Kp/SubPixIterations = "0"                           [See cv::cornerSubPix(). 0 disables sub pixel refining.]
Param: Kp/SubPixWinSize = "3"                              [See cv::cornerSubPix().]
Param: Kp/TfIdfLikelihoodUsed = "true"                     [Use of the td-idf strategy to compute the likelihood.]
Param: Rtabmap/StartNewMapOnGoodSignature = "false"        [Start a new map only if the first signature is not bad (i.e., has enough features, see Kp/BadSignRatio).]
```
* Marker
```bash
Param: Marker/CornerRefinementMethod = "0"                 [Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag2). For OpenCV <3.3.0, this is "doCornerRefinement" parameter: set 0 for false and 1 for true.]
Param: Marker/Dictionary = "0"                             [Dictionary to use: DICT_ARUCO_4X4_50=0, DICT_ARUCO_4X4_100=1, DICT_ARUCO_4X4_250=2, DICT_ARUCO_4X4_1000=3, DICT_ARUCO_5X5_50=4, DICT_ARUCO_5X5_100=5, DICT_ARUCO_5X5_250=6, DICT_ARUCO_5X5_1000=7, DICT_ARUCO_6X6_50=8, DICT_ARUCO_6X6_100=9, DICT_ARUCO_6X6_250=10, DICT_ARUCO_6X6_1000=11, DICT_ARUCO_7X7_50=12, DICT_ARUCO_7X7_100=13, DICT_ARUCO_7X7_250=14, DICT_ARUCO_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16, DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20]
Param: Marker/Length = "0"                                 [The length (m) of the markers side. 0 means automatic marker length estimation using the depth image (the camera should look at the marker perpendicularly for initialization).]
Param: Marker/MaxDepthError = "0.01"                       [Maximum depth error between all corners of a marker when estimating the marker length (when Marker/Length is 0). The smaller it is, the more perpendicular the camera should be toward the marker to initialize the length.]
Param: Marker/MaxRange = "0.0"                             [Maximum range in which markers will be detected. <=0 for unlimited range.]
Param: Marker/MinRange = "0.0"                             [Miniminum range in which markers will be detected. <=0 for unlimited range.]
Param: Marker/VarianceAngular = "0.01"                     [Angular variance to set on marker detections. Set to >=9999 to use only position (xyz) constraint in graph optimization.]
Param: Marker/VarianceLinear = "0.001"                     [Linear variance to set on marker detections.]
```
* Mem
```bash
Param: Mem/BadSignatures* Ignored = "false"                  [Bad signatures are ignored.]
Param: Mem/BinDataKept = "true"                            [Keep binary data in db.]
Param: Mem/CompressionParallelized = "true"                [Compression of sensor data is multi-threaded.]
Param: Mem/CovOffDiagIgnored = "true"                      [Ignore off diagonal values of the covariance matrix.]
Param: Mem/DepthAsMask = "true"                            [Use depth image as mask when extracting features for vocabulary.]
Param: Mem/GenerateIds = "true"                            [True=Generate location IDs, False=use input image IDs.]
Param: Mem/ImageCompressionFormat = ".jpg"                 [RGB image compression format. It should be ".jpg" or ".png".]
Param: Mem/ImageKept = "false"                             [Keep raw images in RAM.]
Param: Mem/ImagePostDecimation = "1"                       [Image decimation (>=1) of saved data in created signatures (after features extraction). Decimation is done from the original image.]
Param: Mem/ImagePreDecimation = "1"                        [Image decimation (>=1) before features extraction.]
Param: Mem/IncrementalMemory = "true"                      [SLAM mode, otherwise it is Localization mode.]
Param: Mem/InitWMWithAllNodes = "false"                    [Initialize the Working Memory with all nodes in Long-Term Memory. When false, it is initialized with nodes of the previous session.]
Param: Mem/IntermediateNodeDataKept = "false"              [Keep intermediate node data in db.]
Param: Mem/LaserScanDownsampleStepSize = "1"               [If > 1, downsample the laser scans when creating a signature.]
Param: Mem/LaserScanNormalK = "0"                          [If > 0 and laser scans dont have normals, normals will be computed with K search neighbors when creating a signature.]
Param: Mem/LaserScanNormalRadius = "0.0"                   [If > 0 m and laser scans dont have normals, normals will be computed with radius search neighbors when creating a signature.]
Param: Mem/LaserScanVoxelSize = "0.0"                      [If > 0 m, voxel filtering is done on laser scans when creating a signature. If the laser scan had normals, they will be removed. To recompute the normals, make sure to use "Mem/LaserScanNormalK" or "Mem/LaserScanNormalRadius" parameters.]
Param: Mem/LocalizationDataSaved = "false"                 [Save localization data during localization session (when Mem/IncrementalMemory=false). When enabled, the database will then also grow in localization mode. This mode would be used only for debugging purpose.]
Param: Mem/MapLabelsAdded = "true"                         [Create map labels. The first node of a map will be labelled as "map#" where # is the map ID.]
Param: Mem/NotLinkedNodesKept = "true"                     [Keep not linked nodes in db (rehearsed nodes and deleted nodes).]
Param: Mem/RawDescriptorsKept = "true"                     [Raw descriptors kept in memory.]
Param: Mem/RecentWmRatio = "0.2"                           [Ratio of locations after the last loop closure in WM that cannot be transferred.]
Param: Mem/ReduceGraph = "false"                           [Reduce graph. Merge nodes when loop closures are added (ignoring those with user data set).]
Param: Mem/RehearsalIdUpdatedToNewOne = "false"            [On merge, update to new id. When false, no copy.]
Param: Mem/RehearsalSimilarity = "0.6"                     [Rehearsal similarity.]
Param: Mem/RehearsalWeightIgnoredWhileMoving = "false"     [When the robot is moving, weights are not updated on rehearsal.]
Param: Mem/STMSize = "10"                                  [Short-term memory size.]
Param: Mem/SaveDepth16Format = "false"                     [Save depth image into 16 bits format to reduce memory used. Warning: values over ~65 meters are ignored (maximum 65535 millimeters).]
Param: Mem/StereoFromMotion = "false"                      [Triangulate features without depth using stereo from motion (odometry). It would be ignored if Mem/DepthAsMask is true and the feature detector used supports masking.]
Param: Mem/TransferSortingByWeightId = "false"             [On transfer, signatures are sorted by weight->ID only (i.e. the oldest of the lowest weighted signatures are transferred first). If false, the signatures are sorted by weight->Age->ID (i.e. the oldest inserted in WM of the lowest weighted signatures are transferred first). Note that retrieval updates the age, not the ID.]
Param: Mem/UseOdomFeatures = "true"                        [Use odometry features instead of regenerating them.]
Param: Mem/UseOdomGravity = "false"                        [Use odometry instead of IMU orientation to add gravity links to new nodes created. We assume that odometry is already aligned with gravity (e.g., we are using a VIO approach). Gravity constraints are used by graph optimization only if "Optimizer/GravitySigma" is not zero.]
Param: RGBD/MaxOdomCacheSize = "0"                         [Maximum odometry cache size. Used only in localization mode (when Mem/IncrementalMemory=false) and when RGBD/OptimizeMaxError!=0. This is used to verify localization transforms to make sure we dont teleport to a location very similar to one we previously localized on. When the cache is full, the whole cache is cleared and the next localization is automatically accepted without verification. Set 0 to disable caching.]
Param: RGBD/StartAtOrigin = "false"                        [If true, rtabmap will assume the robot is starting from origin of the map. If false, rtabmap will assume the robot is restarting from the last saved localization pose from previous session (the place where it shut down previously). Used only in localization mode (Mem/IncrementalMemory=false).]
```
* ORB
```bash
Param: ORB/EdgeThreshold = "19"                            [This is size of the border where the features are not detected. It should roughly match the patchSize parameter.]
Param: ORB/FirstLevel = "0"                                [It should be 0 in the current implementation.]
Param: ORB/Gpu = "false"                                   [GPU-ORB: Use GPU version of ORB. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.]
Param: ORB/NLevels = "3"                                   [The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).]
Param: ORB/PatchSize = "31"                                [size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.]
Param: ORB/ScaleFactor = "2"                               [Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.]
Param: ORB/ScoreType = "0"                                 [The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.]
Param: ORB/WTA_K = "2"                                     [The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).]
```
* Optimizer
```bash
Param: Optimizer/Epsilon = "0.0"                           [Stop optimizing when the error improvement is less than this value.]
Param: Optimizer/GravitySigma = "0.3"                      [Gravity sigma value (>=0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches, for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints. Currently supported only with g2o and GTSAM optimization strategies (see Optimizer/Strategy).]
Param: Optimizer/Iterations = "20"                         [Optimization iterations.]
Param: Optimizer/LandmarksIgnored = "false"                [Ignore landmark constraints while optimizing. Currently only g2o and gtsam optimization supports this.]
Param: Optimizer/PriorsIgnored = "true"                    [Ignore prior constraints (global pose or GPS) while optimizing. Currently only g2o and gtsam optimization supports this.]
Param: Optimizer/Robust = "false"                          [Robust graph optimization using Vertigo (only work for g2o and GTSAM optimization strategies). Not compatible with "RGBD/OptimizeMaxError" if enabled.]
Param: Optimizer/Strategy = "1"                            [Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.]
Param: Optimizer/VarianceIgnored = "false"                 [Ignore constraints variance. If checked, identity information matrix is used for each constraint. Otherwise, an information matrix is generated from the variance saved in the links.]
Param: RGBD/OptimizeMaxError = "3.0"                       [Reject loop closures if optimization error ratio is greater than this value (0=disabled). Ratio is computed as absolute error over standard deviation of each link. This will help to detect when a wrong loop closure is added to the graph. Not compatible with "Optimizer/Robust" if enabled.]
```
* PyDetector
```bash
Param: PyDetector/Cuda = "true"                            [Use cuda.]
Param: PyDetector/Path = ""                                [Path to python script file (see available ones in rtabmap/corelib/src/python/*). See the header to see where the script should be copied.]
```
* PyMatcher
```bash
Param: PyMatcher/Cuda = "true"                             [Used by SuperGlue.]
Param: PyMatcher/Iterations = "20"                         [Sinkhorn iterations. Used by SuperGlue.]
Param: PyMatcher/Model = "indoor"                          [For SuperGlue, set only "indoor" or "outdoor". For OANet, set path to one of the pth file (e.g., "OANet/model/gl3d/sift-4000/model_best.pth").]
Param: PyMatcher/Path = ""                                 [Path to python script file (see available ones in rtabmap/corelib/src/python/*). See the header to see where the script should be copied.]
Param: PyMatcher/Threshold = "0.2"                         [Used by SuperGlue.]
```
* RGBD
```bash
Param: RGBD/AngularSpeedUpdate = "0.0"                     [Maximum angular speed (rad/s) to update the map (0 means not limit).]
Param: RGBD/AngularUpdate = "0.1"                          [Minimum angular displacement (rad) to update the map. Rehearsal is done prior to this, so weights are still updated.]
Param: RGBD/CreateOccupancyGrid = "false"                  [Create local occupancy grid maps. See "Grid" group for parameters.]
Param: RGBD/Enabled = "true"                               [Activate metric SLAM. If set to false, classic RTAB-Map loop closure detection is done using only images and without any metric information.]
Param: RGBD/GoalReachedRadius = "0.5"                      [Goal reached radius (m).]
Param: RGBD/GoalsSavedInUserData = "false"                 [When a goal is received and processed with success, it is saved in user data of the location with this format: "GOAL:#".]
Param: RGBD/LinearSpeedUpdate = "0.0"                      [Maximum linear speed (m/s) to update the map (0 means not limit).]
Param: RGBD/LinearUpdate = "0.1"                           [Minimum linear displacement (m) to update the map. Rehearsal is done prior to this, so weights are still updated.]
Param: RGBD/LocalBundleOnLoopClosure = "false"             [Do local bundle adjustment with neighborhood of the loop closure.]
Param: RGBD/LocalImmunizationRatio = "0.25"                [Ratio of working memory for which local nodes are immunized from transfer.]
Param: RGBD/LocalRadius = "10"                             [Local radius (m) for nodes selection in the local map. This parameter is used in some approaches about the local map management.]
Param: RGBD/LoopClosureIdentityGuess = "false"             [Use Identity matrix as guess when computing loop closure transform, otherwise no guess is used, thus assuming that registration strategy selected (Reg/Strategy) can deal with transformation estimation without guess.]
Param: RGBD/LoopClosureReextractFeatures = "false"         [Extract features even if there are some already in the nodes.]
Param: RGBD/LoopCovLimited = "false"                       [Limit covariance of non-neighbor links to minimum covariance of neighbor links. In other words, if covariance of a loop closure link is smaller than the minimum covariance of odometry links, its covariance is set to minimum covariance of odometry links.]
Param: RGBD/MarkerDetection = "false"                      [Detect static markers to be added as landmarks for graph optimization. If input data have already landmarks, this will be ignored. See "Marker" group for parameters.]
Param: RGBD/MaxLocalRetrieved = "2"                        [Maximum local locations retrieved (0=disabled) near the current pose in the local map or on the current planned path (those on the planned path have priority).]
Param: RGBD/MaxLoopClosureDistance = "0.0"                 [Reject loop closures/localizations if the distance from the map is over this distance (0=disabled).]
Param: RGBD/MaxOdomCacheSize = "0"                         [Maximum odometry cache size. Used only in localization mode (when Mem/IncrementalMemory=false) and when RGBD/OptimizeMaxError!=0. This is used to verify localization transforms to make sure we dont teleport to a location very similar to one we previously localized on. When the cache is full, the whole cache is cleared and the next localization is automatically accepted without verification. Set 0 to disable caching.]
Param: RGBD/NeighborLinkRefining = "false"                 [When a new node is added to the graph, the transformation of its neighbor link to the previous node is refined using registration approach selected (Reg/Strategy).]
Param: RGBD/NewMapOdomChangeDistance = "0"                 [A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled).]
Param: RGBD/OptimizeFromGraphEnd = "false"                 [Optimize graph from the newest node. If false, the graph is optimized from the oldest node of the current graph (this adds an overhead computation to detect to oldest node of the current graph, but it can be useful to preserve the map referential from the oldest node). Warning when set to false: when some nodes are transferred, the first referential of the local map may change, resulting in momentary changes in robot/map position (which are annoying in teleoperation).]
Param: RGBD/OptimizeMaxError = "3.0"                       [Reject loop closures if optimization error ratio is greater than this value (0=disabled). Ratio is computed as absolute error over standard deviation of each link. This will help to detect when a wrong loop closure is added to the graph. Not compatible with "Optimizer/Robust" if enabled.]
Param: RGBD/PlanAngularVelocity = "0"                      [Angular velocity (rad/sec) used to compute path weights.]
Param: RGBD/PlanLinearVelocity = "0"                       [Linear velocity (m/sec) used to compute path weights.]
Param: RGBD/PlanStuckIterations = "0"                      [Mark the current goal node on the path as unreachable if it is not updated after X iterations (0=disabled). If all upcoming nodes on the path are unreachabled, the plan fails.]
Param: RGBD/ProximityAngle = "45"                          [Maximum angle (degrees) for one-to-one proximity detection.]
Param: RGBD/ProximityBySpace = "true"                      [Detection over locations (in Working Memory) near in space.]
Param: RGBD/ProximityByTime = "false"                      [Detection over all locations in STM.]
Param: RGBD/ProximityMaxGraphDepth = "50"                  [Maximum depth from the current/last loop closure location and the local loop closure hypotheses. Set 0 to ignore.]
Param: RGBD/ProximityMaxPaths = "3"                        [Maximum paths compared (from the most recent) for proximity detection. 0 means no limit.]
Param: RGBD/ProximityOdomGuess = "false"                   [Use odometry as motion guess for one-to-one proximity detection.]
Param: RGBD/ProximityPathFilteringRadius = "1"             [Path filtering radius to reduce the number of nodes to compare in a path in one-to-many proximity detection. The nearest node in a path should be inside that radius to be considered for one-to-one proximity detection.]
Param: RGBD/ProximityPathMaxNeighbors = "0"                [Maximum neighbor nodes compared on each path for one-to-many proximity detection. Set to 0 to disable one-to-many proximity detection (by merging the laser scans).]
Param: RGBD/ProximityPathRawPosesUsed = "true"             [When comparing to a local path for one-to-many proximity detection, merge the scans using the odometry poses (with neighbor link optimizations) instead of the ones in the optimized local graph.]
Param: RGBD/ScanMatchingIdsSavedInLinks = "true"           [Save scan matching IDs from one-to-many proximity detection in links user data.]
Param: RGBD/StartAtOrigin = "false"                        [If true, rtabmap will assume the robot is starting from origin of the map. If false, rtabmap will assume the robot is restarting from the last saved localization pose from previous session (the place where it shut down previously). Used only in localization mode (Mem/IncrementalMemory=false).]
Param: Rtabmap/LoopGPS = "true"                            [Use GPS to filter likelihood (if GPS is recorded). Only locations inside the local radius "RGBD/LocalRadius" of the current GPS location are considered for loop closure detection.]
Param: VhEp/Enabled = "false"                              [Verify visual loop closure hypothesis by computing a fundamental matrix. This is done prior to transformation computation when RGBD/Enabled is enabled.]
```
* Reg
```bash
Param: Reg/Force3DoF = "false"                             [Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0.]
Param: Reg/RepeatOnce = "true"                             [Do a second registration with the output of the first registration as guess. Only done if no guess was provided for the first registration (like on loop closure). It can be useful if the registration approach used can use a guess to get better matches.]
Param: Reg/Strategy = "0"                                  [0=Vis, 1=Icp, 2=VisIcp]
```
* Rtabmap
```bash
Param: Rtabmap/ComputeRMSE = "true"                        [Compute root mean square error (RMSE) and publish it in statistics, if ground truth is provided.]
Param: Rtabmap/CreateIntermediateNodes = "false"           [Create intermediate nodes between loop closure detection. Only used when Rtabmap/DetectionRate>0.]
Param: Rtabmap/DetectionRate = "1"                         [Detection rate (Hz). RTAB-Map will filter input images to satisfy this rate.]
Param: Rtabmap/ImageBufferSize = "1"                       [Data buffer size (0 min inf).]
Param: Rtabmap/ImagesAlreadyRectified = "true"             [Images are already rectified. By default RTAB-Map assumes that received images are rectified. If they are not, they can be rectified by RTAB-Map if this parameter is false.]
Param: Rtabmap/LoopGPS = "true"                            [Use GPS to filter likelihood (if GPS is recorded). Only locations inside the local radius "RGBD/LocalRadius" of the current GPS location are considered for loop closure detection.]
Param: Rtabmap/LoopRatio = "0"                             [The loop closure hypothesis must be over LoopRatio x lastHypothesisValue.]
Param: Rtabmap/LoopThr = "0.11"                            [Loop closing threshold.]
Param: Rtabmap/MaxRetrieved = "2"                          [Maximum locations retrieved at the same time from LTM.]
Param: Rtabmap/MemoryThr = "0"                             [Maximum nodes in the Working Memory (0 means infinity). Similar to "Rtabmap/TimeThr", when the number of nodes in Working Memory (WM) exceeds this treshold, some nodes are transferred to Long-Term Memory to keep WM size fixed.]
Param: Rtabmap/PublishLastSignature = "true"               [Publishing last signature.]
Param: Rtabmap/PublishLikelihood = "true"                  [Publishing likelihood.]
Param: Rtabmap/PublishPdf = "true"                         [Publishing pdf.]
Param: Rtabmap/PublishRAMUsage = "false"                   [Publishing RAM usage in statistics (may add a small overhead to get info from the system).]
Param: Rtabmap/PublishStats = "true"                       [Publishing statistics.]
Param: Rtabmap/RectifyOnlyFeatures = "false"               [If "Rtabmap/ImagesAlreadyRectified" is false and this parameter is true, the whole RGB image will not be rectified, only the features. Warning: As projection of RGB-D image to point cloud is assuming that images are rectified, the generated point cloud map will have wrong colors if this parameter is true.]
Param: Rtabmap/SaveWMState = "false"                       [Save working memory state after each update in statistics.]
Param: Rtabmap/StartNewMapOnGoodSignature = "false"        [Start a new map only if the first signature is not bad (i.e., has enough features, see Kp/BadSignRatio).]
Param: Rtabmap/StartNewMapOnLoopClosure = "false"          [Start a new map only if there is a global loop closure with a previous map.]
Param: Rtabmap/StatisticLogged = "false"                   [Logging enabled.]
Param: Rtabmap/StatisticLoggedHeaders = "true"             [Add column header description to log files.]
Param: Rtabmap/StatisticLogsBufferedInRAM = "true"         [Statistic logs buffered in RAM instead of written to hard drive after each iteration.]
Param: Rtabmap/TimeThr = "0"                               [Maximum time allowed for map update (ms) (0 means infinity). When map update time exceeds this fixed time threshold, some nodes in Working Memory (WM) are transferred to Long-Term Memory to limit the size of the WM and decrease the update time.]
Param: Rtabmap/WorkingDirectory = ""                       [Working directory.]
```
* SIFT
```bash
Param: SIFT/ContrastThreshold = "0.04"                     [The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.]
Param: SIFT/EdgeThreshold = "10"                           [The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).]
Param: SIFT/NFeatures = "0"                                [The number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast).]
Param: SIFT/NOctaveLayers = "3"                            [The number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution.]
Param: SIFT/RootSIFT = "false"                             [Apply RootSIFT normalization of the descriptors.]
Param: SIFT/Sigma = "1.6"                                  [The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.]
```
* SURF
```bash
Param: SURF/Extended = "false"                             [Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).]
Param: SURF/GpuKeypointsRatio = "0.01"                     [Used with SURF GPU.]
Param: SURF/GpuVersion = "false"                           [GPU-SURF: Use GPU version of SURF. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.]
Param: SURF/HessianThreshold = "500"                       [Threshold for hessian keypoint detector used in SURF.]
Param: SURF/OctaveLayers = "2"                             [Number of octave layers within each octave.]
Param: SURF/Octaves = "4"                                  [Number of pyramid octaves the keypoint detector will use.]
Param: SURF/Upright = "false"                              [Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).]
Param: Vis/FeatureType = "8"                               [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
```
* Stereo
```bash
Param: Stereo/DenseStrategy = "0"                          [0=cv::StereoBM, 1=cv::StereoSGBM]
Param: Stereo/Eps = "0.01"                                 [[Stereo/OpticalFlow=true] Epsilon stop criterion.]
Param: Stereo/Iterations = "30"                            [Maximum iterations.]
Param: Stereo/MaxDisparity = "128.0"                       [Maximum disparity.]
Param: Stereo/MaxLevel = "5"                               [Maximum pyramid level.]
Param: Stereo/MinDisparity = "0.5"                         [Minimum disparity.]
Param: Stereo/OpticalFlow = "true"                         [Use optical flow to find stereo correspondences, otherwise a simple block matching approach is used.]
Param: Stereo/SSD = "true"                                 [[Stereo/OpticalFlow=false] Use Sum of Squared Differences (SSD) window, otherwise Sum of Absolute Differences (SAD) window is used.]
Param: Stereo/WinHeight = "3"                              [Window height.]
Param: Stereo/WinWidth = "15"                              [Window width.]
```
* StereoBM
```bash
Param: StereoBM/BlockSize = "15"                           [See cv::StereoBM]
Param: StereoBM/Disp12MaxDiff = "-1"                       [See cv::StereoBM]
Param: StereoBM/MinDisparity = "0"                         [See cv::StereoBM]
Param: StereoBM/NumDisparities = "128"                     [See cv::StereoBM]
Param: StereoBM/PreFilterCap = "31"                        [See cv::StereoBM]
Param: StereoBM/PreFilterSize = "9"                        [See cv::StereoBM]
Param: StereoBM/SpeckleRange = "4"                         [See cv::StereoBM]
Param: StereoBM/SpeckleWindowSize = "100"                  [See cv::StereoBM]
Param: StereoBM/TextureThreshold = "10"                    [See cv::StereoBM]
Param: StereoBM/UniquenessRatio = "15"                     [See cv::StereoBM]
```
* StereoSGBM
```bash
Param: StereoSGBM/BlockSize = "15"                         [See cv::StereoSGBM]
Param: StereoSGBM/Disp12MaxDiff = "1"                      [See cv::StereoSGBM]
Param: StereoSGBM/MinDisparity = "0"                       [See cv::StereoSGBM]
Param: StereoSGBM/Mode = "2"                               [See cv::StereoSGBM]
Param: StereoSGBM/NumDisparities = "128"                   [See cv::StereoSGBM]
Param: StereoSGBM/P1 = "2"                                 [See cv::StereoSGBM]
Param: StereoSGBM/P2 = "5"                                 [See cv::StereoSGBM]
Param: StereoSGBM/PreFilterCap = "31"                      [See cv::StereoSGBM]
Param: StereoSGBM/SpeckleRange = "4"                       [See cv::StereoSGBM]
Param: StereoSGBM/SpeckleWindowSize = "100"                [See cv::StereoSGBM]
Param: StereoSGBM/UniquenessRatio = "20"                   [See cv::StereoSGBM]
```
* SuperPoint
```bash
Param: SuperPoint/Cuda = "true"                            [Use Cuda device for Torch, otherwise CPU device is used by default.]
Param: SuperPoint/ModelPath = ""                           [[Required] Path to pre-trained weights Torch file of SuperPoint (*.pt).]
Param: SuperPoint/NMS = "true"                             [If true, non-maximum suppression is applied to detected keypoints.]
Param: SuperPoint/NMSRadius = "4"                          [[SuperPoint/NMS=true] Minimum distance (pixels) between keypoints.]
Param: SuperPoint/Threshold = "0.010"                      [Detector response threshold to accept keypoint.]
```
* VhEp
```bash
Param: VhEp/Enabled = "false"                              [Verify visual loop closure hypothesis by computing a fundamental matrix. This is done prior to transformation computation when RGBD/Enabled is enabled.]
Param: VhEp/MatchCountMin = "8"                            [Minimum of matching visual words pairs to accept the loop hypothesis.]
Param: VhEp/RansacParam1 = "3"                             [Fundamental matrix (see cvFindFundamentalMat()): Max distance (in pixels) from the epipolar line for a point to be inlier.]
Param: VhEp/RansacParam2 = "0.99"                          [Fundamental matrix (see cvFindFundamentalMat()): Performance of RANSAC.]
```
* Vis
```bash
Param: Vis/BundleAdjustment = "1"                          [Optimization with bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.]
Param: Vis/CorFlowEps = "0.01"                             [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
Param: Vis/CorFlowIterations = "30"                        [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
Param: Vis/CorFlowMaxLevel = "3"                           [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
Param: Vis/CorFlowWinSize = "16"                           [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
Param: Vis/CorGuessMatchToProjection = "false"             [[Vis/CorType=0] Match frames corners to sources projected points (when guess transform is provided) instead of projected points to frames corners.]
Param: Vis/CorGuessWinSize = "40"                          [[Vis/CorType=0] Matching window size (pixels) around projected points when a guess transform is provided to find correspondences. 0 means disabled.]
Param: Vis/CorNNDR = "0.8"                                 [[Vis/CorType=0] NNDR: nearest neighbor distance ratio. Used for knn features matching approach.]
Param: Vis/CorNNType = "1"                                 [[Vis/CorType=0] kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7. Used for features matching approach.]
Param: Vis/CorType = "0"                                   [Correspondences computation approach: 0=Features Matching, 1=Optical Flow]
Param: Vis/DepthAsMask = "true"                            [Use depth image as mask when extracting features.]
Param: Vis/EpipolarGeometryVar = "0.1"                     [[Vis/EstimationType = 2] Epipolar geometry maximum variance to accept the transformation.]
Param: Vis/EstimationType = "1"                            [Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)]
Param: Vis/FeatureType = "8"                               [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
Param: Vis/ForwardEstOnly = "true"                         [Forward estimation only (A->B). If false, a transformation is also computed in backward direction (B->A), then the two resulting transforms are merged (middle interpolation between the transforms).]
Param: Vis/GridCols = "1"                                  [Number of columns of the grid used to extract uniformly "Vis/MaxFeatures / grid cells" features from each cell.]
Param: Vis/GridRows = "1"                                  [Number of rows of the grid used to extract uniformly "Vis/MaxFeatures / grid cells" features from each cell.]
Param: Vis/InlierDistance = "0.1"                          [[Vis/EstimationType = 0] Maximum distance for feature correspondences. Used by 3D->3D estimation approach.]
Param: Vis/Iterations = "300"                              [Maximum iterations to compute the transform.]
Param: Vis/MaxDepth = "0"                                  [Max depth of the features (0 means no limit).]
Param: Vis/MaxFeatures = "1000"                            [0 no limits.]
Param: Vis/MeanInliersDistance = "0.0"                     [Maximum distance (m) of the mean distance of inliers from the camera to accept the transformation. 0 means disabled.]
Param: Vis/MinDepth = "0"                                  [Min depth of the features (0 means no limit).]
Param: Vis/MinInliers = "20"                               [Minimum feature correspondences to compute/accept the transformation.]
Param: Vis/MinInliersDistribution = "0.0"                  [Minimum distribution value of the inliers in the image to accept the transformation. The distribution is the second eigen value of the PCA (Principal Component Analysis) on the keypoints of the normalized image [-0.5, 0.5]. The value would be between 0 and 0.5. 0 means disabled.]
Param: Vis/PnPFlags = "0"                                  [[Vis/EstimationType = 1] PnP flags: 0=Iterative, 1=EPNP, 2=P3P]
Param: Vis/PnPRefineIterations = "0"                       [[Vis/EstimationType = 1] Refine iterations. Set to 0 if "Vis/BundleAdjustment" is also used.]
Param: Vis/PnPReprojError = "2"                            [[Vis/EstimationType = 1] PnP reprojection error.]
Param: Vis/RefineIterations = "5"                          [[Vis/EstimationType = 0] Number of iterations used to refine the transformation found by RANSAC. 0 means that the transformation is not refined.]
Param: Vis/RoiRatios = "0.0 0.0 0.0 0.0"                   [Region of interest ratios [left, right, top, bottom].]
Param: Vis/SubPixEps = "0.02"                              [See cv::cornerSubPix().]
Param: Vis/SubPixIterations = "0"                          [See cv::cornerSubPix(). 0 disables sub pixel refining.]
Param: Vis/SubPixWinSize = "3"                             [See cv::cornerSubPix().]
```
* g2o
```bash
Param: g2o/Baseline = "0.075"                              [When doing bundle adjustment with RGB-D data, we can set a fake baseline (m) to do stereo bundle adjustment (if 0, mono bundle adjustment is done). For stereo data, the baseline in the calibration is used directly.]
Param: g2o/Optimizer = "0"                                 [0=Levenberg 1=GaussNewton]
Param: g2o/PixelVariance = "1.0"                           [Pixel variance used for bundle adjustment.]
Param: g2o/RobustKernelDelta = "8"                         [Robust kernel delta used for bundle adjustment (0 means dont use robust kernel). Observations with chi2 over this threshold will be ignored in the second optimization pass.]
Param: g2o/Solver = "0"                                    [0=csparse 1=pcg 2=cholmod 3=Eigen]
```

# PARAMETERS LIST EKF

```bash
/ekf_local/base_link_frame
/ekf_local/debug
/ekf_local/frequency
/ekf_local/imu0
/ekf_local/imu0_config
/ekf_local/imu0_differential
/ekf_local/imu0_linear_acceleration_rejection_threshold
/ekf_local/imu0_nodelay
/ekf_local/imu0_pose_rejection_threshold
/ekf_local/imu0_queue_size
/ekf_local/imu0_relative
/ekf_local/imu0_remove_gravitational_acceleration
/ekf_local/imu0_twist_rejection_threshold
/ekf_local/initial_estimate_covariance
/ekf_local/map_frame
/ekf_local/odom0
/ekf_local/odom0_config
/ekf_local/odom0_differential
/ekf_local/odom0_nodelay
/ekf_local/odom0_pose_rejection_threshold
/ekf_local/odom0_queue_size
/ekf_local/odom0_relative
/ekf_local/odom0_twist_rejection_threshold
/ekf_local/odom_frame
/ekf_local/print_diagnostics
/ekf_local/process_noise_covariance
/ekf_local/publish_tf
/ekf_local/sensor_timeout
/ekf_local/transform_time_offset
/ekf_local/transform_timeout
/ekf_local/two_d_mode
/ekf_local/use_control
/ekf_local/world_frame
/ekf_global/base_link_frame
/ekf_global/debug
/ekf_global/frequency
/ekf_global/imu0
/ekf_global/imu0_config
/ekf_global/imu0_differential
/ekf_global/imu0_linear_acceleration_rejection_threshold
/ekf_global/imu0_nodelay
/ekf_global/imu0_pose_rejection_threshold
/ekf_global/imu0_queue_size
/ekf_global/imu0_relative
/ekf_global/imu0_remove_gravitational_acceleration
/ekf_global/imu0_twist_rejection_threshold
/ekf_global/initial_estimate_covariance
/ekf_global/map_frame
/ekf_global/odom0
/ekf_global/odom0_config
/ekf_global/odom0_differential
/ekf_global/odom0_nodelay
/ekf_global/odom0_pose_rejection_threshold
/ekf_global/odom0_queue_size
/ekf_global/odom0_relative
/ekf_global/odom0_twist_rejection_threshold
/ekf_global/odom1
/ekf_global/odom1_config
/ekf_global/odom1_differential
/ekf_global/odom1_nodelay
/ekf_global/odom1_queue_size
/ekf_global/odom1_relative
/ekf_global/odom_frame
/ekf_global/print_diagnostics
/ekf_global/process_noise_covariance
/ekf_global/publish_tf
/ekf_global/sensor_timeout
/ekf_global/transform_time_offset
/ekf_global/transform_timeout
/ekf_global/two_d_mode
/ekf_global/use_control
/ekf_global/world_frame* 
```
