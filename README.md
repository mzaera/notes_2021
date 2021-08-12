# ROBOTS

Connect to them:
```bash
ssh transporter
```

```bash
ssh vinebot
```

```bash
ssh ranger
```

Run full system:

```bash
roslaunch transporter_launcher full_system.launch
```

```bash
roslaunch vinebot_launcher full_system.launch
```

```bash
roslaunch ranger_launcher full_system.launch
```
Run rviz:

```bash
rosrun rviz rviz -d /home/mzaera/Documents/rviz_config/transporter_config.rviz
```

```bash
rosrun rviz rviz -d /home/mzaera/Documents/rviz_config/vinebot_config.rviz
```

```bash
rosrun rviz rviz -d /home/mzaera/Documents/rviz_config/ranger_config.rviz
```


## Initial Configs
*NO cakin_make if some ccp file is modified should do catkin build*

### SSH

Empty!

### File folders

Other locations & connect:

```bash
sftp://solarcleano@192.168.8.70
```

And right click + "Add Bookmark".

### Bash

Empty!

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

You can obtain it form this git.

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

* Echo topic only oe msg:

```bash
rostopic echo -n1 /topic_name
```
