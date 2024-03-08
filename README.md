# CARLA_Autoware
# ROS2/Autoware.universe bridge for CARLA simulator


### Thanks to https://github.com/gezp for ROS2 Humble support for CARLA ROS bridge. 
 This ros package enables autonomous driving using Autoware in addition to the basic function of the official [ros-bridge](https://github.com/carla-simulator/ros-bridge) package (communication between ros and carla). (https://github.com/gezp for ROS2 Humble)
- Make sure to Download the Python egg for 3.10 from [here](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.14-ubuntu-22.04). 
- Add the egg file to the folder: ../CARLA_0.9.14/PythonAPI/carla/dist

# Environment 
|ubuntu|ros|carla|autoware|
|:---:|:---:|:---:|:---:|
|22.04|humble|0.9.14|universe/master|

# Setup
## install
* [Autoware.Universe](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/) 
* [CARLA Installation](https://carla.readthedocs.io/en/latest/start_quickstart/) 
* [autoware containts](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)  
  1. Download maps (y-axis inverted version) to arbitaly location
  2. Change names. (point_cloud/Town01.pcd -> Town01/pointcloud_map.pcd, vector_maps/lanelet2/Town01.osm -> Town01/lanelet2_map.osm)
* Clone this repositories and ROSBridge
  ```
  git clone https://github.com/mraditya01/CARLA_Autoware.git
  git clone --recurse-submodules https://github.com/gezp/carla_ros.git -b humble-carla-0.9.14
  git clone https://github.com/astuff/astuff_sensor_msgs.git
  ```
 * Copy the files (sensor_kit_calibration.yaml, sensors.calibration.yaml) from folder "GNSS_interface/src/carla_sensor_kit_launch/carla_sensor_kit_description/config" to "src/param/autoware_individual_params/carla_sensor_kit".


## build
```bash
cd colcon_ws
colcon build --symlink-install
```

# Run
1. Run carla, change map, spawn object if you need
```bash
cd CARLA
./CarlaUE4.sh -prefernvidia -quality-level=Low
```

2. Run ros nodes
```bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py role_name:='ego_vehicle'
ros2 launch carla_autoware carla_autoware.launch.py
ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/Town01 vehicle_model:=sample_vehicle sensor_model:=carla_sensor_kit
```

3. Set initial pose (Init by GNSS)
4. Set goal position
5. Wait for planning
6. Engage
