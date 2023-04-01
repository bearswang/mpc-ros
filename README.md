# MPC-ROS
A Python ROS package for model predictive control (MPC) of an autonomous vehicle

# Install MPC-ROS:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/bearswang/mpc-ros.git
cd .. && catkin_make
```

# Use Example: Carla-ROS-MPC

# Start CARLA server
```bash
cd $CARLA_ROOT
./CarlaUE4.sh
```

# Spawn agents and start CARLA-ROS-Bridge 
```bash
cd $CARLA_ROS_BRIDGE
source devel/setup.bash
roslaunch carla_ros_bridge run_car_sim_Town04.launch 
```

# Run MPC algorithm 
```bash
source devel/setup.bash
rosrun mpc_ros mpc_Town04_launch.py
```

## Acknowledgement

* [Carla](https://github.com/carla-simulator)
* [Carla-ROS-Bridge](https://github.com/carla-simulator/ros-bridge)
* [RDA](https://github.com/carla-simulator/ros-bridge](https://github.com/hanruihua/RDA_planner)

### Authors

[Shuai Wang](https://github.com/bearswang)

[Ruihua Han](https://github.com/hanruihua)

[Guoliang Li](https://github.com/ReusLI1998)


