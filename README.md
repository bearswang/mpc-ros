# MPC-ROS
A Python ROS package for model predictive control (MPC) of an autonomous vehicle

BY Ruihua Han, Guoliang Li, and Shuai Wang

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
