# CARLA-ROS-MPC
BY Ruihua Han, Guoliang Li, and Shuai Wang

# Install Carla-ROS-MPC:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/bearswang/carla-ros-mpc.git
cd .. && catkin_make
```

# Start CARLA server
cd $CARLA_ROOT
./CarlaUE4.sh

# Spawn agents and start CARLA-ROS-Bridge 
source devel/setup.bash
roslaunch carla_ros_bridge run_car_sim_Town04.launch 

# Run MPC algorithm 
source devel/setup.bash
rosrun mpc_ros mpc_Town04_launch.py