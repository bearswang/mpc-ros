# MPC-ROS
A python ROS package for model predictive control (MPC) of an autonomous vehicle

## Install MPC-ROS:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/bearswang/mpc-ros.git
cd .. && catkin_make
```


## Example: MPC-ROS in Carla Simulation

Use the MPC algorithm to navigate an autonomous vehicle in the Carla Town04 map

https://user-images.githubusercontent.com/107024891/231101881-44654b2d-cf1e-4530-8aec-bc2e5b88f802.mp4


### 1. Test Environment

- Ubuntu 20.04
- CUDA 11.3 (Nvidia Driver 470)
- Python 3.8
- ROS Noetic
- CARLA 0.9.13
- Carla-ROS-Bridge

### 2. Install Carla

See the website [Carla](https://github.com/carla-simulator/carla)

### 3. Install Carla-ROS-bridge with our configurations in Town04

```bash
mkdir -p ~/ros-bridge/src
cd ~/ros-bridge/src
git clone https://github.com/bearswang/ros-bridge.git
cd .. && catkin_make
```

### 4. Start Carla server
```bash
cd $CARLA_ROOT
./CarlaUE4.sh
```
CARLA_ROOT is the root folder for [Carla](https://github.com/carla-simulator/carla) 

### 5. Spawn agents and launch Carla-ROS-Bridge
```bash
cd $CARLA_ROS_BRIDGE
source devel/setup.bash
roslaunch carla_ros_bridge run_car_sim_Town04.launch 
```
CARLA_ROS_BRIDGE is the root folder for [Carla-ROS-Bridge](https://github.com/bearswang/ros-bridge) (our forked version)

### 6a. Run MPC algorithm 
```bash
source devel/setup.bash
rosrun mpc_ros mpc_Town04_launch.py
```

### 6b. Run MPC algorithm with configuration

```
roslaunch mpc_ros mpc_Town04.launch
```


## Citation

MPC-ROS can reproduce the experimental results in the following paper:

```
@article{RDA,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Zhang, Qianru and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Robotics and Automation Letters}, 
  title={RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments}, 
  year={2023},
  volume={8},
  number={3},
  pages={1715-1722},
  publisher={IEEE}
```

```tex
@article{CarlaMPC,
  title={Collision Avoidance Predictive Motion Planning Based on Integrated Perception and V2V Communication},
  author={Shiyao Zhang and Shuai Wang and Shuai Yu and James J. Q. Yu and Miaowen Wen},
  journal={IEEE Transactions on Intelligent Transportation Systems},
  year={2022},
  volume={23},
  number={7},
  pages={9640-9653},
  publisher={IEEE}
}
```


## Acknowledgement
* This repo was supported by the Open Research Fund from Guangdong Laboratory of Artificial Intelligence and Digital Economy (Shenzhen) under Grant GML-KF-22-17.
* [Carla](https://github.com/carla-simulator/carla)
* [Carla-ROS-Bridge](https://github.com/carla-simulator/ros-bridge)
* [RDA](https://github.com/hanruihua/RDA_planner)

## Authors

[Ruihua Han](https://github.com/hanruihua)

[Shuai Wang](https://github.com/bearswang)

[Guoliang Li](https://github.com/GuoliangLI1998)


