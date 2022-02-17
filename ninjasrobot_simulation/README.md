# Usage (for collaborators)
Make sure: **you are using Ubuntu 20.04 and ROS2 Galactic** (Yes, these versions matter). Open a terminal and perform following steps:
1. Create a workspace.
```bash
cd ~
mkdir -p ninjasrobot_ws/src
```
2. Build simulation package
```bash
cd ~/ninjasrobot_ws/src
git@github.com:KingKorb/RobotNinjas.git
cd ..
colcon build
```
3. Launch simulation
```bash
source ~/ninjasrobot_ws/install/setup.bash
ros2 launch ninjasrobot_simulation sim_ninjasrobot.launch.py 
```

