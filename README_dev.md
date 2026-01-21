# Developer Notes

These are notes on what setup and configuration the real robots need to support the assignments. Students don't have to read this.






## Real Robot Network Configuration

Connect with network cable on the laptop
Configure IP & Test Ping

```Bash
# Go2
chmod +x go2_net_setup.sh
./go2_net_setup.sh

# Go2-w
chmod +x go2w_net_setup.sh
./go2w_net_setup.sh
```

## Container Environment Setup

Build the docker
```Bash
docker build -f docker/Dockerfile  --progress plain .

docker run -it \
  --net=host --ipc=host \
  --rm \
  --device /dev/dri/ \
  -e XAUTHORITY=/tmp/.Xauthority \
  test \
  /bin/bash
```


Inside the docker
```Bash
source /opt/ros/humble/setup.bash

mkdir ~/ros_ws/src
cd ~/ros_ws/src

git clone https://github.com/Unitree-Go2-Robot/go2_robot.git -b humble

# prepare third party repos
vcs import < go2_robot/dependencies.repos

cd ~/ros_ws
sudo rosdep init
rosdep update

sudo apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install 


ros2 launch go2_bringup go2.launch.py

```
