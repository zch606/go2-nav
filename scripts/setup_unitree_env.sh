#!/bin/bash
# ============================================================
# 宇树 Go2 环境搭建脚本
# 在当前 osrf/ros:humble-desktop-full 容器内运行此脚本
# 用法: bash scripts/setup_unitree_env.sh
# ============================================================
set -e

echo "=== Step 1: 安装系统依赖 ==="
sudo apt update
sudo apt install -y libyaml-cpp-dev
dpkg -l | grep rmw-cyclonedds-cpp || sudo apt install -y ros-humble-rmw-cyclonedds-cpp
dpkg -l | grep rosidl-generator-dds-idl || sudo apt install -y ros-humble-rosidl-generator-dds-idl
echo "> 系统依赖安装完成"

echo ""
echo "=== Step 2: 拉取 unitree_ros2 仓库 ==="
cd /root/ws/src
if [ -d "unitree_ros2" ]; then
    echo "> unitree_ros2 已存在，跳过 clone"
else
    git clone https://github.com/unitreerobotics/unitree_ros2.git
    echo "> unitree_ros2 拉取完成"
fi

echo ""
echo "=== Step 3: 编译 unitree_api 和 unitree_go 消息包 ==="
cd /root/ws/src/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select unitree_api unitree_go
echo "> cyclonedds_ws 编译完成"

echo ""
echo "=== Step 4: 编译 dog_nav 项目 ==="
cd /root/ws
source /opt/ros/humble/setup.bash
source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select dog_nav_interfaces dog_nav_step56
echo "> dog_nav 项目编译完成"

echo ""
echo "=== 全部完成! ==="
echo ""
echo "使用时 source 环境:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /root/ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
echo "  source /root/ws/install/setup.bash"
echo ""
echo "网络配置 (连接 Go2 真机时):"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>"
echo "      <NetworkInterface name=\"enp3s0\" priority=\"default\" multicast=\"default\" />"
echo "  </Interfaces></General></Domain></CycloneDDS>'"
echo ""
echo "启动 bridge 节点:"
echo "  ros2 launch dog_nav_step56 unitree_bridge.launch.py"
