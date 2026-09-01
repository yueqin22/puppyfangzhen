#!/bin/bash
# 配置 CoppeliaSim 环境变量
grep -q 'COPPELIASIM_ROOT_DIR' ~/.bashrc || {
  echo 'export COPPELIASIM_ROOT_DIR=~/CoppeliaSim' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT_DIR' >> ~/.bashrc
}
export COPPELIASIM_ROOT_DIR=~/CoppeliaSim
echo "COPPELIASIM_ROOT_DIR=$COPPELIASIM_ROOT_DIR"
echo "---"
echo "检查预编译 ROS2 插件:"
ls ~/CoppeliaSim/compiledROSPlugins/ 2>/dev/null || echo "无 compiledROSPlugins 目录"
echo "---"
ls ~/CoppeliaSim/libsimROS2* 2>/dev/null || echo "无预编译 simROS2 插件（需要编译）"
