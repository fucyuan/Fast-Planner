# Fast-Planner

**Fast-Planner** 是一个为四旋翼无人机在复杂未知环境中实现快速飞行而开发的规划框架。它包含了一套精心设计的规划算法，同时为一些流行的开源无人机项目（如 [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)、[FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) 和 [RACER](https://github.com/SYSU-STAR/RACER)）提供了基础的代码框架和算法支持。

**最新消息：**

- **2021年3月13日**：快速自主探索的代码现已开放！详情请查看 [此仓库](https://github.com/HKUST-Aerial-Robotics/FUEL)。
- **2020年10月20日**：Fast-Planner 被扩展并应用于快速自主探索。详情请查看 [此仓库](https://github.com/HKUST-Aerial-Robotics/FUEL)。

**作者**：来自 [HUKST Aerial Robotics Group](http://uav.ust.hk/) 的 [Boyu Zhou](http://sysu-star.com) 和 [Shaojie Shen](http://uav.ust.hk/group/)，以及来自 ZJU FAST Lab 的 [Fei Gao](http://zju-fast.com/fei-gao/)。

<p align="center">
  <img src="files/raptor1.gif" width = "400" height = "225"/>
  <img src="files/raptor2.gif" width = "400" height = "225"/>
  <img src="files/icra20_2.gif" width = "400" height = "225"/>
  <img src="files/ral19_2.gif" width = "400" height = "225"/>
</p>

完整视频：
[视频1](https://www.youtube.com/watch?v=NvR8Lq2pmPg&feature=emb_logo)、
[视频2](https://www.youtube.com/watch?v=YcEaFTjs-a0)、
[视频3](https://www.youtube.com/watch?v=toGhoGYyoAY)。
相关演示已在 IEEE Spectrum 报道：[页面1](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-nasa-lemur-robot)、[页面2](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-india-space-humanoid-robot)、[页面3](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-soft-exoskeleton-glove-extra-thumb)（在页面中搜索 _HKUST_）。

如果本项目对您有帮助，请为我们加颗星 :star:！我们付出了巨大的努力来开发和维护它 :grin::grin:。

## 目录

* [快速开始](#1-快速开始)
* [算法与论文](#2-算法与论文)
* [安装与配置](#3-安装与配置)
* [运行仿真](#4-运行仿真)
* [在项目中使用](#5-在项目中使用)
* [更新日志](#6-更新日志)
* [已知问题](#已知问题)

## 1. 快速开始

本项目已在 Ubuntu 18.04（ROS Melodic）和 20.04（ROS Noetic）上测试。

首先，您需要安装 **nlopt v2.7.1**：
```bash
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

接下来，安装其他必要工具：
```bash
sudo apt-get install libarmadillo-dev
```

然后克隆并编译本项目：
```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git
cd ../
catkin_make
```

您可以查看详细的[安装说明](#3-安装与配置)以完成项目设置。
编译完成后，启动可视化：
```bash
source devel/setup.bash && roslaunch plan_manage rviz.launch
```
并在新终端中启动仿真：
```bash
source devel/setup.bash && roslaunch plan_manage kino_replan.launch
```
您将在 ```Rviz``` 中看到随机生成的地图和无人机。可以使用 ```2D Nav Goal``` 工具为无人机选择目标点。

## 2. 算法与论文

本项目包含一组鲁棒且计算高效的四旋翼快速飞行规划算法：
- 动态可行路径搜索
- 基于 B 样条的轨迹优化
- 拓扑路径搜索及路径引导优化
- 感知优化规划策略（即将发布）

详细算法请参考以下论文。

如果您在研究中使用本项目，请至少引用以下论文之一：[Bibtex](files/bib.txt)。

- [**Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight**](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019.
- [**Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths**](https://arxiv.org/abs/1912.12644), Boyu Zhou, Fei Gao, Jie Pan and Shaojie Shen, IEEE International Conference on Robotics and Automation (**ICRA**), 2020.
- [**RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight**](https://arxiv.org/abs/2007.03465), Boyu Zhou, Jie Pan, Fei Gao and Shaojie Shen, IEEE Transactions on Robotics (**T-RO**)。

所有规划算法及其他关键模块（如建图）均在 **fast_planner** 文件夹中实现：

- **plan_env**：在线建图算法，接收深度图像（或点云）及相机位姿（里程计）数据，使用光线投射更新概率体素图，并构建欧几里得签名距离场（ESDF）以供规划模块使用。
- **path_searching**：前端路径搜索算法，包括动态可行路径搜索和拓扑路径搜索。
- **bspline**：B 样条轨迹表示的实现。
- **bspline_opt**：基于梯度的 B 样条轨迹优化。
- **active_perception**：感知优化规划策略，使无人机主动观察并避免未知障碍物（即将发布）。
- **plan_manage**：高层管理模块，用于调度建图和规划算法，并包含系统启动接口及配置文件。

## 3. 安装与配置

### 先决条件

1. 软件已在 Ubuntu 18.04（ROS Melodic）和 20.04（ROS Noetic）上开发并测试。
2. 使用 [**NLopt**](https://nlopt.readthedocs.io/en/latest/NLopt_Installation) 解决非线性优化问题，使用 **Armadillo** 提供线性代数支持。

首先，安装 **nlopt v2.7.1**：
```bash
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

然后安装其他必要工具：
```bash
sudo apt-get install libarmadillo-dev
```

### 在 ROS 中构建

完成上述依赖安装后，将此仓库克隆到 catkin 工作空间并编译：
```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git
cd ../
catkin_make
```

若遇到问题，请优先查看已有 **issues**、**pull requests** 和 **Google**，然后再提新的问题。

## 4. 运行仿真

首先运行带有配置的 [Rviz](http://wiki.ros.org/rviz)：
```bash
source devel/setup.bash
roslaunch plan_manage rviz.launch
```
然后运行四旋翼模拟器和 **Fast-Planner**：

### 动态路径搜索与 B 样条优化

此方法使用动态路径搜索在离散化控制空间中找到安全、动态可行且最小时间的初始轨迹。然后通过 B 样条优化改善轨迹的平滑性和避障性能。
运行以下命令：
```bash
source devel/setup.bash
roslaunch plan_manage kino_replan.launch
```

在 ```Rviz``` 中，使用 ```2D Nav Goal``` 工具点击目标点，即可生成新的轨迹并由无人机执行。

## 5. 在项目中使用

若已成功运行仿真，可查看文件 `kino_replan.launch` 或 `topo_replan.launch` 来了解如何在您的项目中使用 **Fast-Planner**。

## 6. 更新日志

- **2020年10月20日**：扩展并应用于快速自主探索。
- **2020年7月5日**：即将发布关于 RAPTOR 的实现。
- **2020年4月12日**：ICRA2020 论文相关实现已发布。

## 致谢
我们使用 **NLopt** 进行非线性优化。

## 许可证
源码基于 [GPLv3](http://www.gnu.org/licenses/) 许可证发布。

## 声明
本项目为研究代码，提供时不附带任何形式的担保，包括适销性或特定用途的适用性担保。

