# Multi-Sensor Fusion for Localization & Mapping -- 多传感器融合定位与建图

深蓝学院[多传感器融合定位与建图/Multi-Sensor Fusion for Localization & Mapping](https://www.shenlanxueyuan.com/course/324)学习环境.

Maintained by **Ge Yao**, alexgecontrol@qq.com

---

## 先修课程与准备知识检查

我在学习这门课程时，已经完成如下深蓝学院课程的学习:

* [机器人学中的状态估计](https://www.shenlanxueyuan.com/course/421)
* [视觉SLAM理论与实践](https://www.shenlanxueyuan.com/course/433)
* [视觉SLAM进阶-从零开始手写VIO](https://www.shenlanxueyuan.com/course/388)
* [激光SLAM理论与实践](https://www.shenlanxueyuan.com/course/348)

且有多年的机器人系统开发经验, 熟悉以下的基础理论, 开发语言/框架/工具:

* **State Estimation for Robotics** [Reference Book](https://www.amazon.com/State-Estimation-Robotics-Timothy-Barfoot/dp/1107159393/ref=sr_1_1?crid=3A1AU9FS3OA4U&keywords=state+estimation+for+robotics&qid=1639835747&sprefix=state+estimation+for+robotic%2Caps%2C501&sr=8-1)
* **Modern C++ (11 and above)** [Reference Book](https://www.amazon.com/Effective-Modern-Specific-Ways-Improve/dp/1491903996/ref=sr_1_1?crid=3FBSOVE5S7DG2&keywords=effective+modern+c%2B%2B&qid=1639809190&sprefix=effective+modern+c%2B%2Caps%2C573&sr=8-1)
* Python [Reference Book](https://www.amazon.com/Fluent-Python-Concise-Effective-Programming/dp/1491946008/ref=sr_1_2?crid=1N4TWSFVKTSUC&keywords=fluent+python+2nd+edition&qid=1639911762&sprefix=fluent+python%27%2Caps%2C332&sr=8-2)
* **ROS** [深蓝学院参考课程](https://www.shenlanxueyuan.com/course/364)
* **Git/GitHub, Markdown**与LaTeX [Coursera参考课程](https://www.coursera.org/professional-certificates/google-it-automation#courses)

---

## Docker Env - Up & Running

### WARNING - About Environment 

**若您坚持在Native Ubuntu上进行开发**, 此处**将默认您有足够的定位&建图开发环境运维经验, 能够自定义开发环境, 并且自主解决由于环境依赖冲突导致的各种问题**. 课程的重点在多传感融合定位算法, 并非基于Docker的环境搭建. 如果感兴趣, 请根据以下提示, 自行学习.**本框架只确保Docker环境的开箱即用, 不提供任何本地环境配置冲突解决的帮助**.

* 确保熟悉Docker与Docker-Compose的开发使用
* 根据Dockerfile [Click Here](docker/cpu.Dockerfile)自行准备开发环境
* 熟悉其中的各个依赖库, 有能力在出现依赖问题时, 自行搜索, 查询相关资料, 修复依赖冲突

### WARNING - On This Guide

**请您务必按照本指南, 一步一步操作, 当且仅当您在对本环境足够熟悉的情况下，再自由发挥**

### Overview

本Repo为基于**ROS melodic** @ **Ubuntu 18.04**的[多传感器融合定位与建图/Multi-Sensor Fusion for Localization & Mapping](https://www.shenlanxueyuan.com/course/324)学习环境. 

定位与建图是一个复杂的工程. **每一个解决方案, 都有较为复杂的环境依赖**. 然而:

* 部分依赖项由于网络原因(Great Fire Wall & Server IP Block)难以直接获得

* 由于课程依赖项的版本, 与本地现有依赖项的版本, 可能有所不同, 直接冒然安装, 可能会导致本地开发环境被破坏

故: 本课程的学习环境将以**Docker Image**的形式提供, 以实现与Native PC环境的隔离. 

* 本项目Native PC的操作系统选择**Ubuntu**. Windows与Mac不推荐使用, 若有意尝试, 请自行搜索相关解决方案.

---

### 安装Ubuntu

首先请确保您可以访问**Ubuntu**开发环境. 如果没有**Ubuntu**环境, 请按照[点击链接进入](ubuntu-setup/README.md)指南, 在本地PC上安装配置**Ubuntu**环境.

---

### 获取Docker开发环境

本课程推荐使用配套的Docker环境[点击链接进入](docker/README.md)完成课程学习. Docker提供了一个轻量级的标准化开发环境, 能够避免本地已有环境的差异导致的奇怪问题, 避免环境配置调试的巨额时间浪费.

---

### JFYI: Dependency Overview

* **C++**
  * **Google Protobuf**, [Click Here](https://github.com/google/protobuf.git), **branch 3.14.x**
  * **GeographicLib**, [Click Here](https://nchc.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.50.1.zip), **version 1.50.1**
  * **GeographicLib**, [Click Here](https://nchc.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.50.1.zip), **version 1.50.1**
  * **Sophus**, [Click Here](https://github.com/strasdat/Sophus.git), **commit id 49a7e1286910019f74fb4f0bb3e213c909f8e1b7**
  * **Google Ceres**, [Click Here](https://github.com/ceres-solver/ceres-solver.git), **commit id c2fab6502e5a341ff644c2bb2c5171ebd882b2d6**
  * **G2O**, [Click Here](https://github.com/RainerKuemmerle/g2o.git), **tag 20200410_git**
  * **GTSAM**, [Click Here](https://github.com/borglab/gtsam.git), **tag 4.0.3**

* **Python**
  * **kitti2bag**, [Click Here](https://pypi.org/project/kitti2bag/), **1.5**
  * **evo**, [Click Here](https://pypi.org/project/evo/), **1.12.0**
---

### Workspace

当Native PC与Course Docker Environment均准备就绪时, 即可开始使用本开发环境:

* **第一次使用时, 请首先下载课程配套的KITTI测试数据**[here](workspace/data/kitti/README.md).

* 之后, 即可通过**本地VSCode开发, Docker内部编译测试**的模式, 完成课程作业.

#### 获取课程数据

在第一次使用时, 需要将**课程配套的修复后KITTI数据**下载至本地文件系统. 具体操作方法参考[点击链接进入](workspace/data/kitti/README.md)

#### 安装Native Ubuntu IDE

本课程推荐使用[VSCode](https://code.visualstudio.com/)进行开发. 安装VSCode之后，推荐继续安装以下VSCode Plug-Ins:

* **C/C++** by **Microsoft**

* **Markdown All in One** by **Yu Zhang**

#### 开发, 编译与测试

启动Docker环境后, **Docker**中的/workspace目录, 会被映射到**当前Repo**中的workspace目录.

使用该Workspace进行开发, 编译与测试的方法如下:

* 在**当前Repo的workspace**下, 启动[VSCode](https://code.visualstudio.com/), 编辑源代码:

<img src="doc/development-environment.png" alt="Development Environment, Native VS Code in Mounting Point" width="100%">

* 在**Docker /workspace**下, 进行编译. 具体的编译方法, 请参考[作业1 环境搭建](workspace/assignments/01-introduction/README.md)

---

### Assignments

请各位执行如下的分支切换指令, 完成各周对应的作业.

#### Week 02, Lidar Odometry, Basic

```bash
git checkout 02-lidar-odometry-basic
```

#### Week 03, Lidar Odometry, Advanced

```bash
git checkout 03-lidar-odometry-advanced
```

#### Week 04, Mapping and Matching

```bash
git checkout 04-mapping-and-matching
```

#### Week 05, IMU Calibration

```bash
git checkout 05-imu-calib
```

#### Week 06, IMU Navigation

```bash
git checkout 06-imu-navigation
```

#### Week 07, Filtering, Basic

```bash
git checkout 07-filtering-basic
```

#### Week 08, Filtering Advanced

```bash
git checkout 08-filtering-advanced
```

#### Week 09, Graph Optimization

```bash
git checkout 09-graph-optimization
```

#### Week 10, Sliding Window

```bash
git checkout 10-sliding-window
```

#### Week 11, Capstone - Do It Own Your Own!

---

Keep Learning & Keep Coding

Ge Yao, alexgecontrol@qq.com