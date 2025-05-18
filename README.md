[ROS官方安装教程](https://wiki.ros.org/noetic/Installation/Ubuntu)

[MoveIt官方配置教程](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

[MoveIt Setup Assistant教程](https://moveit.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html)

下面，我们以Ubuntu20.04为例，[安装ROS1(Noetic)](#ros安装)并[配置MoveIt环境](#moveit配置和安装)进行机械臂的控制以及在Gazebo中完成仿真。
# ROS安装
## 一键安装
```bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

##  手动安装
1. 找到系统中的"**软件和更新**"程序，勾选"**restricted**," "**universe**," 还有 "**multiverse**" ，如下图所示：
![Update](./figures/image.png)

2. 配置`sources.list`，打开终端(Ctrl+Alt+t)
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

3. 配置访问Key
```bash
sudo apt install curl       # 确保curl已安装
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

4. 更新软件源并安装ROS
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full # 推荐全量安装
```

5. 环境变量配置(如果同时存在多个ROS系统，建议进行如下配置)
```bash
source /opt/ros/noetic/setup.bash   # 每新开一个终端都要进行配置，以配置不同的ROS系统
```
如果只存在一个ROS系统，可以直接将其添加到系统环境变量(之后无需配置临时环境变量)：
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

6. 为ROS安装package依赖
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

7. 安装并初始化rosdep(依赖管理工具)
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

至此，我们已经完成ROS的安装。接下来，我们需要完成MoveIt的配置：

# MoveIt配置和安装
1. 更新软件源
```bash
rosdep update
sudo apt update
sudo apt dist-upgrade
```

2. 安装构建工具
```bash
sudo apt install ros-noetic-catkin python3-catkin-tools
sudo apt install python3-wstool
```

3. 创建工作区
> 可以自行修改路径，这里我在`home`中创建了`workspace`总工作空间，并在其中创建`moveit`工作空间
```bash
mkdir -p ~/workspace/ws_moveit/src
cd ~/workspace/ws_moveit/src
```

4. 构建工作区
```bash
# 使用二进制包安装MoveIt（比从源码构建更快）
sudo apt install ros-noetic-moveit

# 安装MoveIt附加组件
sudo apt install ros-noetic-moveit-ros-visualization ros-noetic-moveit-planners ros-noetic-moveit-ros-move-group ros-noetic-moveit-ros-perception

# 安装依赖项
cd ~/workspace/ws_moveit/src

# 克隆教程和示例(不必，之后我们会使用URDF文件创建自己的package)
```
至此，我们已经完成基础环境的搭建。之后自行选择合适的机械臂完成配置：

# 导出机械臂相应pkg并安装依赖
这里推荐一个开源URDF文件数据集[awesome-robot-descriptions](https://github.com/robot-descriptions/awesome-robot-descriptions)

我们以`iiwa7机械臂`为例，完成作业要求

1. 下载urdf文件
```bash
cd 替换为实际路径/ws_moveit/src
git clone https://github.com/facebookresearch/differentiable-robot-model.git
```
2. 使用`MoveIt Setup Assistant`导出机械臂package
```bash
sudo apt install ros-noetic-franka-description      # 确保相关程序已安装
roslaunch moveit_setup_assistant setup_assistant.launch
```

- 成功运行后，如图所示点击`Create New MoveIt Configuration Package`：
![](./figures/msa_0.jpg)
- 点击`Browse`
![](./figures/msa_1.jpg)
- 找到我们的`iiwa7.urdf`文件，双击即可
![](./figures/msa_2.jpg)
- 成功加载后会显示机械臂模型
![](./figures/msa_3.jpg)

> 注意：如果在加载URDF文件时出现找不到mesh文件的错误，请参考[附录A：解决mesh文件路径问题](#附录a解决mesh文件路径问题)进行修复。

3. 配置机器人组件
   - 自碰撞矩阵配置：点击`Self-Collisions`，然后点击`Generate Collision Matrix`。
   
   ![](./figures/collision_matrix.jpg)
   
   - 虚拟关节配置：点击`Virtual Joints`，添加一个虚拟关节，将机器人固定在世界坐标系。
     - 名称：`virtual_joint`
     - 子链接：`iiwa_link_0`（机器人的基座链接）
     - 父链接：`world`
     - 类型：`fixed`
     
   ![](./figures/virtual_joint.jpg)
   
   - 规划组配置：点击`Planning Groups`，创建一个规划组
     - 名称：`manipulator`
     - 运动学求解器：`KDLKinematicsPlugin`
     - 组配置：包含所有关节（`iiwa_joint_1`到`iiwa_joint_7`）
     
   ![](./figures/planning_group_0.jpg)
   ![](./figures/planning_group_1.jpg)
   
   - 机器人姿态配置：点击`Robot Poses`，添加常用姿态
     - 名称：`home`（回原位姿态）
     - 设置关节角度为初始位置
     - 这里均设置为`0`
     
   ![](./figures/robot_poses.jpg)
   
   - 末端执行器配置：点击`End Effectors`，添加末端执行器
     - 名称：`end_effector`
     - 组：`manipulator`
     - 父链接：`iiwa_link_7`（机器人的末端链接）
     
   ![](./figures/end_effector.jpg)

   - 作者信息填写：点击`Author Information`，自行填写作者信息
     - 维护者名称
     - 维护者邮箱

   ![](./figures/author.png)

4. 生成配置文件
   - 点击`Configuration Files`
   - 填写配置包的基本信息
     - 名称：`iiwa7_moveit_config`
     - 路径：选择保存的目录，如`~/workspace/ws_moveit/src`
   - 点击`Generate Package`生成配置包
   
   ![](./figures/config_files.png)

5. 构建MoveIt配置包
```bash
cd ~/workspace/ws_moveit
catkin build
source devel/setup.bash
```

6. 启动MoveIt演示
```bash
roslaunch iiwa7_moveit_config demo.launch
```

这将启动RViz，显示iiwa7机械臂的模型，并可以通过MoveIt插件进行规划和控制。
![](./figures/rviz_0.png)

# 机械臂控制
我们已经完成机械臂的配置和导入，接下来，我们将实现机械臂的点位控制和轨迹控制，包括8字形轨迹、椭圆轨迹和*螺旋轨迹*（或其它控制性能，我觉得实现一个键盘操控+鼠标跟随也是可以的）。

## 创建控制包

首先，我们需要创建一个新的ROS包来实现机械臂控制功能：

```bash
# 进入工作空间的src目录
cd ~/workspace/ws_moveit/src

# 创建一个新的ROS包，依赖于MoveIt等相关包
catkin_create_pkg iiwa7_control moveit_ros_planning_interface roscpp rospy std_msgs geometry_msgs

# 创建脚本目录
cd iiwa7_control
mkdir -p scripts
```

接下来，我们可以参考[MoveIt Python Interface教程](https://moveit.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)完成任务。
官方教程十分完善，涉及到很多基础操作和模块。在本项目中，我们会继续指出跟随官方教程可能出现的问题并提出参考解决方案。

我们将参考官方教程示例代码实现具体代码，并将相关功能封装到类（控制器类:`PointControl`）

## 点位控制

点位控制是指控制机械臂的末端执行器按顺序运动到空间中的多个预定义点位。下面我们实现一个Python脚本，控制机械臂运动到6个不同的空间点位：

```bash
# 创建点位控制脚本
touch scripts/point_control.py
chmod +x scripts/point_control.py
```

在[`point_control.py`](./src/iiwa7_control/scripts/point_control.py)中实现以下功能：
### 1. 初始化并获取基本信息
- 初始化ROS节点和MoveIt Commander。
- 获取机器人基本信息，如规划坐标系、末端执行器名称。
- 创建用于在RViz中可视化MoveIt规划轨迹的Publisher。

### 2. 核心控制方法
- `go_to_joint_state(joint_goal_array)`: 控制机械臂各关节运动到指定的目标角度。
- `go_to_pose_goal(pose_goal)`: 控制机械臂末端执行器运动到空间中的目标位姿 (位置和姿态)。
- `plan_cartesian_path(waypoints)`: 根据给定的路径点列表（`geometry_msgs.msg.Pose`对象），规划笛卡尔空间下的直线运动路径。
- `execute_plan(plan)`: 执行先前规划好的机器人轨迹。

### 3. 交互式六边形轨迹控制 (基于当前末端姿态)
`point_control.py` 脚本经过增强，现在支持一个更高级和交互式的演示功能：在空间中绘制一个正六边形。其特点如下：

- **交互式初始位姿确定**:
    1. 脚本首先会控制机械臂移动到一个预定义的"准备"关节姿态。
    2. 然后程序会暂停，等待用户在RViz中观察并调整（如果需要，可以通过RViz的交互工具微调目标姿态，尽管脚本本身不直接处理这种外部调整）或仅仅是确认当前末端执行器的位姿。
    3. 用户按下回车键后，脚本会捕获末端执行器当前的完整世界位姿（包括位置和姿态/朝向）。

- **六边形平面与中心的定义**:
    - **中心**: 捕获到的末端执行器的世界位置将作为生成六边形的中心点。
    - **平面朝向**: 捕获到的末端执行器的世界姿态（朝向）将决定六边形所在的平面。这意味着六边形会绘制在末端执行器"前方"的、与其当前朝向对齐的平面上。如果末端执行器是水平的，六边形也是水平的；如果末端执行器是倾斜的，六边形也会相应倾斜。
    - **顶点计算**: 六边形的顶点首先在末端执行器的局部坐标系（XY平面，Z为0）中计算，然后通过捕获到的世界位姿变换到世界坐标系中。

- **固定姿态与闭合路径**:
    - **姿态保持**: 在绘制六边形的过程中，末端执行器会尝试保持其在初始捕获时刻的那个姿态。
    - **路径闭合**: 在经过六边形的6个顶点后，脚本会自动添加第7个路径点，该点与六边形的第1个顶点相同，从而使机械臂的运动轨迹形成一个闭合的六边形。

- **参数可调**:
    - 六边形的半径 (`radius`) 可以在脚本的 `run()` 方法中方便地修改。

### 4. 路径可视化增强：Marker显示
为了更清晰地展示期望的末端执行器路径（例如绘制的六边形），除了MoveIt默认的规划路径显示外，`point_control.py` 脚本还增加了以下功能：

- **发布Marker消息**: 脚本会创建一个 `visualization_msgs/Marker` 类型的消息 (具体为 `LINE_STRIP`)，其中包含六边形所有路径点的位置。
- **Marker话题**: 此Marker消息会发布到 `/eef_trajectory_marker` 话题。
- **RViz中配置**:
    1. 在RViz的左侧"Displays"面板中，点击"Add"按钮。
    2. 选择"Marker"显示类型（通常在 `rviz` 或 `visualization_msgs` 分类下）。
    3. 选中新添加的"Marker"Display，在其属性中找到"Marker Topic"一项。
    4. 将其值修改为 `/eef_trajectory_marker`。
    5. 完成后，您应该能在RViz中看到一条独立的、默认为红色的线状轨迹，精确地描绘出程序计算出的六边形路径点。

### 5. 运行控制
（运行控制的指令与下方"构建并运行"部分一致，确保先启动 `demo.launch`）

## 轨迹控制

轨迹控制是指控制机械臂的末端执行器按照特定轨迹运动，如8字形、椭圆和螺旋轨迹。以下是实现的代码：

```bash
# 创建轨迹控制脚本
touch scripts/trajectory_control.py
chmod +x scripts/trajectory_control.py
```

在`trajectory_control.py`中实现以下功能：


## 构建并运行

编译工作空间并运行控制脚本：

```bash
# 构建工作空间
cd ~/workspace/ws_moveit
catkin build
source devel/setup.bash

# 先启动MoveIt演示环境
roslaunch iiwa7_moveit_config demo.launch
```

在另一个终端中运行点位控制或轨迹控制：

```bash
# 点位控制
source ~/workspace/ws_moveit/devel/setup.bash
rosrun iiwa7_control point_control.py

# 或者轨迹控制
source ~/workspace/ws_moveit/devel/setup.bash
rosrun iiwa7_control trajectory_control.py
```

## 控制效果

1. 点位控制：机械臂的末端执行器会依次移动到6个预定义的空间点位，形成一个空间闭合路径。
   ![](./figures/point_control.png)

2. 8字形轨迹：机械臂的末端执行器会在垂直平面内画出一个8字形轨迹。
   ![](./figures/figure_eight.png)

3. 椭圆轨迹：机械臂的末端执行器会在垂直平面内画出一个椭圆轨迹。
   ![](./figures/ellipse.png)

4. 螺旋轨迹：机械臂的末端执行器会沿着一个螺旋路径前进，同时半径逐渐减小。
   ![](./figures/spiral.png)

## 代码分析

1. **初始化**：脚本首先初始化了MoveIt的各种组件，包括RobotCommander、PlanningSceneInterface和MoveGroupCommander，这些组件用于和MoveIt框架交互。同时设置规划时间限制为15秒，比默认的5秒要长，有助于复杂路径的规划。

2. **位姿控制**：使用`go_to_pose_goal`方法实现末端执行器位姿的控制，可以指定空间中的位置和姿态。

3. **笛卡尔路径规划**：使用`plan_cartesian_path`方法规划笛卡尔路径，通过指定一系列中间点位来生成轨迹。它接收三个参数：
   - `waypoints`：路径点列表
   - `eef_step`：末端执行器的步长（0.01m）
   - `jump_threshold`：跳跃阈值（0.0表示不限制）

4. **轨迹生成**：
   - 8字形轨迹：使用参数方程 y = scale * sin(θ), z = scale * sin(2θ) / 2 + offset
   - 椭圆轨迹：使用参数方程 y = a * cos(θ), z = b * sin(θ) + offset
   - 螺旋轨迹：使用参数方程 y = r * cos(θ), z = r * sin(θ) + offset, x = x₀ + height * t / steps

5. **轨迹执行**：使用`execute_plan`方法执行规划好的轨迹，实现机械臂的平滑运动。

使用这种基于MoveIt的轨迹规划和执行方法，可以实现机械臂的复杂运动，同时利用MoveIt的碰撞检测功能确保运动安全。

## 故障排除

如果在执行轨迹时遇到类似"Unable to sample any valid states for goal tree"的错误，可以尝试以下解决方法：

1. 减小轨迹的幅度（scale、radius等参数）
2. 减少路径点的数量（steps参数）
3. 调整起始位置，确保轨迹在机械臂的工作空间内
4. 增加规划时间限制（可通过设置move_group的planning_time参数）

请确保`compute_cartesian_path`方法包含必要的参数（路径点、步长、跳跃阈值和避免碰撞标志）。

### 笛卡尔路径规划失败或fraction为0（机械臂不动）

如果 `compute_cartesian_path` 返回的 `fraction` 很低（例如0.0）或者机械臂在尝试执行笛卡尔路径时没有移动，即使没有明显错误，也可能存在以下问题：

*   **起始姿态接近奇异点**：许多机械臂（包括iiwa7）在完全伸直或某些特定关节角度组合时会处于奇异状态。在奇异点附近，机械臂的雅可比矩阵会变得病态，导致逆运动学求解困难，从而使得笛卡尔空间下的直线运动规划失败。
    *   **解决方案**：在开始任何笛卡尔路径规划（如点位控制或复杂轨迹跟踪）之前，建议首先将机械臂通过**关节空间规划**移动到一个已知的、非奇异的"准备"姿态。例如，可以将所有关节移动到微小偏移的非零角度。
      ```python
      # 示例：移动到一个初始的非奇异关节姿态 (在您的控制脚本中)
      # joint_goal = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1] # 根据实际机器人调整
      # controller.go_to_joint_state(joint_goal) # 假设控制器类中有此方法
      # rospy.sleep(1) # 等待移动完成
      ```
      对于 `README.md` 中给出的 `point_control.py` 或 `trajectory_control.py` 示例，如果遇到此问题，可以在 `point_control_demo` 方法或各个轨迹演示方法（如 `figure_eight_demo`）的**开头**，先调用 `go_to_joint_state` (在`point_control.py`中已定义) 或直接使用 `move_group.go()` 移动到一个合适的初始关节角度，例如：
      ```python
      # 在演示方法开头添加：
      # print("正在移动到初始准备姿态...")
      # initial_joint_goal = [0.0, 0.2, 0.0, -1.5, 0.0, 1.7, 0.0] # 示例准备姿态，请根据您的机器人调整
      # self.move_group.go(initial_joint_goal, wait=True)
      # self.move_group.stop()
      # rospy.sleep(1)
      # print("已到达准备姿态。")
      ```
      确保这个初始移动本身是成功的。在您和我共同开发的 `point_to_point_control.py` 脚本中，我们正是通过 `go_to_initial_joint_state` 方法实现了这一点，从而确保了后续笛卡尔运动的成功。

*   **目标点不可达或路径上有障碍物**：确保您规划的目标点在机械臂的可达工作空间内，并且路径上没有未知的障碍物（如果启用了碰撞检测）。
*   **笛卡尔路径本身不可行**：即使目标点可达，但如果要求的直线路径导致机械臂关节超出限制或进入自碰撞，规划也会失败。尝试将路径分解为更小的分段，或允许通过中间点进行关节空间移动。
*   **规划时间不足**：对于复杂的笛卡尔路径，默认的规划时间可能不够。可以尝试增加规划时间（例如，在 `Iiwa7Controller` 的 `__init__` 方法中，或者在规划前临时设置）：
    ```python
    # self.move_group.set_planning_time(15.0) # 增加规划时间
    ```

# 附录
## 附录A：解决mesh文件路径问题

如果在加载URDF文件时出现以下错误：
```
Error retrieving file [meshes/iiwa7/collision/link_0.stl]: Could not resolve host: meshes
```

这是因为URDF文件中的mesh文件路径不正确。需要修改URDF文件中的mesh路径，添加`package://`前缀并指向正确的包路径：

```bash
# 找到URDF文件
find ~/workspace/ws_moveit/src -name "iiwa7.urdf"

# 编辑URDF文件，将所有mesh路径从类似
# <mesh filename="meshes/iiwa7/visual/link_0.stl"/>
# 修改为
# <mesh filename="package://differentiable-robot-model/diff_robot_data/kuka_iiwa/meshes/iiwa7/visual/link_0.stl"/>
```

此外，还需要将differentiable-robot-model目录配置为ROS包，以便ROS能够找到这些文件：

```bash
# 创建package.xml文件
cd ~/workspace/ws_moveit/src/differentiable-robot-model
cat > package.xml << EOL
<?xml version="1.0"?>
<package format="2">
  <name>differentiable-robot-model</name>
  <version>0.1.0</version>
  <description>Differentiable Robot Model Package</description>

  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  
  <export>
  </export>
</package>
EOL
```

```bash
# 创建CMakeLists.txt文件
cat > CMakeLists.txt << EOL
cmake_minimum_required(VERSION 3.0.2)
project(differentiable-robot-model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  \${catkin_INCLUDE_DIRS}
)

## Mark mesh files for installation
install(DIRECTORY diff_robot_data
  DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.stl" PATTERN "*.urdf" PATTERN "*.xacro"
)
EOL
```

```bash
# 重新构建工作空间
cd ~/workspace/ws_moveit
catkin build
source devel/setup.bash
```

修改后再次运行Setup Assistant：
```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

修复后应该可以成功加载机械臂模型。

## 附录B：具体代码
1. `point_control.py`
   
   我们项目中实际使用的 `point_control.py` 脚本 (`ws_moveit/src/iiwa7_control/scripts/point_control.py`) 已经包含了上述交互式六边形绘制、Marker可视化等高级功能，其功能比下面这个基础版本更丰富。建议直接参考和运行项目中的实际脚本。
   
   以下是一个非常基础的点位控制脚本结构示例，主要用于展示核心API的调用方式。项目中的实际脚本在此基础上进行了大量扩展。

```python
#!/usr/bin/env python3
# coding: utf-8

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_matrix, quaternion_from_euler
from visualization_msgs.msg import Marker


class PointControl:
    def __init__(self):
        """初始化Iiwa7PointToPointControl节点和MoveIt! Commander。

        此构造函数负责设置ROS节点、初始化MoveIt!的Python接口
        (RobotCommander, PlanningSceneInterface, MoveGroupCommander)，
        并创建一个用于在RViz中可视化轨迹的发布者。
        同时，它会获取并打印机器人的基本信息，如规划坐标系、
        末端执行器名称和可用的规划组。
        """
        # 初始化moveit_commander和rospy节点
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('iiwa7_point_to_point_control', anonymous=True)

        # 实例化一个RobotCommander对象，提供有关机器人运动学模型和机器人当前关节状态的信息
        self.robot = moveit_commander.RobotCommander()

        # 实例化一个PlanningSceneInterface对象，这提供了一个远程接口，用于获取、设置和更新机器人对周围世界的内部理解
        self.scene = moveit_commander.PlanningSceneInterface()

        # 实例化一个MoveGroupCommander对象，该对象是一个运动规划组（一组关节）的接口。注意我们之前在MoveIt Setup Assistant中配置的"manipulator"组，参数名就是"manipulator"
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")

        # 创建一个用于发布轨迹的DisplayTrajectory发布者，用于在 Rviz 中显示轨迹
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',     # 话题名称
            moveit_msgs.msg.DisplayTrajectory,      # 消息类型
            queue_size=20)                          # 队列大小

        # Publisher for end-effector path markers
        self.eef_path_marker_publisher = rospy.Publisher(
            '/eef_trajectory_marker', # Topic name for the markers
            Marker,
            queue_size=10)

        # 获取规划参考坐标系的名称
        self.planning_frame = self.move_group.get_planning_frame()
        # 获取末端执行器链接的名称
        self.eef_link = self.move_group.get_end_effector_link()
        # 获取机器人组的名称
        self.group_names = self.robot.get_group_names()

        rospy.loginfo("============ 机器人参考坐标系: %s" % self.planning_frame)
        rospy.loginfo("============ 末端执行器: %s" % self.eef_link)
        rospy.loginfo("============ 机器人组: %s" % self.group_names)
        # 启动时打印一次初始状态,一般用于调试
        initial_state = self.move_group.get_current_state()
        rospy.loginfo("============ 初始当前状态: %s" % initial_state)

    def move_J(self, joint_goal_array):
        """控制机械臂移动到指定的目标关节角度。

        此方法首先验证输入的目标关节角度数量是否与机器人匹配，
        然后设置关节目标，规划路径，最后执行规划的轨迹。
        它会明确区分规划阶段和执行阶段的成功与否，并返回相应的布尔值。

        Args:
            joint_goal_array (list of float): 一个包含目标关节角度的列表（弧度）。
                列表的长度必须与机械臂的关节数量一致。

        Returns:
            bool: 如果规划和执行都成功，则返回True；否则返回False。
        """
        current_joints = self.move_group.get_current_joint_values()
        if len(joint_goal_array) != len(current_joints):    # 判断给定关节角度是否合理（关节数目）
            rospy.logerr(f"错误: 提供的目标关节数 {len(joint_goal_array)} 与实际关节数 {len(current_joints)} 不符")
            return False

        rospy.loginfo(f"当前关节角度: {current_joints}")
        rospy.loginfo(f"目标关节角度: {joint_goal_array}")

        self.move_group.set_joint_value_target(joint_goal_array)
        

        # 直接使用Go函数,但是报错信息不完整，一旦遇到错误，很难定位
        # success = self.move_group.go(joint_goal_array, wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()
        # return success

        # 分别规划和执行
        plan_success, plan, planning_time, error_code = self.move_group.plan()
        if not plan_success:
            rospy.logerr(f"关节目标规划失败! 错误码: {error_code}")
            return False
            
        rospy.loginfo(f"关节目标规划成功，开始执行...")
        execute_success = self.move_group.execute(plan, wait=True)
        
        # 确保不出现冗余动作
        self.move_group.stop()

        if not execute_success:
            rospy.logerr("关节目标执行失败!")
            return False
        
        rospy.loginfo("关节目标执行成功。")
        return True

    def go_to(self, pose_goal):
        """控制机械臂的末端执行器移动到指定的目标位姿。

        此方法使用MoveGroupCommander的 `go()` 方法，该方法会尝试规划并执行
        到目标位姿的运动。执行后会清除目标位姿。

        Args:
            pose_goal (geometry_msgs.msg.Pose or geometry_msgs.msg.PoseStamped):
                期望的末端执行器目标位姿。

        Returns:
            bool: 如果规划和执行成功，则返回True；否则返回False。
        """
        rospy.loginfo(f"尝试移动到目标位姿: P({pose_goal.position.x:.3f}, {pose_goal.position.y:.3f}, {pose_goal.position.z:.3f}), "
                      f"Q({pose_goal.orientation.x:.3f}, {pose_goal.orientation.y:.3f}, {pose_goal.orientation.z:.3f}, {pose_goal.orientation.w:.3f})")
        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        # 官方教程建议在实现规划后清除目标
        self.move_group.clear_pose_targets()

        if success:
            rospy.loginfo("移动到目标位姿成功。")
        else:
            rospy.logerr("移动到目标位姿失败。")
        return success

    def plan_cartesian_path(self, waypoints):
        """规划笛卡尔路径，使末端执行器通过一系列指定的空间点。

        此方法尝试计算一个笛卡尔空间（直线运动）的轨迹，该轨迹连接
        提供的路径点。它会返回计算出的轨迹以及规划成功的路径比例。
        在规划前，会降低最大速度和加速度以提高规划成功率和运动平滑度。

        Args:
            waypoints (list of geometry_msgs.msg.Pose): 一个包含一系列
                `geometry_msgs.msg.Pose` 对象的列表，定义了期望的路径点。
                机械臂的末端执行器将尝试按顺序通过这些位姿。

        Returns:
            tuple: 一个包含两个元素的元组:
                - plan (moveit_msgs.msg.RobotTrajectory): 计算出的机器人轨迹。
                  如果规划失败或部分成功，这可能是一个空的或部分的轨迹。
                - fraction (float): 一个介于0.0和1.0之间的浮点数，表示
                  成功规划的路径段的比例。例如，0.9表示90%的请求路径
                  已成功规划。值越接近1.0越好。
        """
        # 设置较低的最大速度和加速度缩放因子，以提高规划成功率和运动平滑度
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

        waypoints_list = []
        waypoints_list = copy.deepcopy(waypoints)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints_list,  # 路径点列表 (list of Pose)
            0.01,            # eef_step (float) - 末端执行器步长，用于控制路径的平滑度
        )
        rospy.loginfo(f"笛卡尔路径规划完成，成功比例: {fraction:.2f}")
        return plan, fraction

    def execute_plan(self, plan):
        """执行一个先前规划好的机器人轨迹。

        Args:
            plan (moveit_msgs.msg.RobotTrajectory): 要执行的机器人轨迹，
                通常由 `plan_cartesian_path` 或 `move_group.plan()` 返回。

        Returns:
            bool: 如果轨迹执行成功，则返回True；否则返回False。
        """
        rospy.loginfo("开始执行规划的轨迹...")
        success = self.move_group.execute(plan, wait=True)
        if success:
            rospy.loginfo("轨迹执行成功。")
        else:
            rospy.logerr("轨迹执行失败。")
        return success

    def display_eef_path_as_marker(self, waypoints_pose_list):
        """在RViz中将末端执行器路径显示为LINE_STRIP Marker."""
        if not waypoints_pose_list:
            rospy.logwarn("无法显示空的路径点列表作为Marker。")
            return

        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "eef_hexagon_path"
        marker.id = 0 # Unique ID for this marker
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Marker的姿态 (对于LINE_STRIP，通常设为单位姿态)
        marker.pose.orientation.w = 1.0

        # 线条的宽度
        marker.scale.x = 0.01  # 例如1cm宽

        # 线条的颜色 (红色, RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0 # 不透明

        # Marker的生命周期 (rospy.Duration() 表示永久)
        marker.lifetime = rospy.Duration()

        # 填充路径点
        for pose_stamped_or_pose in waypoints_pose_list:
            # waypoints_pose_list包含的是Pose对象
            p = Point()
            p.x = pose_stamped_or_pose.position.x
            p.y = pose_stamped_or_pose.position.y
            p.z = pose_stamped_or_pose.position.z
            marker.points.append(p)
        
        rospy.loginfo(f"发布包含 {len(marker.points)} 个点的 EEF 路径 Marker 到 /eef_trajectory_marker")
        self.eef_path_marker_publisher.publish(marker)

    def run_demo(self):
        """
        <测试用>

        运行一个简单的空间点位控制演示
        """
        # 1. 移动到一个已知的非奇异关节姿态 ("ready" pose)
        rospy.loginfo("============ 移动到初始准备关节姿态 ============")
        # KUKA LBR iiwa R800/R820 的一个常用准备姿态 (弧度)，
        # [A1, A2, A3, A4, A5, A6, A7]
        # 我们可以先在Rviz可视化界面手动调整一个合适的姿态，将关节角度值记录下来
        ready_joint_angles = [0.0, np.deg2rad(20), 0.0, np.deg2rad(-70), 0.0, np.deg2rad(90), 0.0]

        
        
        if not self.move_J(ready_joint_angles):
            rospy.logerr("移动到准备姿态失败，演示中止。")
            return
        # 中断，观察可视化界面，直到用户回车
        rospy.loginfo("成功移动到准备姿态。请在Rviz可视化界面观察机械臂姿态，并按回车键继续...")
        input()
        
        rospy.sleep(1.0) # 等待机械臂稳定

        # 2. 获取新的当前姿态
        try:
            current_pose = self.move_group.get_current_pose().pose
            rospy.loginfo("============ 新的当前姿态 (关节移动后): %s" % current_pose)
        except Exception as e:
            rospy.logerr("获取关节移动后的姿态失败: %s，演示中止。" %e)
            return

        # 3. 定义基于新姿态的简单路径点
        waypoints = []
        scale = 0.1 # 移动幅度 (米)

        # 点1: 从当前位置沿X轴（基坐标系）正向移动
        target_pose_1 = copy.deepcopy(current_pose)
        target_pose_1.position.x += scale
        waypoints.append(copy.deepcopy(target_pose_1))

        # 点2: 从点1位置沿Y轴（基坐标系）正向移动
        target_pose_2 = copy.deepcopy(target_pose_1)
        target_pose_2.position.y += scale
        waypoints.append(copy.deepcopy(target_pose_2))
        
        # 点3: 从点2位置沿Z轴（基坐标系）向上移动
        target_pose_3 = copy.deepcopy(target_pose_2)
        target_pose_3.position.z += scale
        waypoints.append(copy.deepcopy(target_pose_3))


        rospy.loginfo("============ 生成的路径点 ============")
        for i, p in enumerate(waypoints):
            rospy.loginfo(f"路径点 {i+1}: P({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f}), "
                  f"Q({p.orientation.x:.3f}, {p.orientation.y:.3f}, {p.orientation.z:.3f}, {p.orientation.w:.3f})")

        # 在这里也显示demo的路径点
        self.display_eef_path_as_marker(waypoints)

        # 4. 规划笛卡尔路径
        rospy.loginfo("============ 规划笛卡尔路径 ============")
        plan, fraction = self.plan_cartesian_path(waypoints)
        rospy.loginfo("规划成功比例: %.2f" % fraction)

        if fraction < 0.9: # 如果路径规划不完整
            rospy.logwarn("警告: 路径规划不完整 (成功比例 < 0.9)。机械臂可能不会按预期移动。")
            # 可以选择在此处中止，或者尝试执行部分路径
            # return 

        # 5. 可视化轨迹 (可选, 但推荐)
        rospy.loginfo("============ 发布轨迹用于Rviz可视化 ============")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(2.0) # 给Rviz一点时间来显示轨迹

        # 6. 执行轨迹
        rospy.loginfo("============ 正在执行轨迹 ============")
        if fraction > 0.01: # 只在规划到一些路径时才执行
            success = self.execute_plan(plan)
            rospy.loginfo("轨迹执行 " + ("成功" if success else "失败"))
        else:
            rospy.loginfo("规划成功比例过低，不执行轨迹。")
        
    def run(self):
        """
        根据用户提供的点列表执行空间点位控制
        """
        # (与run_demo类似，首先确保机器人不在奇异位姿)
        rospy.loginfo("============ 移动到初始准备关节姿态 (自定义点位前) ============")
        ready_joint_angles = [0.0, np.deg2rad(20), 0.0, np.deg2rad(-70), 0.0, np.deg2rad(90), 0.0]
        if not self.move_J(ready_joint_angles):
            rospy.logerr("移动到准备姿态失败 (自定义点位)，中止。")
            return

        rospy.loginfo("成功移动到准备姿态。请在Rviz可视化界面观察机械臂姿态，并按回车键继续...")
        input()

        # 获取当前姿态作为六边形的中心参考
        hexagon_center_pose_world = None
        try:
            current_pose_msg = self.move_group.get_current_pose() # 获取PoseStamped
            hexagon_center_pose_world = current_pose_msg.pose    # 提取Pose
            rospy.loginfo(f"当前末端位姿 (将作为六边形参考): "
                          f"P({hexagon_center_pose_world.position.x:.3f}, "
                          f"{hexagon_center_pose_world.position.y:.3f}, "
                          f"{hexagon_center_pose_world.position.z:.3f}), "
                          f"Q({hexagon_center_pose_world.orientation.x:.3f}, "
                          f"{hexagon_center_pose_world.orientation.y:.3f}, "
                          f"{hexagon_center_pose_world.orientation.z:.3f}, "
                          f"{hexagon_center_pose_world.orientation.w:.3f})")
        except Exception as e:
            rospy.logerr(f"获取当前姿态失败: {e}，演示中止。")
            return

        # 定义六边形参数 (半径)
        radius = 0.15    # 六边形半径 (米)，也等于边长。
        
        # 末端执行器在每个顶点的姿态将与初始获取的姿态一致
        eef_orientation_quat_msg = hexagon_center_pose_world.orientation
        eef_orientation_list = [eef_orientation_quat_msg.x, eef_orientation_quat_msg.y, 
                                eef_orientation_quat_msg.z, eef_orientation_quat_msg.w]

        # 构建从局部坐标系到世界坐标系的变换矩阵
        # 旋转部分
        T_world_from_local = quaternion_matrix(eef_orientation_list)
        # 平移部分 (六边形中心在世界坐标系的位置)
        T_world_from_local[0,3] = hexagon_center_pose_world.position.x
        T_world_from_local[1,3] = hexagon_center_pose_world.position.y
        T_world_from_local[2,3] = hexagon_center_pose_world.position.z

        points_data_list = [] # Renamed from 'points' to avoid confusion with geometry_msgs.msg.Point
        num_vertices = 6
        for i in range(num_vertices):
            angle = (tau / num_vertices) * i  # tau 是 2*pi，角度从0开始

            # 在局部XY平面计算顶点 (六边形中心在局部坐标系原点)
            x_local = radius * np.cos(angle)
            y_local = radius * np.sin(angle)
            z_local = 0.0 # 六边形位于末端执行器的局部XY平面

            p_local_homogeneous = np.array([x_local, y_local, z_local, 1.0])
            
            # 转换到世界坐标系
            p_world_homogeneous = T_world_from_local @ p_local_homogeneous
            
            vertex_world_x = p_world_homogeneous[0]
            vertex_world_y = p_world_homogeneous[1]
            vertex_world_z = p_world_homogeneous[2]
            
            points_data_list.append([vertex_world_x, vertex_world_y, vertex_world_z] + eef_orientation_list)

        # 打印生成的六边形顶点，方便调试
        rospy.loginfo("============ 生成的六边形顶点数据 ============")
        for i, p_data in enumerate(points_data_list):
            rospy.loginfo(f"顶点 {i+1}: P({p_data[0]:.3f}, {p_data[1]:.3f}, {p_data[2]:.3f}), "
                          f"Q({p_data[3]:.3f}, {p_data[4]:.3f}, {p_data[5]:.3f}, {p_data[6]:.3f})")

        waypoints = []
        for point_data in points_data_list:
            pose = Pose()
            pose.position.x = point_data[0]
            pose.position.y = point_data[1]
            pose.position.z = point_data[2]
            pose.orientation.x = point_data[3]
            pose.orientation.y = point_data[4]
            pose.orientation.z = point_data[5]
            pose.orientation.w = point_data[6]
            waypoints.append(copy.deepcopy(pose))

        # 回到初始点points[0]
        pose = Pose()
        point_data = points_data_list[0]
        pose.position.x = point_data[0]
        pose.position.y = point_data[1]
        pose.position.z = point_data[2]
        pose.orientation.x = point_data[3]
        pose.orientation.y = point_data[4]
        pose.orientation.z = point_data[5]
        pose.orientation.w = point_data[6]
        waypoints.append(copy.deepcopy(pose))
        
        # 在这里显示自定义路径的Marker
        self.display_eef_path_as_marker(waypoints) # Call the new method

        rospy.loginfo("============ 规划自定义笛卡尔路径 ============")
        plan, fraction = self.plan_cartesian_path(waypoints)
        rospy.loginfo("规划成功比例: %.2f" % fraction)

        if fraction < 0.9:
            rospy.logwarn("警告: 自定义路径规划不完整 (成功比例 < 0.9)。")

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(2.0)
        
        rospy.loginfo("============ 正在执行自定义轨迹 ============")
        if fraction > 0.01:
            success = self.execute_plan(plan)
            rospy.loginfo("自定义轨迹执行 " + ("成功" if success else "失败"))
        else:
            rospy.loginfo("自定义轨迹规划成功比例过低，不执行。")


def main():
    try:
        controller = PointControl()
        
        # 运行演示
        # controller.run_demo()
        
        rospy.loginfo("初始化完成，控制器已就绪，按回车键继续...")
        input()
        
        # 自定义点位控制
        rospy.loginfo("\n============ 开始自定义点位控制测试 ============")
        controller.run()

        rospy.loginfo("============ 点位控制演示完成 ============")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
```