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

        # Marker发布器，用于在Rviz中显示机械臂的轨迹
        self.marker_publisher = rospy.Publisher(
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

    def clear_markers(self, ns="trajectory"):
        """清除指定命名空间下的所有RViz Markers。"""
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0 # 对于DELETEALL，id通常设为0，它会清除整个命名空间
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(marker)
        rospy.loginfo(f"已发送清除 Marker 命令到命名空间: {ns}")

    def show_path(self, waypoints_pose_list, ns="trajectory", r=1.0, g=0.0, b=0.0):
        """在RViz中将路径显示为LINE_STRIP Marker。
        在添加新Marker之前，会先清除该命名空间下的旧Markers。
        """
        # 1. 清除此命名空间中的旧Markers
        self.clear_markers(ns)
        # rospy.sleep(0.05) # 可选：短暂延时，确保RViz有时间处理删除命令

        # 2. 创建并发布新的ADD Marker
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now() # 可以使用与clear_marker相同的时间戳，或新的时间戳
        marker.ns = ns
        marker.id = 0 # 新的marker也使用id 0 (或者其他ID)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015 # 线条宽度
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.8 # 不透明度
        marker.lifetime = rospy.Duration()

        for pose in waypoints_pose_list:
            marker.points.append(pose.position)
        
        rospy.loginfo(f"发布包含 {len(marker.points)} 个点的轨迹 Marker 到 /eef_trajectory_marker (ns: {ns})")
        self.marker_publisher.publish(marker)

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
        self.show_path(waypoints)

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
        center = None
        current_pose_msg = self.move_group.get_current_pose()
        center = current_pose_msg.pose
        rospy.loginfo(f"当前末端位姿 (将作为六边形参考): "
                        f"P({center.position.x:.3f}, "
                        f"{center.position.y:.3f}, "
                        f"{center.position.z:.3f}), "
                        f"Q({center.orientation.x:.3f}, "
                        f"{center.orientation.y:.3f}, "
                        f"{center.orientation.z:.3f}, "
                        f"{center.orientation.w:.3f})")

        # 定义六边形参数 (半径)
        radius = 0.15    # 六边形半径 (米)，也等于边长。
        
        # 末端执行器在每个顶点的姿态将与初始获取的姿态一致
        eef_msg = center.orientation
        eef_orientation = [eef_msg.x, eef_msg.y, 
                                eef_msg.z, eef_msg.w]

        # 构建从局部坐标系到世界坐标系的变换矩阵
        # 旋转部分
        local2world = quaternion_matrix(eef_orientation)
        # 平移部分 (六边形中心在世界坐标系的位置)
        local2world[0,3] = center.position.x
        local2world[1,3] = center.position.y
        local2world[2,3] = center.position.z

        points_data = []
        num_vertices = 6
        for i in range(num_vertices):
            angle = (tau / num_vertices) * i  # tau 是 2*pi，角度从0开始

            # 在局部XY平面计算顶点 (六边形中心在局部坐标系原点)
            x_local = radius * np.cos(angle)
            y_local = radius * np.sin(angle)
            z_local = 0.0 # 六边形位于末端执行器的局部XY平面

            p_local = np.array([x_local, y_local, z_local, 1.0])
            
            # 转换到世界坐标系
            p_world = local2world @ p_local
            
            vertex_world_x = p_world[0]
            vertex_world_y = p_world[1]
            vertex_world_z = p_world[2]
            
            points_data.append([vertex_world_x, vertex_world_y, vertex_world_z] + eef_orientation)

        # 打印生成的六边形顶点，用于调试
        rospy.loginfo("============ 生成的六边形顶点数据 ============")
        for i, p_data in enumerate(points_data):
            rospy.loginfo(f"顶点 {i+1}: P({p_data[0]:.3f}, {p_data[1]:.3f}, {p_data[2]:.3f}), "
                          f"Q({p_data[3]:.3f}, {p_data[4]:.3f}, {p_data[5]:.3f}, {p_data[6]:.3f})")

        waypoints = []
        for point_data in points_data:
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
        point_data = points_data[0]
        pose.position.x = point_data[0]
        pose.position.y = point_data[1]
        pose.position.z = point_data[2]
        pose.orientation.x = point_data[3]
        pose.orientation.y = point_data[4]
        pose.orientation.z = point_data[5]
        pose.orientation.w = point_data[6]
        waypoints.append(copy.deepcopy(pose))
        
        # 在这里显示自定义路径的Marker
        self.show_path(waypoints)

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
