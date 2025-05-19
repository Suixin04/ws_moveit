#!/usr/bin/env python3
# coding: utf-8

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, sin, cos
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

class TrajectoryControl:
    def __init__(self):
        """初始化节点和MoveIt! Commander。"""
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('iiwa7_trajectory_control', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        # 增加规划时间，默认为5s，对于复杂轨迹可能不够
        self.move_group.set_planning_time(15.0)


        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)

        self.marker_publisher = rospy.Publisher(
            '/eef_trajectory_marker',
            Marker,
            queue_size=10)

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        rospy.loginfo("============ 机器人参考坐标系: %s" % self.planning_frame)
        rospy.loginfo("============ 末端执行器: %s" % self.eef_link)
        rospy.loginfo("============ 机器人组: %s" % self.group_names)
        
        # 获取当前的末端执行器位姿，作为轨迹的参考原点
        self.initial_pose = self.move_group.get_current_pose().pose
        rospy.loginfo(f"============ 初始末端位姿: {self.initial_pose}")


    def move_J(self, joint_goal_array):
        """控制机械臂各关节运动到指定的目标角度。"""
        current_joints = self.move_group.get_current_joint_values()
        if len(joint_goal_array) != len(current_joints):
            rospy.logerr(f"错误: 提供的目标关节数 {len(joint_goal_array)} 与实际关节数 {len(current_joints)} 不符")
            return False

        self.move_group.set_joint_value_target(joint_goal_array)
        plan_success, plan, _, _ = self.move_group.plan()
        if not plan_success:
            rospy.logerr("关节目标规划失败!")
            return False
        
        execute_success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        if not execute_success:
            rospy.logerr("关节目标执行失败!")
            return False
        rospy.loginfo("关节目标执行成功。")
        return True

    def plan_cartesian_path(self, waypoints):
        """规划笛卡尔路径。"""
        self.move_group.set_max_velocity_scaling_factor(0.1) # 降低速度和加速度以提高规划成功率
        self.move_group.set_max_acceleration_scaling_factor(0.1)

        plan = None
        fraction = 0.0
        try:
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # 路径点列表
                0.01       # eef_step，末端执行器步长
            )
            rospy.loginfo(f"笛卡尔路径规划完成，成功比例: {fraction:.2f}")
        except Exception as e:
            rospy.logerr(f"笛卡尔路径规划中发生异常: {e}")
            # plan will be None, fraction will be 0.0 as initialized
        return plan, fraction

    def execute_plan(self, plan):
        """执行规划好的轨迹。"""
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

    def fig_8_gen(self, scale=0.3, steps=100, center_pose=None):
        """
        生成8字形轨迹的路径点。
        轨迹在以center_pose为中心的、垂直于当前末端X轴的平面上 (即在末端的局部YZ平面上画)。
        Args:
            scale (float): 8字形的大小。
            steps (int): 轨迹点的数量。
            center_pose (geometry_msgs.msg.Pose): 轨迹的中心位姿。如果为None，则使用初始化时的末端位姿。
        Returns:
            list of geometry_msgs.msg.Pose: 路径点列表。
        """
        waypoints = []
        if center_pose is None:
            center_pose = self.initial_pose
        
        # 使用初始姿态的X值作为固定X，Y和Z值作为偏移中心
        # 轨迹的朝向将保持与center_pose的朝向一致
        base_x = center_pose.position.x
        offset_y = center_pose.position.y
        offset_z = center_pose.position.z
        fixed_orientation = center_pose.orientation

        for i in range(steps + 1): # +1 以闭合轨迹
            theta = (float(i) / steps) * 2 * tau # 0 to 2*tau for full figure-eight (two loops)
            
            # 在末端执行器的局部坐标系中计算y, z
            # 8字形通常在YZ平面绘制
            # y_local = scale * sin(theta)
            # z_local = scale * sin(theta) * cos(theta) # (sin(2*theta)/2)
            # 为了更像一个标准的8字形，参数方程可以是:
            # y_local = scale * sin(theta)
            # z_local = scale * sin(2*theta) / 2.0 

            # README 中的公式 y = scale * sin(θ), z = scale * sin(2θ) / 2 + offset
            # 我们这里theta从0到2tau (即4pi)，所以需要调整
            t = (float(i) / steps) * 2 * pi

            y_local = scale * sin(t) 
            z_local = scale * sin(2*t) / 2.0

            # 创建一个相对于中心点的局部Pose
            current_pose = Pose()
            current_pose.position.x = 0 # X在局部坐标系为0
            current_pose.position.y = y_local
            current_pose.position.z = z_local
            current_pose.orientation = Quaternion(0,0,0,1) # 局部姿态，先不旋转

            # 将局部点位变换到世界坐标系 (基于center_pose)
            # 简单起见，这里我们直接在世界坐标系下基于center_pose进行偏移
            # 假设center_pose定义了轨迹的基准平面和位置
            # TODO: 更鲁棒的做法是使用tf变换
            
            # 我们假设8字形绘制在以center_pose为原点, 保持其原有朝向的平面上
            # 8字形的 Y 和 Z 变化是相对于 center_pose 的局部 YZ 轴的
            # 但由于我们直接在世界坐标系下操作，需要一种方式将局部 YZ 偏移转换到世界坐标系下的偏移
            # 为了简单，我们先假设轨迹是在一个固定的世界平面上绘制，例如平行于全局YZ平面
            # X = base_x (固定)
            # Y = offset_y + y_local
            # Z = offset_z + z_local

            # 更准确的：将 (0, y_local, z_local) 从 center_pose 的局部坐标系转换到世界坐标系
            # T_world_from_local = quaternion_matrix([center_pose.orientation.x, center_pose.orientation.y, center_pose.orientation.z, center_pose.orientation.w])
            # T_world_from_local[0,3] = center_pose.position.x
            # T_world_from_local[1,3] = center_pose.position.y
            # T_world_from_local[2,3] = center_pose.position.z
            # local_point_homogeneous = np.array([0, y_local, z_local, 1.0])
            # world_point_homogeneous = np.dot(T_world_from_local, local_point_homogeneous)
            
            # 暂时简化：假设轨迹平面与机器人基座的YZ平面大致平行，直接在世界坐标系中加减
            pose = Pose()
            pose.position.x = base_x # X固定
            pose.position.y = offset_y + y_local
            pose.position.z = offset_z + z_local
            pose.orientation = copy.deepcopy(fixed_orientation) # 保持初始姿态
            waypoints.append(pose)
        return waypoints

    def ellipse_gen(self, radius_a=0.3, radius_b=0.2, steps=100, center_pose=None):
        """
        生成椭圆轨迹的路径点。
        轨迹在以center_pose为中心的、垂直于当前末端X轴的平面上。
        Args:
            radius_a (float): 椭圆在Y轴方向的半长轴。
            radius_b (float): 椭圆在Z轴方向的半短轴。
            steps (int): 轨迹点的数量。
            center_pose (geometry_msgs.msg.Pose): 轨迹的中心位姿。如果为None，则使用初始化时的末端位姿。
        Returns:
            list of geometry_msgs.msg.Pose: 路径点列表。
        """
        waypoints = []
        if center_pose is None:
            center_pose = self.initial_pose

        base_x = center_pose.position.x
        offset_y = center_pose.position.y
        offset_z = center_pose.position.z
        fixed_orientation = center_pose.orientation

        for i in range(steps + 1): # +1 以闭合轨迹
            theta = (float(i) / steps) * 2 * pi # 0 to 2*pi

            # README 中的公式 y = a * cos(θ), z = b * sin(θ) + offset
            y_local = radius_a * cos(theta)
            z_local = radius_b * sin(theta)
            
            # 简化处理，同8字形轨迹
            pose = Pose()
            pose.position.x = base_x # X固定
            pose.position.y = offset_y + y_local
            pose.position.z = offset_z + z_local
            pose.orientation = copy.deepcopy(fixed_orientation) # 保持初始姿态
            waypoints.append(pose)
        return waypoints

    def run(self):
        rospy.loginfo("============ 初始化轨迹控制演示 ============")
        
        # 0. 移动到一个已知的非奇异"准备"关节姿态
        #    避免从奇异姿态开始规划笛卡尔路径
        rospy.loginfo("--> 移动到准备姿态...")
        # 一个常用的iiwa准备姿态 (弧度)
        ready_joint_angles = [0.0, np.deg2rad(-15), 0.0, np.deg2rad(-90), 0.0, np.deg2rad(90), 0.0]
        if not self.move_J(ready_joint_angles):
            rospy.logerr("移动到准备姿态失败 (自定义点位)，中止。")
            return
        rospy.loginfo("准备姿态完成。")
        
        # 更新轨迹参考中心为当前"准备"姿态后的末端位姿
        current_eef_pose = self.move_group.get_current_pose().pose
        rospy.loginfo(f"当前末端位姿已更新为轨迹中心: {current_eef_pose}")

        # 1. 8字形轨迹演示
        rospy.loginfo("============ 开始8字形轨迹演示 ============")
        figure_eight_waypoints = self.fig_8_gen(scale=0.15, steps=150, center_pose=current_eef_pose)
        if not figure_eight_waypoints:
            rospy.logerr("生成8字形轨迹失败。")
            return

        self.show_path(figure_eight_waypoints, ns="figure_eight", r=1.0, g=0.5, b=0.0) # 橙色
        
        rospy.loginfo("规划8字形轨迹...")
        plan, fraction = self.plan_cartesian_path(figure_eight_waypoints)
        if fraction < 0.9: # 如果规划的路径比例小于90%，则认为失败
            rospy.logwarn(f"8字形轨迹规划不完整 (fraction: {fraction:.2f})，可能无法完全执行。")
            # 可以放宽裕度，比如fraction > 0.1
        
        # 添加执行步骤
        if plan and fraction > 0.5: # 仅当规划部分成功时才执行
            rospy.loginfo("开始执行8字形轨迹...")
            self.execute_plan(plan)
        else:
            rospy.logerr(f"8字形轨迹规划失败或成功率过低 (fraction: {fraction:.2f})，不执行。")

        rospy.loginfo("8字形轨迹演示完成。")
        rospy.loginfo("按回车键继续...")
        input()

        # 2. 回到准备姿态（或者另一个中间姿态）
        rospy.loginfo("--> 移回准备姿态...")
        if not self.move_J(ready_joint_angles): # 可以选择回到同一个准备姿态或另一个
             rospy.logwarn("移回准备姿态失败。")
        current_eef_pose = self.move_group.get_current_pose().pose # 更新姿态

        # 3. 椭圆轨迹演示
        rospy.loginfo("============ 开始椭圆轨迹演示 ============")
        ellipse_waypoints = self.ellipse_gen(radius_a=0.18, radius_b=0.1, steps=120, center_pose=current_eef_pose)
        if not ellipse_waypoints:
            rospy.logerr("生成椭圆轨迹失败。")
            return

        self.show_path(ellipse_waypoints, ns="ellipse", r=0.0, g=1.0, b=0.5) # 青绿色

        rospy.loginfo("规划椭圆轨迹...")
        plan, fraction = self.plan_cartesian_path(ellipse_waypoints)
        if fraction < 0.9:
            rospy.logwarn(f"椭圆轨迹规划不完整 (fraction: {fraction:.2f})，可能无法完全执行。")

        # 添加执行步骤
        if plan and fraction > 0.5: # 仅当规划部分成功时才执行
            rospy.loginfo("开始执行椭圆轨迹...")
            self.execute_plan(plan)
        else:
            rospy.logerr(f"椭圆轨迹规划失败或成功率过低 (fraction: {fraction:.2f})，不执行。")

        rospy.loginfo("椭圆轨迹演示完成。")
        rospy.loginfo("按回车键继续...")
        input()

        # 4. 回到初始姿态或home姿态
        rospy.loginfo("--> 演示结束，尝试回到初始关节姿态...")
        home_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 全部为0
        if not self.move_J(home_joint_angles):
            rospy.logwarn("回到 Home 关节姿态失败。")

        rospy.loginfo("============ 轨迹控制演示完成 ============")

def main():
    try:
        controller = TrajectoryControl()
        controller.run()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == '__main__':
    main()


