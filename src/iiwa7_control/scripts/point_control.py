#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sin, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
from visualization_msgs.msg import Marker


class Iiwa7Controller:
    def __init__(self):
        # 初始化moveit_commander和rospy节点
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('iiwa7_controller', anonymous=True)

        # 实例化RobotCommander对象，提供机器人运动学模型和当前的机器人状态
        robot = moveit_commander.RobotCommander()

        # 实例化PlanningSceneInterface对象，提供与环境交互的接口
        scene = moveit_commander.PlanningSceneInterface()

        # 实例化MoveGroupCommander对象，用于控制机器人的规划组
        group_name = "manipulator"  # 在MoveIt Setup Assistant中创建的规划组名称
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        # 设置规划时间限制为15秒（默认为5秒）
        move_group.set_planning_time(15.0)

        # 创建DisplayTrajectory发布者，用于显示轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # 保存引用
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

        # 获取规划参考坐标系的名称
        self.planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        # 获取末端执行器链的名称
        self.eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        # 获取机器人的所有组
        self.group_names = robot.get_group_names()
        print("============ Available Planning Groups:", self.group_names)

        # 打印机器人当前状态
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.marker_id = 0

    def go_to_joint_state(self, joint_goal):
        """移动机械臂到指定的关节状态"""
        # 设置关节目标
        self.move_group.go(joint_goal, wait=True)
        # 调用stop()确保没有残留的运动
        self.move_group.stop()
        
        # 检查当前状态与目标状态是否接近
        current_joints = self.move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal):
        """移动机械臂到指定的位姿"""
        # 设置位姿目标
        self.move_group.set_pose_target(pose_goal)

        # 调用规划器进行运动规划和执行
        plan = self.move_group.go(wait=True)
        # 确保没有残留的运动
        self.move_group.stop()
        # 清除目标位姿
        self.move_group.clear_pose_targets()

        # 检查当前位姿与目标位姿是否接近
        current_pose = self.move_group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints):
        """规划笛卡尔路径，通过指定的中间点"""
        # 使用compute_cartesian_path方法计算路径
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # 路径点
                                           0.01,        # eef步长
                                           0.0)         # 跳跃阈值

        # 返回计算出的轨迹
        return plan, fraction

    def execute_plan(self, plan):
        """执行传入的轨迹计划"""
        self.move_group.execute(plan, wait=True)

    def all_close(self, goal, actual, tolerance):
        """
        判断一组关节角度或位姿是否足够接近
        """
        all_equal = True
        if type(goal) is list:
            for i in range(len(goal)):
                if abs(actual[i] - goal[i]) > tolerance:
                    return False
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual, tolerance)
        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
        return True

    def publish_marker(self, pose, color=(0, 1, 0)):
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_points"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2] if len(color) > 2 else 0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)
        self.marker_id += 1

    def point_control_demo(self):
        """点位控制演示：控制机械臂运动到6个不同的空间点，并可视化目标点"""
        print("============ 执行空间6点位控制演示")
        
        # 定义6个空间点
        pose_goals = []
        # 1. 前上
        pose1 = geometry_msgs.msg.Pose()
        pose1.orientation.w = 1.0
        pose1.position.x = 0.3
        pose1.position.y = 0.0
        pose1.position.z = 0.55
        pose_goals.append(pose1)
        # 2. 右上
        pose2 = geometry_msgs.msg.Pose()
        pose2.orientation.w = 1.0
        pose2.position.x = 0.25
        pose2.position.y = 0.15
        pose2.position.z = 0.55
        pose_goals.append(pose2)
        # 3. 右下
        pose3 = geometry_msgs.msg.Pose()
        pose3.orientation.w = 1.0
        pose3.position.x = 0.25
        pose3.position.y = 0.15
        pose3.position.z = 0.45
        pose_goals.append(pose3)
        # 4. 后下
        pose4 = geometry_msgs.msg.Pose()
        pose4.orientation.w = 1.0
        pose4.position.x = 0.2
        pose4.position.y = 0.0
        pose4.position.z = 0.45
        pose_goals.append(pose4)
        # 5. 左下
        pose5 = geometry_msgs.msg.Pose()
        pose5.orientation.w = 1.0
        pose5.position.x = 0.25
        pose5.position.y = -0.15
        pose5.position.z = 0.45
        pose_goals.append(pose5)
        # 6. 左上
        pose6 = geometry_msgs.msg.Pose()
        pose6.orientation.w = 1.0
        pose6.position.x = 0.25
        pose6.position.y = -0.15
        pose6.position.z = 0.55
        pose_goals.append(pose6)

        # 依次运动到6个点，并可视化目标点
        for i, pose_goal in enumerate(pose_goals):
            print(f"移动到空间点 {i+1}")
            self.publish_marker(pose_goal, color=(1,0,0))  # 红色marker
            self.go_to_pose_goal(pose_goal)
            rospy.sleep(1)
        print("============ 空间6点位控制演示完成!")

def main():
    try:
        controller = Iiwa7Controller()
        
        # 等待RViz显示
        print("等待RViz...")
        rospy.sleep(2)
        
        input("按 Enter 键开始点位控制演示...")
        controller.point_control_demo()
        
        print("演示完成!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main() 