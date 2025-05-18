#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sin, cos, sqrt
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


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

    def go_to_start_pose(self):
        """移动到轨迹的起始位置"""
        start_pose = geometry_msgs.msg.Pose()
        start_pose.orientation.w = 1.0
        start_pose.position.x = 0.3  # 更保守的起始位置
        start_pose.position.y = 0.0
        start_pose.position.z = 0.5
        self.go_to_pose_goal(start_pose)
        return start_pose

    def figure_eight_demo(self, scale=0.05, steps=20):
        """生成并执行8字形轨迹"""
        print("============ 执行8字形轨迹演示")
        
        # 移动到起始位置
        start_pose = self.go_to_start_pose()
        
        # 创建轨迹点
        waypoints = []
        current_pose = copy.deepcopy(start_pose)
        
        # 生成8字形轨迹的参数方程点
        for t in range(steps):
            theta = 2.0 * pi * t / steps
            # 参数方程: x = a * sin(t), y = b * sin(t) * cos(t)
            # 为了生成一个水平的8字形
            current_pose.position.y = scale * sin(theta)             # y轴方向
            current_pose.position.z = scale * sin(2*theta) / 2 + 0.5 # z轴方向
            # 保持x位置不变
            
            waypoints.append(copy.deepcopy(current_pose))
        
        # 添加起始点闭合轨迹
        waypoints.append(copy.deepcopy(start_pose))
        
        # 规划笛卡尔路径
        plan, fraction = self.plan_cartesian_path(waypoints)
        print(f"规划完成: {fraction*100:.2f}% 的路径有效")
        
        # 执行轨迹
        if fraction > 0.5:  # 如果至少50%的路径有效
            self.execute_plan(plan)
        else:
            print("生成的轨迹不足够完整，放弃执行")
        
        print("============ 8字形轨迹演示完成!")

    def ellipse_demo(self, a=0.05, b=0.03, steps=20):
        """生成并执行椭圆轨迹"""
        print("============ 执行椭圆轨迹演示")
        
        # 移动到起始位置
        start_pose = self.go_to_start_pose()
        
        # 创建轨迹点
        waypoints = []
        current_pose = copy.deepcopy(start_pose)
        
        # 生成椭圆轨迹的参数方程点
        for t in range(steps):
            theta = 2.0 * pi * t / steps
            # 参数方程: x = a * cos(t), y = b * sin(t)
            # 为了生成一个竖直的椭圆
            current_pose.position.y = a * cos(theta)         # y轴方向
            current_pose.position.z = b * sin(theta) + 0.5   # z轴方向加上偏移
            # 保持x位置不变
            
            waypoints.append(copy.deepcopy(current_pose))
        
        # 添加起始点闭合轨迹
        waypoints.append(copy.deepcopy(start_pose))
        
        # 规划笛卡尔路径
        plan, fraction = self.plan_cartesian_path(waypoints)
        print(f"规划完成: {fraction*100:.2f}% 的路径有效")
        
        # 执行轨迹
        if fraction > 0.5:  # 如果至少50%的路径有效
            self.execute_plan(plan)
        else:
            print("生成的轨迹不足够完整，放弃执行")
        
        print("============ 椭圆轨迹演示完成!")

    def spiral_demo(self, radius=0.05, height=0.05, rounds=1, steps=30):
        """生成并执行螺旋轨迹"""
        print("============ 执行螺旋轨迹演示")
        
        # 移动到起始位置
        start_pose = self.go_to_start_pose()
        
        # 创建轨迹点
        waypoints = []
        current_pose = copy.deepcopy(start_pose)
        
        # 生成螺旋轨迹点
        max_angle = 2.0 * pi * rounds
        for t in range(steps):
            angle = max_angle * t / steps
            # 计算螺旋坐标
            r = radius * (1 - t / steps)  # 半径逐渐减小
            current_pose.position.y = r * cos(angle)         # y轴方向
            current_pose.position.z = r * sin(angle) + 0.5   # z轴方向
            current_pose.position.x = 0.4 + height * t / steps  # x轴向前移动
            
            waypoints.append(copy.deepcopy(current_pose))
        
        # 规划笛卡尔路径
        plan, fraction = self.plan_cartesian_path(waypoints)
        print(f"规划完成: {fraction*100:.2f}% 的路径有效")
        
        # 执行轨迹
        if fraction > 0.5:  # 如果至少50%的路径有效
            self.execute_plan(plan)
        else:
            print("生成的轨迹不足够完整，放弃执行")
        
        print("============ 螺旋轨迹演示完成!")

def main():
    try:
        controller = Iiwa7Controller()
        
        # 等待RViz显示
        print("等待RViz...")
        rospy.sleep(2)
        
        choice = input("""
请选择轨迹演示类型:
1. 8字形轨迹
2. 椭圆轨迹
3. 螺旋轨迹
请输入数字(1-3): """)
        
        if choice == '1':
            controller.figure_eight_demo()
        elif choice == '2':
            controller.ellipse_demo()
        elif choice == '3':
            controller.spiral_demo()
        else:
            print("无效选择，退出程序")
        
        print("演示完成!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main() 