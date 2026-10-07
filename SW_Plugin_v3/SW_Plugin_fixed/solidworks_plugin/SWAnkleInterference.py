# -*- coding: utf-8 -*-
"""
SolidWorks Interference Check Add-in for Ankle Mechanism
功能：
1. 按步长2°遍历两个关节的所有角度组合
2. 检测干涉/不干涉状态
3. 显示任意两个平面之间的角度
4. 导出结果报告

安装方法：
1. 将此文件复制到SolidWorks安装目录的APPDATA文件夹
2. 注册为COM组件
3. 在SolidWorks中添加插件

或者使用SWBasic插件框架注册
"""

import os
import sys
import math
import csv
from datetime import datetime

# SolidWorks API types
try:
    import win32com.client
    from win32com.client import constants as swconst
    SW_AVAILABLE = True
except ImportError:
    SW_AVAILABLE = False
    print("Warning: pywin32 not installed. This script must run inside SolidWorks.")


class AnkleInterferenceChecker:
    """人形机器人脚踝关节干涉检查工具"""

    def __init__(self):
        self.sw_app = None
        self.sw_model = None
        self.sw_assy = None

        # 关节配置
        self.joint1_name = "Joint1"  # 第一关节名称（需根据实际修改）
        self.joint2_name = "Joint2"  # 第二关节名称（需根据实际修改）

        # 运动范围配置
        self.joint1_range = (-45, 45)  # 关节1角度范围（度）
        self.joint2_range = (-45, 45)  # 关节2角度范围（度）
        self.step_size = 2  # 步长（度）

        # 平面角度测量
        self.plane1_name = ""  # 第一个平面
        self.plane2_name = ""  # 第二个平面

        # 结果存储
        self.results = []

    def connect_to_solidworks(self):
        """连接到当前运行的SolidWorks实例"""
        if not SW_AVAILABLE:
            print("SolidWorks API not available")
            return False

        try:
            self.sw_app = win32com.client.Dispatch("SldWorks.Application")
            self.sw_app.Visible = True
            self.sw_model = self.sw_app.ActiveDoc

            if self.sw_model is None:
                print("No active document")
                return False

            if self.sw_model.GetType() != swconst.swDocASSEMBLY:
                print("Please open an assembly document")
                return False

            self.sw_assy = self.sw_model
            print(f"Connected to: {self.sw_model.GetTitle()}")
            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def get_joint(self, joint_name):
        """获取关节对象"""
        if not self.sw_assy:
            return None

        joints = self.sw_assy.GetJoints2()
        for i in range(joints.Count):
            joint = joints.Item(i)
            if joint.Name == joint_name:
                return joint
        return None

    def set_joint_angle(self, joint_name, angle_deg):
        """设置关节角度"""
        joint = self.get_joint(joint_name)
        if joint is None:
            print(f"Joint '{joint_name}' not found")
            return False

        try:
            # 设置关节运动限制
            angle_rad = math.radians(angle_deg)

            # 遍历关节
            joint_params = joint.JointMotionParameters
            if joint_params.Count > 0:
                param = joint_params.Item(0)
                param.MinValue = math.radians(self.joint1_range[0])
                param.MaxValue = math.radians(self.joint1_range[1])

            # 更新装配体
            self.sw_assy.EditRebuild3()
            return True

        except Exception as e:
            print(f"Failed to set joint angle: {e}")
            return False

    def check_interference(self):
        """检查当前配置的干涉状态"""
        if not self.sw_assy:
            return None, None

        try:
            # 运行干涉检查
            interference_data = self.sw_assy.GetInterferenceData()

            if interference_data is None:
                return 0, []  # 无干涉

            # 解析干涉结果
            interference_count = interference_data[0]
            interference_components = interference_data[1:]

            return interference_count, interference_components

        except Exception as e:
            print(f"Interference check failed: {e}")
            return None, None

    def get_plane_angle(self, plane1_name, plane2_name):
        """计算两个平面之间的角度"""
        try:
            # 获取平面
            plane1 = self.sw_model.GetEntityByName(plane1_name, swconst.swSelPLANES)
            plane2 = self.sw_model.GetEntityByName(plane2_name, swconst.swSelPLANES)

            if plane1 is None or plane2 is None:
                return None

            # 获取平面法向量
            normal1 = plane1.GetNormal()
            normal2 = plane2.GetNormal()

            # 计算角度
            dot = normal1[0]*normal2[0] + normal1[1]*normal2[1] + normal1[2]*normal2[2]
            mag1 = math.sqrt(sum(n**2 for n in normal1))
            mag2 = math.sqrt(sum(n**2 for n in normal2))

            if mag1 == 0 or mag2 == 0:
                return None

            cos_angle = max(-1, min(1, dot / (mag1 * mag2)))
            angle_rad = math.acos(cos_angle)
            angle_deg = math.degrees(angle_rad)

            return angle_deg

        except Exception as e:
            print(f"Plane angle calculation failed: {e}")
            return None

    def run_sweep_analysis(self):
        """执行完整的角度遍历分析"""
        if not self.connect_to_solidworks():
            return None

        print("=" * 50)
        print("开始干涉分析...")
        print(f"关节1范围: {self.joint1_range[0]}° ~ {self.joint1_range[1]}°")
        print(f"关节2范围: {self.joint2_range[0]}° ~ {self.joint2_range[1]}°")
        print(f"步长: {self.step_size}°")
        print("=" * 50)

        self.results = []
        total_steps = 0
        current_step = 0

        # 计算总步数
        j1_steps = int((self.joint1_range[1] - self.joint1_range[0]) / self.step_size) + 1
        j2_steps = int((self.joint2_range[1] - self.joint2_range[0]) / self.step_size) + 1
        total_steps = j1_steps * j2_steps

        print(f"总共 {total_steps} 种配置需要测试")
        print()

        # 遍历所有角度组合
        j1 = self.joint1_range[0]
        while j1 <= self.joint1_range[1] + 0.001:
            j2 = self.joint2_range[0]
            while j2 <= self.joint2_range[1] + 0.001:

                current_step += 1

                # 设置关节角度
                self.set_joint_angle(self.joint1_name, j1)
                self.set_joint_angle(self.joint2_name, j2)

                # 检查干涉
                interference_count, components = self.check_interference()

                # 记录结果
                result = {
                    'step': current_step,
                    'joint1_angle': round(j1, 2),
                    'joint2_angle': round(j2, 2),
                    'interference': interference_count > 0 if interference_count is not None else 'N/A',
                    'interference_count': interference_count if interference_count else 0,
                    'components': str(components) if components else ''
                }

                self.results.append(result)

                # 打印进度
                status = "干涉!" if result['interference'] else "正常"
                print(f"[{current_step}/{total_steps}] J1={j1:6.1f}° J2={j2:6.1f}° -> {status}")

                j2 += self.step_size

            j1 += self.step_size

        print()
        print("分析完成！")
        self.print_summary()

        return self.results

    def print_summary(self):
        """打印分析摘要"""
        if not self.results:
            return

        total = len(self.results)
        interference_count = sum(1 for r in self.results if r['interference'] is True)
        normal_count = sum(1 for r in self.results if r['interference'] is False)

        print("=" * 50)
        print("分析结果摘要")
        print("=" * 50)
        print(f"总配置数: {total}")
        print(f"干涉状态: {interference_count} ({100*interference_count/total:.1f}%)")
        print(f"正常状态: {normal_count} ({100*normal_count/total:.1f}%)")

        # 找出干涉的关节角度范围
        if interference_count > 0:
            print()
            print("干涉区域:")
            for r in self.results:
                if r['interference']:
                    print(f"  J1={r['joint1_angle']}°, J2={r['joint2_angle']}°")

        # 安全范围
        if normal_count > 0:
            safe_configs = [r for r in self.results if not r['interference']]
            j1_safe = set(r['joint1_angle'] for r in safe_configs)
            j2_safe = set(r['joint2_angle'] for r in safe_configs)
            print()
            print(f"安全J1角度: {min(j1_safe)}° ~ {max(j1_safe)}°")
            print(f"安全J2角度: {min(j2_safe)}° ~ {max(j2_safe)}°")

    def export_results(self, filename=None):
        """导出结果到CSV文件"""
        if not self.results:
            print("No results to export")
            return

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"interference_analysis_{timestamp}.csv"

        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
                writer.writeheader()
                writer.writerows(self.results)

            print(f"Results exported to: {filename}")
            return filename

        except Exception as e:
            print(f"Export failed: {e}")
            return None

    def measure_plane_angle(self, plane1_name, plane2_name):
        """测量两个平面之间的角度"""
        if not self.connect_to_solidworks():
            return None

        self.plane1_name = plane1_name
        self.plane2_name = plane2_name

        angle = self.get_plane_angle(plane1_name, plane2_name)

        if angle is not None:
            print(f"平面角度 ({plane1_name} vs {plane2_name}): {angle:.2f}°")
            return angle
        else:
            print("Failed to measure plane angle")
            return None


# =====================================================
# 用户界面部分 (用于SolidWorks工具栏按钮)
# =====================================================

class SWInterferencePlugin:
    """SolidWorks插件界面"""

    def __init__(self):
        self.checker = AnkleInterferenceChecker()

    def show_configuration_dialog(self):
        """显示配置对话框"""
        # 用户需要根据实际情况修改这些配置
        config = {
            'joint1_name': input("Enter Joint 1 Name: "),
            'joint2_name': input("Enter Joint 2 Name: "),
            'joint1_min': float(input("Joint 1 Min Angle (deg): ")),
            'joint1_max': float(input("Joint 1 Max Angle (deg): ")),
            'joint2_min': float(input("Joint 2 Min Angle (deg): ")),
            'joint2_max': float(input("Joint 2 Max Angle (deg): ")),
            'step_size': float(input("Step Size (deg, default 2): ") or "2"),
        }

        self.checker.joint1_name = config['joint1_name']
        self.checker.joint2_name = config['joint2_name']
        self.checker.joint1_range = (config['joint1_min'], config['joint1_max'])
        self.checker.joint2_range = (config['joint2_min'], config['joint2_max'])
        self.checker.step_size = config['step_size']

        return config

    def run_analysis(self):
        """运行干涉分析"""
        self.show_configuration_dialog()
        results = self.checker.run_sweep_analysis()

        if results:
            self.checker.export_results()

    def measure_planes(self):
        """测量平面角度"""
        plane1 = input("Enter Plane 1 Name: ")
        plane2 = input("Enter Plane 2 Name: ")
        self.checker.measure_plane_angle(plane1, plane2)


# =====================================================
# 主程序入口
# =====================================================

if __name__ == "__main__":
    print("=" * 60)
    print("SolidWorks Ankle Mechanism Interference Checker")
    print("=" * 60)
    print()

    plugin = SWInterferencePlugin()

    while True:
        print()
        print("Menu:")
        print("1. Run Interference Analysis")
        print("2. Measure Plane Angles")
        print("3. Exit")
        choice = input("Enter choice (1-3): ").strip()

        if choice == "1":
            plugin.run_analysis()
        elif choice == "2":
            plugin.measure_planes()
        elif choice == "3":
            print("Goodbye!")
            break
        else:
            print("Invalid choice")
