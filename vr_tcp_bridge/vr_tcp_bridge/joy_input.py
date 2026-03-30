# ==============================================
# 强制路径优先级（必须放最前）
# ==============================================
import sys
import os
TELEOP_ROOT = "/home/ycn/teleoperation/ros2_ws/src/vr_tcp_bridge/vr_tcp_bridge"
sys.path.insert(0, TELEOP_ROOT)
sys.path.append("/home/ycn/miniconda3/envs/teleoperation/lib/python3.10/site-packages")
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np
import pygame
import os
import casadi_ik

os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"


class XboxController:
    def __init__(self):
        self.x = 0.4
        self.y = 0.25
        self.z = 0.33
        self.R = self._rpy_to_matrix(1.5708, 0, 1.5708)

        self.pos_sensitivity = 0.001
        self.ori_sensitivity = 0.007
        self.deadzone = 0.1

        self.controller = self.init_controller()

    def _rpy_to_matrix(self, roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])

    def _axis_angle_to_matrix(self, axis, angle):
        if abs(angle) < 1e-6:
            return np.eye(3)
        axis = np.array(axis) / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K @ K)

    def init_controller(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("❌ 未检测到手柄")
            return None
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print("✅ 手柄已连接")
        return joy

    def handle_input(self, arm, qpos):
        pygame.event.pump()

        x_axis = self.controller.get_axis(0)
        y_axis = self.controller.get_axis(3)
        z_axis = -self.controller.get_axis(1)

        delta = np.array([-x_axis, -y_axis, z_axis]) * 0.001

        tf = arm.fk(qpos)
        R = tf[:3, :3]
        delta_world = R @ delta

        self.x += delta_world[0]
        self.y += delta_world[1]
        self.z += delta_world[2]

        hat = self.controller.get_hat(0)
        pitch_axis = -self.controller.get_axis(2)

        R_inc = (
            self._axis_angle_to_matrix([1,0,0], hat[1]*0.007) @
            self._axis_angle_to_matrix([0,1,0], pitch_axis*0.007) @
            self._axis_angle_to_matrix([0,0,1], hat[0]*0.007)
        )

        self.R = self.R @ R_inc

    def get_transform(self):
        tf = np.eye(4)
        tf[:3, :3] = self.R
        tf[:3, 3] = [self.x, self.y, self.z]
        return tf


class JoyIKNode(Node):
    def __init__(self):
        super().__init__('joy_input')

        self.pub = self.create_publisher(Float64MultiArray, 'target_joints', 10)

        self.arm = casadi_ik.Kinematics("grasp_point")
        self.arm.buildFromMJCF('/home/ycn/teleoperation/ros2_ws/src/vr_tcp_bridge/urdf/eyou_arm.xml')

        self.controller = XboxController()
        self.last_q = np.zeros(self.arm.model.nq)

        self.timer = self.create_timer(0.02, self.loop)  # 50Hz

    def loop(self):
        if self.controller.controller is None:
            return

        self.controller.handle_input(self.arm, self.last_q)

        tf = self.controller.get_transform()
        q, _ = self.arm.ik(tf, current_arm_motor_q=self.last_q)
        self.last_q = q

        msg = Float64MultiArray()
        msg.data = q[:6].tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JoyIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()