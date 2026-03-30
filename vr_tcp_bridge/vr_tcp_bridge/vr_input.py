# ==============================================
# 强制路径优先级
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

import socket
import numpy as np
import casadi_ik


# ===============================
# 四元数 -> 旋转矩阵
# ===============================
def quat_to_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
    ])


# ===============================
# VR 相对位姿控制器（最终版）
# ===============================
class VRController:
    def __init__(self, arm, qpos):
        # ⭐ 初始化：直接用当前机械臂位姿
        self.T_target = arm.fk(qpos)

        self.active = False
        self.T_vr_init = None
        self.T_robot_init = None

        self.scale = 0.9  # 可调

    def update(self, trigger, vr_pos, vr_quat, arm, qpos):
        # ===== 当前VR位姿 =====
        T_vr = np.eye(4)
        T_vr[:3, :3] = quat_to_matrix(*vr_quat)
        T_vr[:3, 3] = vr_pos

        # ===== clutch =====
        if trigger > 0.5:

            # ===== 刚按下 =====
            if not self.active:
                self.active = True

                self.T_vr_init = T_vr.copy()
                self.T_robot_init = arm.fk(qpos)

                # ⭐ 防止跳变：同步目标
                self.T_target = self.T_robot_init.copy()
                return

            # ===== 相对变换 =====
            T_rel = np.linalg.inv(self.T_vr_init) @ T_vr

            # ===== 应用到机器人 =====
            T_new = self.T_robot_init @ T_rel

            # ===== 位置缩放 =====
            T_new[:3, 3] = self.T_robot_init[:3, 3] + \
                self.scale * (T_new[:3, 3] - self.T_robot_init[:3, 3])

            self.T_target = T_new

        else:
            # 松开：退出控制
            self.active = False
            self.T_vr_init = None
            self.T_robot_init = None

    def get_target(self):
        return self.T_target


# ===============================
# 主节点（含 TCP）
# ===============================
class VRJoyIKNode(Node):
    def __init__(self):
        super().__init__('vr_right_hand_ik')

        self.pub = self.create_publisher(Float64MultiArray, 'target_joints', 10)

        # ===== IK =====
        self.arm = casadi_ik.Kinematics("grasp_point")
        self.arm.buildFromMJCF(
            '/home/ycn/teleoperation/ros2_ws/src/vr_tcp_bridge/urdf/eyou_arm.xml'
        )

        self.last_q = np.zeros(self.arm.model.nq)

        # ⭐ 初始化控制器（用当前姿态）
        self.controller = VRController(self.arm, self.last_q)

        # ===== TCP =====
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', 5005))
        self.sock.listen(1)

        self.get_logger().info("等待 Unity 连接...")
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"连接成功: {addr}")

        self.buffer = ""

        self.timer = self.create_timer(0.02, self.loop)  # 50Hz

    def loop(self):
        try:
            data = self.conn.recv(1024).decode()
            if not data:
                return

            self.buffer += data

            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                self.process_line(line)

        except Exception as e:
            self.get_logger().warn(f"接收异常: {str(e)}")

    def process_line(self, line):
        parts = line.split(',')

        if len(parts) < 26:
            return

        # ===== 右手 =====
        # rp = [float(x) for x in parts[7:10]]
        # rq = [float(x) for x in parts[10:14]]

        # right_trigger = float(parts[20])

        # ===== 左手 ======
        rp = [float(x) for x in parts[0:3]]
        rq = [float(x) for x in parts[3:7]]
        left_trigger = float(parts[14])

        # ===== 坐标修正（按你原来的）=====
        rp[0] = -rp[0]
        rq = [-rq[0], rq[1], rq[2], -rq[3]]

        # ===== 更新控制 =====
        self.controller.update(
            left_trigger,
            rp,
            rq,
            self.arm,
            self.last_q
        )

        # ===== IK =====
        T = self.controller.get_target()
        q, _ = self.arm.ik(T, current_arm_motor_q=self.last_q)
        self.last_q = q

        # ===== 发布 =====
        msg = Float64MultiArray()
        msg.data = q[:6].tolist()
        self.pub.publish(msg)


# ===============================
# main
# ===============================
def main():
    rclpy.init()
    node = VRJoyIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()