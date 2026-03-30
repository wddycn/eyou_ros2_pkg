import rclpy
from rclpy.node import Node
import socket
import math

from msgs_init.msg import VRget


def quat_to_euler(qx, qy, qz, qw):
    """
    四元数 转 欧拉角（ROLL, PITCH, YAW）
    输出：x旋转(roll), y旋转(pitch), z旋转(yaw)，单位：弧度
    """
    # 欧拉角公式（标准3D旋转转换）
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


class TCPReceiver(Node):

    def __init__(self):
        super().__init__('vr_tcp_receiver')

        self.publisher_ = self.create_publisher(VRget, 'vr_controller', 10)

        # TCP服务端
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', 5005))
        self.sock.listen(1)

        self.get_logger().info("等待 Unity 连接...")
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"连接成功: {addr}")

        self.buffer = ""

        # 100Hz读取
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        try:
            data = self.conn.recv(1024).decode()
            if not data:
                return

            self.buffer += data

            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                self.process_line(line)

        except Exception as e:
            self.get_logger().warn(f"数据接收异常: {str(e)}")

    def process_line(self, line):
        parts = line.split(',')

        if len(parts) < 26:
            return

        msg = VRget()

        # ===== 位姿 =====
        lp = [float(x) for x in parts[0:3]]
        lq = [float(x) for x in parts[3:7]]

        rp = [float(x) for x in parts[7:10]]
        rq = [float(x) for x in parts[10:14]]

        # ===== 输入 =====
        msg.left_trigger = float(parts[14])
        msg.left_grip = float(parts[15])
        msg.left_joystick = [float(parts[16]), float(parts[17])]
        msg.left_button_x = parts[18] == "True"
        msg.left_button_y = parts[19] == "True"

        msg.right_trigger = float(parts[20])
        msg.right_grip = float(parts[21])
        msg.right_joystick = [float(parts[22]), float(parts[23])]
        msg.right_button_a = parts[24] == "True"
        msg.right_button_b = parts[25] == "True"

        # ===== 坐标系修正（X轴取反）=====
        lp[0] = -lp[0]
        rp[0] = -rp[0]

        lq[0] = -lq[0]
        rq[0] = -rq[0]

        # ===== 四元数 → 欧拉角（直接输出绕X/Y/Z旋转弧度）=====
        left_euler = quat_to_euler(*lq)
        right_euler = quat_to_euler(*rq)

        # ===== 填入msg（left_axis / right_axis 现在 = 欧拉角）=====
        msg.left_pos = lp
        msg.left_axis = left_euler  # [X旋转, Y旋转, Z旋转] 单位：弧度

        msg.right_pos = rp
        msg.right_axis = right_euler

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TCPReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
