import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class Pc2ArmNode(Node):
    def __init__(self):
        super().__init__('pc2arm')

        # ✅ 改成 position controller 话题
        self.pub_arm = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.sub_target = self.create_subscription(
            Float64MultiArray,
            '/target_joints',
            self.target_callback,
            10
        )

        self.current_target = None
        self.filtered_target = None

        # ✅ 控制频率（200Hz OK）
        self.dt = 0.005
        self.timer = self.create_timer(self.dt, self.control_loop)

        # ✅ 滤波（关键参数）
        self.alpha = 0.15   # 比你之前更稳一点

        # ✅ 限速（防止突变）
        self.max_delta = 0.02  # 每周期最大变化（弧度）

        self.get_logger().info("✅ 桥接节点（流式控制版）启动")

    def target_callback(self, msg):
        self.current_target = np.array(msg.data)

    def control_loop(self):
        if self.current_target is None:
            return

        # ========================
        # 1️⃣ 初始化
        # ========================
        if self.filtered_target is None:
            self.filtered_target = self.current_target.copy()
            return

        # ========================
        # 2️⃣ 低通滤波（抗抖）
        # ========================
        filtered = (
            self.alpha * self.current_target +
            (1 - self.alpha) * self.filtered_target
        )

        # ========================
        # 3️⃣ 限速（非常关键！）
        # ========================
        delta = filtered - self.filtered_target
        delta = np.clip(delta, -self.max_delta, self.max_delta)

        self.filtered_target = self.filtered_target + delta

        # ========================
        # 4️⃣ 发布（直接位置流）
        # ========================
        msg = Float64MultiArray()
        msg.data = self.filtered_target.tolist()

        self.pub_arm.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Pc2ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()