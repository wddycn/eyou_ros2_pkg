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

import mujoco
import mujoco_viewer
import numpy as np
import threading
import time

SCENE_XML_PATH = '/home/ycn/teleoperation/ros2_ws/src/vr_tcp_bridge/urdf/scene.xml'


class RobotSimNode(Node):

    def __init__(self):
        super().__init__('robot_sim')

        # ✅ 创建viewer
        self.viewer = mujoco_viewer.CustomViewer(
            SCENE_XML_PATH,
            distance=1.5,
            azimuth=135,
            elevation=-30
        )

        self.model = self.viewer.model
        self.data = self.viewer.data

        self.target_q = np.zeros(6)

        self.create_subscription(
            Float64MultiArray,
            'target_joints',
            self.callback,
            10
        )

        # 🔥 启动 Mujoco线程（用run_loop）
        self.sim_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.sim_thread.start()

    def callback(self, msg):
        self.target_q = np.array(msg.data)

    def viewer_loop(self):
        print("✅ Mujoco Viewer线程启动")

        # 👇 重写 runFunc（关键技巧）
        def runFunc_override():
            self.data.qpos[:6] = self.target_q
            mujoco.mj_step(self.model, self.data)
            time.sleep(0.01)

        # 👇 动态替换
        self.viewer.runFunc = runFunc_override

        # 👇 用你原来的机制
        self.viewer.run_loop()


def main():
    rclpy.init()

    node = RobotSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()