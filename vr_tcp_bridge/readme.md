## tcp通信因上一个通信没退出而卡住，开不了下一个通信的话可以用以下命令杀死上一个进程

报错：
```bash
2026-03-26 21:10:01.671 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port17917: open_and_lock_file failed -> Function open_port_internal
Traceback (most recent call last):
  File "/home/ycn/teleoperation/ros2_ws/install/vr_tcp_bridge/lib/vr_tcp_bridge/vr_input", line 33, in <module>
    sys.exit(load_entry_point('vr-tcp-bridge==0.0.0', 'console_scripts', 'vr_input')())
  File "/home/ycn/teleoperation/ros2_ws/install/vr_tcp_bridge/lib/python3.10/site-packages/vr_tcp_bridge/vr_input.py", line 177, in main
    node = VRJoyIKNode()
  File "/home/ycn/teleoperation/ros2_ws/install/vr_tcp_bridge/lib/python3.10/site-packages/vr_tcp_bridge/vr_input.py", line 110, in __init__
    self.sock.bind(('0.0.0.0', 5005))
OSError: [Errno 98] Address already in use
[ros2run]: Process exited with failure 1
```
解决方法：
```bash
sudo fuser -k 5005/tcp
```
# 启动方式
## 手柄遥操
1. 先ssh进jetson板子系统，启动ros2_control系统（仿真不需要这一步）
```bash
ros2 launch eyou_ros2_control eyou_control.launch
```
2. 在pc端进虚拟环境并激活ros2的环境
```bash
conda activate teleoperation
source /opt/ros/humble/setup.bash
source teleoperation/ros2_ws/install/setup.bash
```
3. 启动
```bash
# VR遥操（仿真），需要确保VR设备与pc设备连接相同网络，启动launch文件后，再在vr设备当中打开控制程序
ros2 launch vr_tcp_bridge sim_vr_control.launch.py 
# 手柄遥操（仿真），需要提前插好游戏手柄
ros2 launch vr_tcp_bridge sim_joy_control.launch.py 
# VR遥操（实物）
ros2 launch vr_tcp_bridge robot_vr_control.launch.py 
# 手柄遥操（实物）
ros2 launch vr_tcp_bridge robot_joy_control.launch.py 
```
