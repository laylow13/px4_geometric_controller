## 功能介绍
一个适用于无人机的几何控制器ROS2包，使用PX4作为底层控制器（接受期望推力和姿态指令，生成电机控制指令），借助于microRTPS agent与PX4 STACK通信。
- base_env功能包提供了动捕数据转发程序，期望轨迹生成器，仿真状态数据转发程序，以及SITL仿真和实物测试的launch文件。
- controller功能包提供了核心的几何控制器节点和扰动观测器节点。
- px4_msgs子模块为PX4 1.13版本使用的消息类型。
- px4_ros_com子模块包含PX4 1.13版本使用的dds agent
