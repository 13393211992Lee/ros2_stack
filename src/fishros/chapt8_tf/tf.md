
### 静态发布 static_transform_publisher
$ ros2 run tf2_ros static_transform_publisher -h

usage: static_transform_publisher [--x X] [--y Y] [--z Z] [--qx QX] [--qy QY] [--qz QZ] [--qw QW] [--roll ROLL] [--pitch PITCH] [--yaw YAW] --frame-id FRAME_ID --child-frame-id CHILD_FRAME_ID

required arguments:
  --frame-id FRAME_ID parent frame
  --child-frame-id CHILD_FRAME_ID child frame id

optional arguments:
  --x X                 x component of translation
  --y Y                 y component of translation
  --z Z                 z component of translation
  --qx QX               x component of quaternion rotation
  --qy QY               y component of quaternion rotation
  --qz QZ               z component of quaternion rotation
  --qw QW               w component of quaternion rotation
  --roll ROLL           roll component Euler rotation
  --pitch PITCH         pitch component Euler rotation
  --yaw YAW             yaw component Euler rotation

示例：
发布 base_link to base_laser 的 tf 关系
$ ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0.0 --z 0.2 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id base_laser
[INFO] [1760114130.279174479] [static_transform_publisher_AxW3DQGT8Vre1Eqk]: Spinning until stopped - publishing transform
translation: ('0.100000', '0.000000', '0.200000')
rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
from 'base_link' to 'base_laser'

发布 base_laser to wall_point 的 tf 关系
$ ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_laser --child-frame-id wall_point
[INFO] [1760114297.177048905] [static_transform_publisher_890iKTS9jAcGFAVz]: Spinning until stopped - publishing transform
translation: ('0.300000', '0.000000', '0.000000')
rotation: ('0.000000', '0.000000', '0.000000', '1.000000')
from 'base_laser' to 'wall_point'

查看 TF 关系
$ ros2 run tf2_ros tf2_echo base_link wall_point
[INFO] [1760114437.108455426] [tf2_echo]: Waiting for transform base_link ->  wall_point: Invalid frame ID "base_link" passed to canTransform argument target_frame - frame does not exist
At time 0.0
- Translation: [0.400, 0.000, 0.200]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000  0.400
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.200
  0.000  0.000  0.000  1.000

同一个旋转的4种表达方式：
Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]        # Quaternion 
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]          # RPY <radian>
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]          # RPY <degree>
- Matrix:                                                   # Matrix
  1.000  0.000  0.000  0.400
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.200
  0.000  0.000  0.000  1.000

查看 TF 关系
$ ros2 run tf2_tools view_frames
生成 pdf & gv 两个文件


TF 原理
前提条件： 启动静态发布
$ ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0.0 --z 0.2 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id base_laser
$ ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_laser --child-frame-id wall_point

查看话题
$ ros2 topic list
/parameter_events
/rosout
/tf_static          # 该话题

$ ros2 topic info /tf_static
Type: tf2_msgs/msg/TFMessage
Publisher count: 2
Subscription count: 0

查看tf2_msgs/msg/TFMessage 类型格式：
$ ros2 interface show tf2_msgs/msg/TFMessage
geometry_msgs/TransformStamped[] transforms
	#
	#
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string child_frame_id
	Transform transform
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1

监听tf_static 话题
$ ros2 topic echo /tf_static
transforms:
- header:
    stamp:
      sec: 1760115195
      nanosec: 480985987
    frame_id: base_link
  child_frame_id: base_laser
  transform:
    translation:
      x: 0.1
      y: 0.0
      z: 0.2
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1760115203
      nanosec: 800418296
    frame_id: base_laser
  child_frame_id: wall_point
  transform:
    translation:
      x: 0.3
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---

静态发布只发布一次,然后根据qos配置,保留最近一次发布的信息,可以订阅到最近一次发布的变换信息。

静态发布 代码：
src/fishros/chapt8_tf/chapt8_tf_cpp/src/static_tf_broadcaster.cpp

仿真小车的 TF 变换（坐标变换）通常不需要手动编写发布代码，而是通过URDF/Xacro 模型定义与机器人状态发布器（robot_state_publisher） 配合实现自动发布。
物理小车的 TF 变换发布逻辑与仿真小车类似，核心仍依赖URDF 模型定义结构和关节状态数据，但区别在于：物理小车的关节状态（如轮子角度、传感器位置）需要从实际硬件（编码器、IMU 等） 获取，而非仿真环境的虚拟数据。
差异：
环节	                  仿真小车	                                                      物理小车
URDF模型	              相同，需定义 link 和 joint 关系                       	     相同，需精确匹配物理结构
关节状态来源	           仿真环境Gazebo自动生成	物理传感器编码器、IMU读取                 物理传感器编码器、IMU 读取
TF 发布节点	            robot_state_publisher	                                     robot_state_publisher

物理小车 TF 发布的完整流程
1. 编写精确的 URDF 模型
物理小车的 URDF 必须严格匹配实际机械结构，尤其是关节的位置、旋转轴和类型
若 URDF 与物理结构不符（如轮子位置偏差），会导致 TF 变换错误，进而影响导航、定位等功能

2. 从硬件获取关节状态（核心差异点）
物理小车的关节状态（如轮子转动角度）需要通过传感器读取并发布到/joint_states话题
该话题 包含关节名称、位置、速度等信息
  实现方式1：直接读取编码器数据
  import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import JointState

  class JointStatePublisher(Node):
      def __init__(self):
          super().__init__('joint_state_publisher')
          self.pub = self.create_publisher(JointState, '/joint_states', 10)
          self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz发布

      def timer_callback(self):
          msg = JointState()
          msg.header.stamp = self.get_clock().now().to_msg()
          msg.name = ['left_wheel_joint', 'right_wheel_joint']  # 关节名称需与URDF一致
          # 从硬件读取角度（示例：假设左轮转动了1.2弧度，右轮1.3弧度）
          msg.position = [1.2, 1.3]  # 单位：弧度
          self.pub.publish(msg)

  if __name__ == '__main__':
      rclpy.init()
      node = JointStatePublisher()
      rclpy.spin(node)

  实现方式2：
  若使用成熟的电机驱动板（如 RoboMaster M2006 电机 + C610 电调），可直接使用官方 ROS 接口包（如rm_msg），
  其内置关节状态发布功能，无需重复开发。

3. 启动robot_state_publisher节点
robot_state_publisher节点的作用与仿真中完全一致：
  从robot_description参数读取 URDF 模型，解析 link 和 joint 的静态关系。
  订阅/joint_states话题，获取动态关节角度。
  自动计算所有 link 的坐标变换，通过/tf（动态变换）和/tf_static（固定变换）发布。

4.验证 TF 发布
  查看话题：确保/tf、/tf_static和/joint_states正常发布。
    ros2 topic list | grep -E "tf|joint_states"
    ros2 echo /tf 
    ros2 echo /tf_static
  Rviz2  
    添加TF显示， 设置固定坐标系：base_link, 观察tf是否一致


#### 首演标定
### 动态变换发布
安装：$ sudo apt install ros-$ROS_DISTRO-tf-transformations
动态 TF 变换 没有直接对应的终端指令
动态发布 代码： src/fishros/chapt8_tf/chapt8_tf_cpp/src/tf_broadcaster.cpp

#### 查询tf关系
代码: src/fishros/chapt8_tf/chapt8_tf_cpp/src/tf_listener.cpp