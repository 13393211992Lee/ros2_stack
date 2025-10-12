import rclpy
from rclpy.node import Node
from status_interface.msg import SystemStatus
import psutil
import platform

class SysStatusPub(Node):
    def __init__(self ,node_name ):
        super().__init__(node_name)
        self.status_publisher_ = self.create_publisher(SystemStatus, 'sys_status', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """
        builtin_interfaces/Time stamp #纪律时间戳
            int32 sec
            uint32 nanosec
        string hostname                 # 主机
        float32 cpu_percent             # cpu使用率
        float32 memory_percent          # 内存占用大小
        float32 memory_total            # 内存总大小
        float32 memory_available        # 内存可用大小
        float64 net_sent                # 网络数据发送总量
        float64 net_recv                # 网络数据接收总量
        """
        cpu_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        net_io_counters = psutil.net_io_counters()

        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.hostname = platform.node()
        msg.cpu_percent = cpu_percent
        msg.memory_available = memory_info.available/1024/1024
        msg.memory_percent = memory_info.percent
        msg.memory_total = memory_info.total/1024/1024
        msg.net_sent = net_io_counters.bytes_sent/1024/1024
        msg.net_recv = net_io_counters.bytes_recv/1024/1024

        self.get_logger().info(f'发布：{str(msg)}')
        self.status_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SysStatusPub("sys_status_node"))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
