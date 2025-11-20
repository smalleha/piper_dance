#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HandCmdPublisher(Node):
    def __init__(self):
        super().__init__('hand_cmd_publisher')

        # 声明参数：默认 /piper_1/hand_cmd
        self.declare_parameter("topic_name", "/piper_1/hand_cmd")

        # 读取参数
        self.topic_name = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )

        # 创建 publisher
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)
        self.get_logger().info(f"Publishing to topic: {self.topic_name}")

        # 每 2 秒发布一次 “open” 指令（示例）
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "dance_1"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = HandCmdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
