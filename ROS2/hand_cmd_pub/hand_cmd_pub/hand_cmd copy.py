#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hand_api import *


class SingleHand:
    """单独一个手的控制器，包括 HandApi 和 subscription"""
    def __init__(self, node, hand_id, can_port):
        self.node = node
        self.hand_id = hand_id
        self.can_port = can_port

        self.controller = None
        self.connect_hand()

        # 构建独立订阅话题
        topic_name = f"/piper_{hand_id}/hand_cmd"
        self.node.get_logger().info(f"Creating subscriber for: {topic_name}")

        self.subscription = node.create_subscription(
            String,
            topic_name,
            self.cmd_callback,
            10
        )

    def connect_hand(self):
        """连接单个手部控制器"""
        try:
            self.controller = HandApi(can_name=self.can_port)
            self.controller.ConnectPort()
            self.node.get_logger().info(f"[hand{self.hand_id}] Connected on {self.can_port}")
        except Exception as e:
            self.node.get_logger().error(f"[hand{self.hand_id}] Connect error: {str(e)}")
            self.controller = None

    def exec_gesture(self, values):
        """执行手势操作"""
        if self.controller:
            try:
                self.controller.FingerPosCtrl(*values)
                self.node.get_logger().info(f"[hand{self.hand_id}] Gesture: {values}")
            except Exception as e:
                self.node.get_logger().error(f"[hand{self.hand_id}] Gesture failed: {str(e)}")
        else:
            self.node.get_logger().warn(f"[hand{self.hand_id}] Not connected, reconnecting...")
            self.connect_hand()

    def cmd_callback(self, msg):
        """各手独立的消息回调"""
        cmd = msg.data.strip().lower()
        self.node.get_logger().info(f"[hand{self.hand_id}] Received cmd: {cmd}")

        if cmd == "open":
            self.exec_gesture([0, 0, 0, 0, 0, 0])

        elif cmd == "close":
            self.exec_gesture([100, 100, 100, 100, 100, 100])

        elif cmd == "point":
            self.exec_gesture([0, 20, 20, 20, 20, 0])

        elif cmd == "fist":
            self.exec_gesture([100, 100, 100, 100, 100, 100])

        elif cmd == "peace":
            self.exec_gesture([0, 0, 100, 100, 0, 0])

        else:
            self.node.get_logger().warn(f"[hand{self.hand_id}] Unknown command: {cmd}")


class HandCmd(Node):
    def __init__(self):
        super().__init__('hand_cmd')

        # ------------------ 参数声明与读取 ------------------
        self.declare_parameter("can_port", "can0")
        self.can_port = self.get_parameter("can_port").get_parameter_value().string_value

        self.declare_parameter("num_hand", 3)
        self.num_hand = self.get_parameter("num_hand").get_parameter_value().integer_value

        self.get_logger().info(f"Using CAN port: {self.can_port}")
        self.get_logger().info(f"Number of hands: {self.num_hand}")

        # ------------------ 多手控制 ------------------
        self.hands = []
        for i in range(1, self.num_hand + 1):
            hand = SingleHand(self, i, self.can_port)
            self.hands.append(hand)

    def destroy(self):
        """关闭所有手"""
        for hand in self.hands:
            if hand.controller:
                try:
                    hand.controller.DisconnectPort()
                except:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = HandCmd()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
