#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class BVHNode:
    """BVH骨骼节点"""
    def __init__(self, name):
        self.name = name
        self.offset = [0.0, 0.0, 0.0]
        self.channels = []
        self.children = []

class BVHParser:
    """简易BVH解析器"""
    def __init__(self, filepath):
        self.filepath = filepath
        self.root = None
        self.frames = []
        self.frame_time = 0.0
        self.channel_order = []   # ["Hips_Xposition","Hips_Zrotation",...]

    def parse(self):
        with open(self.filepath, "r") as f:
            lines = [line.strip() for line in f.readlines()]

        idx = 0
        if lines[idx] != "HIERARCHY":
            raise ValueError("BVH 格式错误：缺少 HIERARCHY")

        idx += 1
        self.root, idx = self._parse_joint(lines, idx)
        idx = self._parse_motion(lines, idx)

    def _parse_joint(self, lines, idx):
        line = lines[idx]

        if line.startswith("End"):
            name = "EndSite"
        else:
            name = line.split()[1]

        node = BVHNode(name)

        idx += 1
        if lines[idx] != "{":
            raise ValueError("BVH 缺少 {")
        idx += 1

        while True:
            line = lines[idx]

            if line.startswith("OFFSET"):
                node.offset = list(map(float, line.split()[1:]))

            elif line.startswith("CHANNELS"):
                parts = line.split()
                count = int(parts[1])
                node.channels = parts[2:2 + count]
                for c in node.channels:
                    self.channel_order.append(f"{node.name}_{c}")

            elif line.startswith("JOINT") or line.startswith("End"):
                child, idx = self._parse_joint(lines, idx)
                node.children.append(child)
                continue

            elif line == "}":
                return node, idx + 1

            idx += 1

    def _parse_motion(self, lines, idx):
        if lines[idx] != "MOTION":
            raise ValueError("BVH 缺少 MOTION")
        idx += 1

        frame_count = int(lines[idx].split()[1])
        idx += 1

        self.frame_time = float(lines[idx].split()[2])
        idx += 1

        for _ in range(frame_count):
            values = list(map(float, lines[idx].split()))
            self.frames.append(values)
            idx += 1

        return idx


class BVHJointStatePublisher(Node):
    """ROS2 JointState 发布器"""
    def __init__(self):
        super().__init__("bvh_jointstate_pub")

        self.declare_parameter("file_path", "")
        filepath = self.get_parameter("file_path").get_parameter_value().string_value

        if filepath == "":
            self.get_logger().error("请通过 file_path 指定 BVH 文件路径")
            return

        # 解析 BVH 文件
        self.parser = BVHParser(filepath)
        self.parser.parse()

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.frame_index = 0

        timer_period = self.parser.frame_time   # 动画实际速度
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"BVH加载成功: {filepath}")
        self.get_logger().info(f"帧数: {len(self.parser.frames)}, frame_time={self.parser.frame_time}")

    def timer_callback(self):
        frame = self.parser.frames[self.frame_index]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        names = []
        positions = []

        # BVH 通道按顺序放入 JointState
        for i, channel_name in enumerate(self.parser.channel_order):
            joint = channel_name.split("_")[0]
            channel = channel_name.split("_")[1]

            if "rotation" in channel.lower():
                if joint not in names:
                    names.append(joint)
                    positions.append(0.0)

                idx = names.index(joint)
                positions[idx] += frame[i] * 3.14159 / 180.0   # 转成弧度

        msg.name = names
        msg.position = positions

        self.publisher.publish(msg)

        self.frame_index = (self.frame_index + 1) % len(self.parser.frames)


def main(args=None):
    rclpy.init(args=args)
    node = BVHJointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
