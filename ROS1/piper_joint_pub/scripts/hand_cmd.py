#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from hand_api import *

class HandCmd:
    def __init__(self):
        rospy.init_node('hand_cmd', anonymous=True)
        self.sub = rospy.Subscriber('hand_cmd', String, self.cmd_callback)
        self.controller = None
        self.connect_hand()

    def connect_hand(self):
        """连接手部控制器"""
        try:
            self.controller = HandApi()
            self.controller.ConnectPort()
            rospy.loginfo("Hand controller connected successfully")
        except Exception as e:
            rospy.logerr("Failed to connect hand controller: %s", str(e))
            self.controller = None

    def open_hand(self):
        """打开手部"""
        if self.controller:
            try:
                self.controller.FingerPosCtrl(0, 0, 0, 0, 0, 0)
                rospy.loginfo("Hand opened successfully")
            except Exception as e:
                rospy.logerr("Failed to open hand: %s", str(e))
        else:
            rospy.logwarn("Hand controller not connected, attempting to reconnect...")
            self.connect_hand()

    def close_hand(self):
        """闭合手部"""
        if self.controller:
            try:
                self.controller.FingerPosCtrl(50, 50, 100, 100, 100, 100)
                rospy.loginfo("Hand closed successfully")
            except Exception as e:
                rospy.logerr("Failed to close hand: %s", str(e))
        else:
            rospy.logwarn("Hand controller not connected, attempting to reconnect...")
            self.connect_hand()

    def custom_gesture(self, finger1=0, finger2=0, finger3=0, finger4=0, finger5=0, finger6=0):
        """自定义手势"""
        if self.controller:
            try:
                self.controller.FingerPosCtrl(finger1, finger2, finger3, finger4, finger5, finger6)
                rospy.loginfo("Custom gesture executed: [%s]" %
                              str([finger1, finger2, finger3, finger4, finger5, finger6]))
            except Exception as e:
                rospy.logerr("Failed to execute custom gesture: %s", str(e))
        else:
            rospy.logwarn("Hand controller not connected, attempting to reconnect...")
            self.connect_hand()

    def cmd_callback(self, msg):
        """命令回调"""
        hand_pose = msg.data.strip().lower()
        rospy.loginfo("Received command: %s", hand_pose)

        if hand_pose == "open":
            self.open_hand()

        elif hand_pose == "close":
            self.close_hand()

        elif hand_pose == "nice":
            # 示例：和平手势
            self.custom_gesture(0, 0, 100, 100, 100, 100)

        elif hand_pose == "direction":
            # 示例：和平手势
            self.custom_gesture(0, 0, 0, 100, 100, 100)
        else:
            rospy.logwarn("Unknown command: %s", hand_pose)
            rospy.loginfo("Available commands: open, close, point, fist, peace")

    def spin(self):
        """ROS循环"""
        rospy.loginfo("HandCmd node started, waiting for commands...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down HandCmd node...")
        finally:
            if self.controller:
                try:
                    self.controller.DisconnectPort()  # 假设存在断开方法
                    rospy.loginfo("Hand controller disconnected")
                except Exception as e:
                    rospy.logerr("Error disconnecting hand controller: %s", str(e))

if __name__ == '__main__':
    node = HandCmd()
    node.spin()
