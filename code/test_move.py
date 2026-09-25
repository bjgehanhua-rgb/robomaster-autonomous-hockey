#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

ROBOT_ID = 9

class TestMove(Node):
    def __init__(self):
        super().__init__(f"test_move_robot{ROBOT_ID}")
        self.pub = self.create_publisher(Twist, f"/robot{ROBOT_ID}/cmd_vel", 10)

    def send_cmd(self, vx, wz, duration):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz

        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(cmd)
            time.sleep(0.05)

        self.stop()

    def stop(self):
        cmd = Twist()
        self.pub.publish(cmd)
        time.sleep(0.5)


def main():
    rclpy.init()
    node = TestMove()
    print(f"Testing Robot {ROBOT_ID}")

    print("Forward slowly...")
    node.send_cmd(1.0, 0.0, 5.0)

    print("Backward slowly...")
    node.send_cmd(-1.0, 0.0, 5.0)

    print("Rotate left slowly...")
    node.send_cmd(0.0, 0.25, 3.0)

    print("Rotate right slowly...")
    node.send_cmd(0.0, -0.25, 3.0)

    print("Done.")
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()