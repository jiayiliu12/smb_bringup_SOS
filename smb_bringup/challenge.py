#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool


from geometry_msgs.msg import PointStamped


class SOS_Node(Node):

    def __init__(self):

        super().__init__('sos_node')

        self.publisher_ = self.create_publisher(PointStamped, '/goal_point', 10)

        self.subscription = self.create_subscription(

            Bool,

            '/exploration_finished',

            self.exploration_finished_callback,

            10)

        self.subscription  # prevent unused variable warning

    def publish_goal_point(self):

        msg = PointStamped()

        msg.header.frame_id = "odom"

        self.publisher_.publish(msg)

        self.get_logger().info('Publishing SOS goal!')

    def exploration_finished_callback(self, msg):

        self.get_logger().info('I heard')

        if msg.data:

            self.publish_goal_point()


def main(args=None):

    rclpy.init(args=args)

    sos_node = SOS_Node()

    rclpy.spin(sos_node)

    # Destroy the node explicitly

    # (optional - otherwise it will be done automatically

    # when the garbage collector destroys the node object)

    sos_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()


 