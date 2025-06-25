#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool


from geometry_msgs.msg import PointStamped


class SOS_Node(Node):

    def __init__(self):
        super().__init__('sos_node')

        self.publisher_ = self.create_publisher(PointStamped, '/waypoint', 10)

        self.subscription_exploration = self.create_subscription(
            Bool,
            '/exploration_finished',
            self.exploration_finished_callback,
            10)
        
        self.subscription_tare_waypoint = self.create_subscription(
            PointStamped,
            '/tare_waypoint',
            self.tare_waypoint_callback,
            10)
        
        self.finished = False
        
        # self.subscription_exploration  # prevent unused variable warning

    def publish_goal_point(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing waypoint!')

    def exploration_finished_callback(self, msg):
        if msg.data:
            self.get_logger().info('Exploration finished!!!!!!!')
            self.finished = True
            msg = PointStamped()
            msg.header.frame_id = "map"
            self.publish_goal_point(msg)

    def tare_waypoint_callback(self, msg):
        if not self.finished:
            self.publish_goal_point(msg)
            self.get_logger().info('Tare callback: forwarding tare waypoint!')



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


 