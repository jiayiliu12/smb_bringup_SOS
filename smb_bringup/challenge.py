#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool


from geometry_msgs.msg import PointStamped


class SOS_Node(Node):

    def __init__(self):
        super().__init__('sos_node')

        self.publisher_ = self.create_publisher(PointStamped, '/way_point', 10)

        self.subscription_exploration = self.create_subscription(
            Bool,
            '/exploration_finish',
            self.exploration_finished_callback,
            10)
        
        self.subscription_tare_waypoint = self.create_subscription(
            PointStamped,
            '/tare_waypoint',
            self.tare_waypoint_callback,
            10)
        
        self.get_logger().info('Challenge node started!')
        
        self.finished = False

        self.timer = self.create_timer(5*60.0, self.timer_callback)
        # self.subscription_exploration  # prevent unused variable warning

    def publish_goal_point(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing waypoint!')

    def exploration_finished_callback(self, msg):
        if msg.data:
            self.get_logger().info('Exploration finished (Actually finished)!')
            self.finished = True
        if self.finished:
            msg = PointStamped()
            # msg.point.x = 3.0
            # msg.point.y = 4.0
            msg.header.frame_id = "map"
            self.publish_goal_point(msg)

    def tare_waypoint_callback(self, msg):
        if not self.finished:
            self.publish_goal_point(msg)
            self.get_logger().info('Tare callback: forwarding tare waypoint!')

    def timer_callback(self):
        self.get_logger().info('Exploration finished (Time Limit)!')
        self.finished = True
        self.timer.cancel()



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


 