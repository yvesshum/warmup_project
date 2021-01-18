#!/usr/bin/env python3
""" This script publishes ROS messages to cmd_vel, telling it to run in a square"""
import rospy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from std_msgs.msg import Header

PI = 3.141592653589793238462643383279

class SquareNode(object):
    def __init__(self, node: str, publisher: str, speed: int, edge_len: int):
        rospy.init_node(node)
        self.publisher = rospy.Publisher(publisher, Twist, queue_size=10)
        self.speed = speed
        self.edge_len = edge_len

    def rotate90(self):
        omega = 30 * 2 * PI / 360
        target_angle = PI / 2 # 90 degrees 
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = omega 

        t_start = rospy.Time.now().to_sec()
        angle = 0
        while (angle < target_angle):
            self.publisher.publish(twist)
            angle = omega * (rospy.Time.now().to_sec() - t_start)

        self.publisher.publish(Twist())
        return
    
    def run(self):
        rospy.sleep(1)
        for i in range(4):
            print(i)
            twist = Twist()
            twist.linear.x = self.speed
            t_start = rospy.Time.now().to_sec()
            traversed_distance = 0
            while (traversed_distance < self.edge_len):
                self.publisher.publish(twist)
                traversed_distance = self.speed * (rospy.Time.now().to_sec() - t_start)
            self.publisher.publish(Twist())
            rospy.sleep(1)
            self.rotate90()
            rospy.sleep(1)
        return

if __name__ == '__main__':
    node = SquareNode("Square", "/cmd_vel", 0.1, 1.5)
    node.run()