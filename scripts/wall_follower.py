#!/usr/bin/env python3
""" This script publishes ROS messages to cmd_vel, telling it to find a wall and follow it """
import rospy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import sys
PI = 3.141592653589793238462643383279
SQRT2 = 1.41421356237
class WallFollower(object):
    def __init__(self, node: str, publisher: str, follow_dist: float):
        """Initializers Wall Follower class

        Args:
            node (str): name of the node 
            publisher (str): name of the publishder
            follow_dist (float): distance to follow the wall
        """
        rospy.init_node(node)
        self.publisher = rospy.Publisher(publisher, Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback)
        self.follow_dist = follow_dist
    
    
    def callback(self, msg: LaserScan):
        """Callback in response to laser scans

        Args:
            msg (LaserScan): Laser Scan msg
        """
        ranges = msg.ranges 
        num_ranges = len(ranges)
        interval = num_ranges // 8 # number of indexes that correspond to 460/8 = 45 degrees 
        dist = self.follow_dist

        # Calculating direction regions
        front = min(ranges[interval * -1:] + ranges[:interval])
        front_right = min(ranges[6 * num_ranges // 8 : 7 * num_ranges // 8])
        front_left = min(ranges[num_ranges // 8 : 2 * num_ranges // 8])
        
        twist = Twist()

        # If nothing is in follow range 
        if (front > dist and front_left > dist and front_right > dist):
            print("searching")
            twist.linear.x = 0.2
            twist.angular.z = -0.03
        # Elif the wall is only on our right side 
        elif (front > dist and front_left > dist and front_right < dist):
            twist.linear.x = 0.2
            if (dist - front_right > 0.2):
                twist.linear.x = 0
                twist.angular.z = -0.2
            print("follow wall, correction: ", twist.angular.z)
        # Elif we didn't match the above cases, we need to turn left
        elif (front < dist or (front < dist and (front_left < dist or front_right < dist))):
            print("turn left")
            # twist.linear.x = 0.1
            twist.angular.z = 0.2
            
        self.publisher.publish(twist)

    def shutdown(self):
        """Shutdown callback to stop all movement
        """
        rospy.Publisher("/cmd_vel", Twist, queue_size=10).publish(Twist())

    def run(self):
        """Run loop
        """
        rospy.on_shutdown(self.shutdown)
        self.publisher.publish(Twist())
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower("Square", "/cmd_vel", 0.5)
    node.run()

