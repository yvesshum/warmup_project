#!/usr/bin/env python3
""" This script publishes ROS messages to cmd_vel, telling it to travel to the closet object (assuming its a wall), and follow it """
import rospy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import sys
import math
PI = 3.141592653589793238462643383279
SQRT2 = 1.41421356237
class PersonFollower(object):
    def __init__(self, node: str, publisher: str, follow_dist: float):
        """Inits a PersonFollower obj

        Args:
            node (str): Node name   
            publisher (str): Publisher name
            follow_dist (float): distance to follow
        """
        rospy.init_node(node)
        self.publisher = rospy.Publisher(publisher, Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback)
        self.follow_dist = follow_dist
    
    def callback(self, msg: LaserScan):
        """Callback in response to laser scan values

        Args:
            msg (LaserScan): LaserScan message
        """
        ranges = msg.ranges 

        twist = Twist()
        closest_range = min(ranges)
        closest_range_index = ranges.index(closest_range)

        # Catching the isolated case
        if (closest_range == math.inf):
            print("no closest object found")
            self.publisher.publish(twist) #Do nothing
            return

        # If we're close enough to the object
        if (closest_range <= self.follow_dist):
            # Try to face closest_object 
            print("close to closest object")
            if (closest_range_index > 0 and closest_range_index < 177):
                twist.angular.z = 0.2
            elif (closest_range_index > 183 and closest_range_index < 360):
                twist.angular.z = -0.2
        # Else, we have to move closer
        else:
            print("moving to closest object")
            if (closest_range_index > 0 and closest_range_index < 177):
                twist.angular.z = 0.2
                twist.linear.x = 0.1
            elif (closest_range_index > 183 and closest_range_index < 360):
                twist.angular.z = -0.2
                twist.linear.x = 0.1
            else:
                twist.linear.x = 0.3
            
        self.publisher.publish(twist)

    def shutdown(self):
        """Shutdown callback
        """
        rospy.Publisher("/cmd_vel", Twist, queue_size=10).publish(Twist())

    def run(self):
        """Run loop
        """
        rospy.on_shutdown(self.shutdown)
        self.publisher.publish(Twist())
        rospy.spin()

if __name__ == '__main__':
    node = PersonFollower("Square", "/cmd_vel", 0.5)
    node.run()

