#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool
from math import cos, inf
import sys

# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.
        self.ackpub = rospy.Publisher('/break', AckermannDriveStamped, queue_size=10)
        self.boolpub = rospy.Publisher('/break_bool', Bool, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rate = rospy.Rate(10) # 10hz



    def odom_callback(self, odom_msg: Odometry):
        # TODO: update current speed
        self.speed = odom_msg.twist.linear.x

    def scan_callback(self, scan_msg: LaserScan):
        # TODO: calculate TTC
        angle = scan_msg.angle_min
        ttc = inf

        for dist in scan_msg.ranges:
            angle = angle + scan_msg.angle_increment
            tempttc = dist/max(-self.speed * cos(angle),sys.float_info.min)
            ttc = min(ttc, tempttc)
        # TODO: publish brake message and publish controller bool

        self.ackpub.publish(AckermannDriveStamped(header = scan_msg.header, drive = AckermannDrive(0,0,0,0,0)))
        self.boolpub.publish(Bool(True))
        


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
