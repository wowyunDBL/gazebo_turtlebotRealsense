#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print 'length: %s' % (len(msg.ranges))
	# values at 0 degree
    print msg.ranges[0]
    # values at 90 degree
    print msg.ranges[90]
    # values at 180 degree
    print msg.ranges[234]
    
    print msg.angle_min
    print msg.angle_max
rospy.init_node('scan_values')
sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, callback)
rospy.spin()
