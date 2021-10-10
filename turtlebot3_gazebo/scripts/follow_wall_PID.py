#! /usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from tf import transformations
#from datetime import datetime
from visualization_msgs.msg import Marker


# Util imports
import random
import math
import time
import csv


hz = 20                     # Cycle Frequency
loop_index = 0              # Number of sampling cycles
loop_index_outer_corner = 0 # Loop index when the outer corner is detected
loop_index_inner_corner = 0 # Loop index when the inner corner is detected
inf = 5                     # Limit to Laser sensor range in meters, all distances above this value are 
                            #      considered out of sensor range
wall_dist = 1             # Distance desired from the wall
max_speed = 0.1             # Maximum speed of the robot on meters/seconds
p = 10                      # Proportional constant for controller  
d = 0                       # Derivative constant for controller 
angle = 1                   # Proportional constant for angle controller (just simple P controller)
direction = -1              # 1 for wall on the left side of the robot (-1 for the right side)
e = 0                       # Diference between current wall measurements and previous one
angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0                # Minimum measured distance

# global visualization tool
marker = Marker()
marker.header.frame_id = "odom"
marker.type = Marker.SPHERE
marker.lifetime = rospy.Duration(5.0)
marker.action = Marker.ADD
marker.id = 0
marker.scale.x = 0.2
marker.scale.y = 0.2
marker.scale.z = 0.2
marker.pose.orientation.w = 1.0
marker.color.a = 1.0
marker.color.g = 1.0

global_x_ = 0
global_y_ = 0

# Time when the last outer corner; direction and inner corner were detected or changed.
last_outer_corner_detection_time = time.time()
last_change_direction_time = time.time()
last_inner_corner_detection_time = time.time()
rotating = 0 
pub_ = None
pubMarker_ = None
msg = Twist()
# Sensor regions
regions_ = {
        'bright': 0,
        'right': 0,
        'fright': 0,
        'front': 0,
        'left': 0,
}
last_kinds_of_wall=[0, 0, 0, 0, 0]
index = 0

state_outer_inner=[0, 0, 0, 0]
index_state_outer_inner = 0

bool_outer_corner = 0
bool_inner_corner =0

last_vel = [random.uniform(0.01,0.01),  random.uniform(-0.3,0.3)]
wall_found =0

#Robot state machines
state_ = 0
state_dict_ = {
    0: 'random wandering',
    1: 'following wall',
    2: 'rotating'
}

def clbk_laser(msg):
    """
    Read sensor messagens, and determine distance to each region. 
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global regions_, e, angle_min, dist_front, diff_e, direction, bool_outer_corner, bool_inner_corner, index, last_kinds_of_wall, pubMarker_, marker, global_x_, global_y_
    size = len(msg.ranges)
    min_index = 200 #size*(direction+1)/4
    max_index = 350 #size*(direction+3)/4
    
    # Determine values for PD control of distance and P control of angle
    for i in range(min_index, max_index):
        if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.01:
            min_index = i
    if i > 180 :
        angle_min = (min_index-359)*msg.angle_increment   # the wall on the right side is negative
    else:
        angle_min = min_index*msg.angle_increment   # the wall on the left side is positive
    dist_min = msg.ranges[min_index]
    dist_front = msg.ranges[0]
    diff_e = min((dist_min - wall_dist) - e, 100)
    e = min(dist_min - wall_dist, 100)
    '''
    draw the point in rviz ###################################################
    '''
    marker.pose.position.x = math.cos(angle_min) * dist_min + global_x_
    marker.pose.position.y = math.sin(angle_min) * dist_min + global_y_
    marker.pose.position.z = 0
    pubMarker_.publish(marker)
    print 'clbk %f %f' % (math.cos(angle_min) * dist_min + global_x_, math.sin(angle_min) * dist_min +  global_x_)
    '''
    fMin = open('/home/anny/wall_follow_with_PID/'+str(p)+'/result_min_dist.csv', 'a+')
        with fMin:
            writer = csv.writer(fMin)
            writer.writerow([dt, dist_min])
    '''
    # Determination of minimum distances in each region
    regions_ = {
        'bright':  min(min(msg.ranges[234:269]), inf),
        'right': min(min(msg.ranges[270:307]), inf),
        'fright':  min(min(msg.ranges[308:341]), inf),
        'front':  min(min(min(msg.ranges[0:17]),min(msg.ranges[342:359])), inf),
        'fleft':   min(min(msg.ranges[18:53]), inf),
        'left':   min(min(msg.ranges[54:89]), inf),
        'bleft':   min(min(msg.ranges[90:126]), inf),
    }
    #rospy.loginfo(regions_)

    # Detection of Outer and Inner corner
    bool_outer_corner = is_outer_corner()
    bool_inner_corner = is_inner_corner()
    if bool_outer_corner == 0 and bool_inner_corner == 0:
        last_kinds_of_wall[index]=0
    
    # Indexing for last five pattern detection
    # This is latter used for low pass filtering of the patterns
    index = index + 1 #5 samples recorded to asses if we are at the corner or not
    if index == len(last_kinds_of_wall):
        index = 0
        
    take_action()

def change_state(state):
    """
    Update machine state
    """
    global state_, state_dict_
    if state is not state_:
        #print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    """
    Change state for the machine states in accordance with the active and inactive regions of the sensor.
            State 0 No wall found - all regions infinite - Random Wandering
            State 1 Wall found - Following Wall
            State 2 Pattern sequence reached - Rotating
    """
    global regions_, index, last_kinds_of_wall, index_state_outer_inner, state_outer_inner, loop_index, loop_index_outer_corner
    
    global wall_dist, max_speed, direction, p, d, angle, dist_min, wall_found, rotating, bool_outer_corner, bool_inner_corner

    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    # Patterns for rotating
    rotate_sequence_V1 = ['I', 'C', 'C', 'C']
    rotate_sequence_V2 = [0, 'C', 'C', 'C']
    rotate_sequence_W = ['I', 'C', 'I', 'C']

    if rotating == 1:
        state_description = 'case 2 - rotating'
        change_state(2)
        if(regions['left'] < wall_dist or regions['right'] < wall_dist):
            rotating = 0
    elif regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['bright'] == inf and regions['fleft'] == inf and regions['left'] == inf and regions['bleft'] == inf:
        state_description = 'case 0 - random wandering'
        change_state(0)
    elif (loop_index == loop_index_outer_corner) and (rotate_sequence_V1 == state_outer_inner or rotate_sequence_V2 == state_outer_inner or rotate_sequence_W == state_outer_inner):
        state_description = 'case 2 - rotating'
        change_direction()
        state_outer_inner = [ 0, 0,  0, 'C']
        change_state(2)
    else:
        state_description = 'case 1 - following wall'
        change_state(1)
    print '%s' % (state_description)

def random_wandering():
    """
    This function defines the linear.x and angular.z velocities for the random wandering of the robot.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> [0.1, 0.3]
                    msg.angular.z -> [-1, 1]
    """
    global direction, last_vel
    msg = Twist()
    msg.linear.x = max(min( last_vel[0] + random.uniform(-0.01,0.01),0.3),0.1)
    msg.angular.z= max(min( last_vel[1] + random.uniform(-0.1,0.1),1),-1)
    if msg.angular.z == 1 or msg.angular.z == -1:
        msg.angular.z = 0
    last_vel[0] = msg.linear.x
    last_vel[1] = msg.angular.z
    return msg

def following_wall():
    """
    PD control for the wall following state. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """
    global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
    msg = Twist()
    if dist_front < wall_dist:
        msg.linear.x = 0
    elif dist_front < wall_dist*2:
        msg.linear.x = 0.5*max_speed
    elif abs(angle_min) > 1.75:
        msg.linear.x = 0.4*max_speed
    else:
        msg.linear.x = max_speed
    msg.angular.z = max(min(direction*(p*e+d*diff_e) + angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)

    #print 'Turn Left angular z, linear x %f - %f  ' % (msg.angular.z, msg.linear.x)
    return msg

def change_direction():
    """
    Toggle direction in which the robot will follow the wall
        1 for wall on the left side of the robot and -1 for the right side
    """
    global direction, last_change_direction, rotating
    print 'Change direction!'
    elapsed_time = time.time() - last_change_direction_time # Elapsed time since last change direction
    if elapsed_time >= 20:
        last_change_direction = time.time()
        direction = -direction # Wall in the other side now
        rotating = 1

def rotating():
    """
    Rotation movement of the robot. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0m/s
                    msg.angular.z -> -2 or +2 rad/s
    """
    global direction
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = direction*2
    return msg


def is_outer_corner():
    """
    Assessment of outer corner in the wall. 
    If all the regions except for one of the back regions are infinite then we are in the presence of a possible corner.
    If all the elements in last_kinds_of_wall are 'C' and the last time a real corner was detected is superior or equal to 30 seconds:
        To state_outer_inner a 'C' is appended and 
        The time is restart.
    Returns:
            bool_outer_corner: 0 if it is not a outer corner; 1 if it is a outer corner
    """
    global regions_, last_kinds_of_wall, last_outer_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index, loop_index_outer_corner
    regions = regions_
    bool_outer_corner = 0
    if (regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['bright'] < inf  and regions['left'] == inf and regions['bleft'] == inf and regions['fleft'] == inf) or (regions['bleft'] < inf and regions['fleft'] == inf and regions['front'] == inf and regions['left'] == inf and regions['right'] == inf and regions['bright'] == inf and regions['fright'] == inf):
        bool_outer_corner = 1 # It is a corner
        last_kinds_of_wall[index]='C'
        elapsed_time = time.time() - last_outer_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('C') == len(last_kinds_of_wall) and elapsed_time >= 30:
            last_outer_corner_detection_time = time.time()
            loop_index_outer_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('C')
            print 'It is a outer corner'
    return bool_outer_corner

def is_inner_corner():
    """
    Assessment of inner corner in the wall. 
    If the three front regions are inferior than the wall_dist.
    If all the elements in last_kinds_of_wall are 'I' and the last time a real corner was detected is superior or equal to 20 seconds:
        To state_outer_inner a 'I' is appended and 
        The time is restart.
    Returns:
            bool_inner_corner: 0 if it is not a inner corner; 1 if it is a inner corner
    """
    global regions_, wall_dist, last_kinds_of_wall, last_inner_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index_inner_corner, loop_index
    regions = regions_
    bool_inner_corner = 0
    if regions['fright'] < wall_dist and regions['front'] < wall_dist and regions['fleft'] < wall_dist:
        bool_inner_corner = 1
        last_kinds_of_wall[index]='I'
        elapsed_time = time.time() - last_inner_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('I') == len(last_kinds_of_wall) and elapsed_time >= 20:
            last_inner_corner_detection_time = time.time()
            loop_index_inner_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('I')
            print 'It is a inner corner'
    return bool_inner_corner
count_odom = 0
start_time_odom = 0

def clbk_odom(msgOdom):
	global count_odom, start_time_odom, msg, marker, pubMarker_, angle_min, dist_min, global_x_, global_y_
	marker.header.stamp = rospy.Time.now()
	global_x_ = msgOdom.pose.pose.position.x
	global_y_ = msgOdom.pose.pose.position.y
	'''
	marker.pose.position.x = math.cos(angle_min) * dist_min + msgOdom.pose.pose.position.x
	marker.pose.position.y = math.sin(angle_min) * dist_min + msgOdom.pose.pose.position.y
	marker.pose.position.z = 0

	print '%f %f' % (math.cos(angle_min) * dist_min, math.sin(angle_min) * dist_min)
	pubMarker_.publish(marker)
	'''
	'''
	if count_odom == 0:
		start_time_odom = msgOdom.header.stamp.secs + msgOdom.header.stamp.nsecs*1e-9
	if count_odom%5 == 0:
		dt = msgOdom.header.stamp.secs + msgOdom.header.stamp.nsecs*1e-9 - start_time_odom
		fOdom = open('/home/anny/wall_follow_with_PID/'+str(p)+'/result_odom.csv', 'a+')
		with fOdom:
			writer = csv.writer(fOdom)
			writer.writerow([dt, msgOdom.pose.pose.position.x, msgOdom.pose.pose.position.y])

		fCmd = open('/home/anny/wall_follow_with_PID/'+str(p)+'/result_cmd.csv', 'a+')
		with fCmd:
			writer = csv.writer(fCmd)
			writer.writerow([dt, msg.linear.x, msg.angular.z])
			print 'Turn Left angular z, linear x %f - %f  ' % (msg.angular.z, msg.linear.x)
	count_odom = count_odom + 1
    '''
'''
count_cmd = 0
start_time_cmd = 0
def clbk_cmd(msg):
	global count_cmd, start_time_cmd
	if count == 0:
		start_time_cmd = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
	if count%5 == 0:
		dt = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9 - start_time_cmd
		f = open('/home/anny/wall_follow_with_PID/result_cmd.csv', 'a+')
		with f:
			writer = csv.writer(f)
			writer.writerow([dt, msg.linear.x, msg.angular.z])
	count_cmd = count_cmd + 1
'''
def main():
    global pub_, active_, hz, loop_index, msg, pubMarker_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pubMarker_ = rospy.Publisher('/visualize', Marker, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    print 'Code is running'
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        loop_index = loop_index + 1

        # State Dispatcher
        if state_ == 0:
            msg = random_wandering()
        elif state_ == 1:
            msg = following_wall()
        elif state_ == 2:
            msg = rotating()
        else:
            rospy.logerr('Unknown state!')
        #print 'publish!'
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
