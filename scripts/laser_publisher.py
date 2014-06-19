#! /usr/bin/env python
"""
@author: Sammy Pfeiffer
"""
# ROS related imports
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import rosbag
import rospkg

# System related imports
import csv

'''
A)right _ angular _ speed ;( rad / s );Ts = 0.02sec
B)left _ angular _ speed ;( rad / s );Ts = 0.02sec
C ) polar _ laser _ data;[mm;deg rees ];Ts = 0.4sec
'''
TIME_OFFSET_ANGULAR_SPEED = 0.02
TIME_OFFSET_POLAR_LASER_DATA = 0.4

def load_csv_data(filename, scale):
    """Given a file read it's rows
    and return an array of arrays with the corresponding lectures
    scale is the number over which to divide the elements from every row"""
    datas = []
    f = open(filename, 'rb') # opens the csv file
    try:
        reader = csv.reader(f)  # creates the reader object
        for row in reader:   # iterates the rows of the file in orders
            float_row = []
            for elem in row:
                float_row.append(float(elem) / scale) # Converting from string to float and giving it meters as unit
            datas.append(float_row)
    finally:
        f.close()      # closing
    return datas

def create_laser_msg(range_data_array):
    ls = LaserScan()
    ls.angle_increment = 0.006283185307179586 # 0.36 deg
    ls.angle_max = 2.0943951023931953 # 120.0 deg
    ls.angle_min = -2.0943951023931953 # -120.0 deg
    ls.range_max = 4.0
    ls.range_min = 0.02
    ls.scan_time = 0.001 # No idea
    ls.time_increment = 1.73611115315e-05 # No idea, took from http://comments.gmane.org/gmane.science.robotics.ros.user/5192
    ls.header = Header()
    ls.header.frame_id = 'laser_link'
    ls.ranges = range_data_array
    return ls

if __name__ == '__main__':
    rp = rospkg.RosPack()
    path = rp.get_path('turtlebot_laser_work')
    path += "/scripts/"
    laser_datas = load_csv_data(path + 'polar_laser_data.csv', 1000.0)
    print "laser_data has length: " + str(len(laser_datas))
    odom_left = load_csv_data(path +'left_angular_speed.csv', 1000.0)
    print "odom_left has length: " + str(len(odom_left))
    odom_right = load_csv_data(path + 'right_angular_speed.csv', 1000.0)
    print "odom_right has length: " + str(len(odom_right))
    
    rospy.init_node("load_and_publish")
    rospy.sleep(0.2) # let a bit of time pass after init_node
    initial_time = rospy.Time.now()
    bag = rosbag.Bag('laser_and_odom.bag', 'w')
    
    num_laser_row = 0
    for laser_row in laser_datas:
        ls_msg = create_laser_msg(laser_row)
        ls_msg.header.stamp = initial_time + rospy.Duration(num_laser_row * TIME_OFFSET_POLAR_LASER_DATA)
        print ls_msg.header.stamp
        print type(ls_msg.header.stamp)
        bag.write('scan', ls_msg, ls_msg.header.stamp)
        num_laser_row += 1
    
    num_odom_row = 0
    for o_l, o_r in zip(odom_left, odom_right):
        v = Vector3(o_l[1], o_r[1], 0.0)
        bag.write('wheels_vel', v, initial_time + rospy.Duration(num_odom_row * TIME_OFFSET_ANGULAR_SPEED))
        num_odom_row += 1
    
    bag.close()
    
    
