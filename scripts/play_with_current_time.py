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

TIME_OFFSET_ANGULAR_SPEED = 0.02
TIME_OFFSET_POLAR_LASER_DATA = 0.4

def yielder(list):
    """An object to yield one list of the list of lists at a time"""
    counter = 0
    while counter < len(list):
        for item in list:
            yield item
            counter += 1

if __name__ == '__main__':
    rospy.init_node("load_and_publish")
    rospy.sleep(0.2)
    rospy.loginfo("Initializing publishers")
    laser_pub = rospy.Publisher('/scan', LaserScan)
    wheels_pub = rospy.Publisher('/wheels_vel', Vector3)
    rp = rospkg.RosPack()
    path = rp.get_path('turtlebot_laser_work')
    bag = rosbag.Bag(path + '/scripts/laser_and_odom.bag', 'r')
    laser_msgs = []  
    wheels_msgs = []
    # Prepare getting all messages
    for topic, msg, t in bag.read_messages():
        if topic == "scan":
            laser_msgs.append(msg)
        if topic == "wheels_vel":
            wheels_msgs.append(msg)
    bag.close()
    
    rospy.loginfo("Publishing data")
    num_msg = 0
    laser_yielder = yielder(laser_msgs)
    vels_yielder = yielder(wheels_msgs)
    # Publish data at the necessary rates with current timestamp
    while not rospy.is_shutdown():
        for iteration in range(int(TIME_OFFSET_POLAR_LASER_DATA / TIME_OFFSET_ANGULAR_SPEED)): # we publish 20 times
            curr_vel = vels_yielder.next()
            wheels_pub.publish(curr_vel)
            rospy.sleep(TIME_OFFSET_ANGULAR_SPEED)
        try:
            curr_laser = laser_yielder.next()
        except StopIteration: # laser data finishes before odometry data
            break
        curr_laser.header.stamp = rospy.Time.now()
        laser_pub.publish(curr_laser)

        
    rospy.loginfo("Published all data.")
    
    