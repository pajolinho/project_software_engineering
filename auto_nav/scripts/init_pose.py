#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import argparse
import yaml
import os

class Velocity:
    def __init__(self, angular, linear):
        self.angular = angular
        self.linear = linear

class Linear:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class Angular:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

def update_twist(velocity, publisher):
    twist = Twist()

    twist.linear.x = velocity.linear.x
    twist.linear.y = velocity.linear.y
    twist.linear.z = velocity.linear.z

    twist.angular.x = velocity.angular.x
    twist.angular.y = velocity.angular.y
    twist.angular.z = velocity.angular.z

    publisher.publish(twist)
    print("Velocity changed!")

def are_odoms_equal(odom1, odom2, tolerance=0.2):
    x_diff = abs(odom1.pose.pose.orientation.x - odom2.pose.pose.orientation.x)
    y_diff = abs(odom1.pose.pose.orientation.y - odom2.pose.pose.orientation.y)
    z_diff = abs(odom1.pose.pose.orientation.z - odom2.pose.pose.orientation.z)

    return x_diff <= tolerance and y_diff <= tolerance and z_diff <= tolerance

def main():
    parser = argparse.ArgumentParser(description='Initialize position and perform initial turn')
    parser.add_argument('initial_position', type=argparse.FileType("r"), help='Path to YAML file that contains the initial position!')

    args = parser.parse_args()
    yaml_file = args.initial_position

    # Load values from YAML file
    yaml_data = yaml.safe_load(yaml_file)

    # Node initialization
    rospy.init_node('init_pose')
    initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Construct message
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.frame_id = "map"

    init_msg.pose.pose.position.x = yaml_data['position']['x']
    init_msg.pose.pose.position.y = yaml_data['position']['y']
    init_msg.pose.pose.orientation.x = yaml_data['orientation']['x']
    init_msg.pose.pose.orientation.y = yaml_data['orientation']['y']
    init_msg.pose.pose.orientation.z = yaml_data['orientation']['z']
    init_msg.pose.pose.orientation.w = yaml_data['orientation']['w']
    init_msg.pose.covariance = yaml_data['covariance']

    # Delay
    rospy.sleep(1)

    # Publish message
    rospy.loginfo("Setting initial pose")
    initialpose_pub.publish(init_msg)
    rospy.loginfo("Initial pose is set")

    current_loc = rospy.wait_for_message('/odom', Odometry)
    rospy.sleep(1)

    turned = False

    current_script_directory = os.path.dirname(os.path.abspath(__file__))

    # Navigate to the parent directory (one level above)
    parent_directory = os.path.abspath(os.path.join(current_script_directory, '..'))

    # Construct the absolute path to config.yaml
    config_path = os.path.join(parent_directory, 'init_pose_config.yaml')

    # Open the YAML file and load its content
    with open(config_path, 'r') as yaml_file:
        config_data = yaml.safe_load(yaml_file)
    

    def turn():
        while not turned:
            rospy.sleep(config_data["vel_sleep_duration"])
            ang = Angular(z=config_data["vel_speed"])
            lin = Linear()
            vel = Velocity(angular=ang, linear=lin)
            update_twist(vel, cmd_vel_pub)
        vel.angular.z=0
        update_twist(vel, cmd_vel_pub)
        print("Initial turn done!")

    my_thread = threading.Thread(target=turn)
    my_thread.start()

    rospy.sleep(3)

    while not turned:
        odom2 = rospy.wait_for_message('/odom', Odometry)
        if are_odoms_equal(current_loc, odom2, tolerance=config_data["odom_equal_tolerance"]):
            turned = True

    print("Initial pose done!")

if __name__ == "__main__":
    main()
