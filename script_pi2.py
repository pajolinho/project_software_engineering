#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def move_to_goal(x, y, z):
    # Initialize the ROS node
    rospy.init_node('move_to_goal_node', anonymous=True)

    # Initialize the Action Client for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the Action Server to become available
    rospy.loginfo("Waiting for move_base Action Server...")
    client.wait_for_server()

    # Create the MoveBaseGoal
    goal = MoveBaseGoal()

    # Define the target in space (position and orientation)
    goal.target_pose.header.frame_id = "map"  # Use "map" as the default coordinate system in Gazebo
    goal.target_pose.header.stamp = rospy.Time.now()

    # Target position (x, y) - z is 0 for 2D navigation
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0

    # Target orientation (rotation about the Z-axis)
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = 1.0

    # Send the goal to move_base
    rospy.loginfo(f"Moving to target: x={x}, y={y}, z={z}")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check if the goal was achieved
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Target reached!")
    else:
        rospy.loginfo("Failed to reach the target.")

if __name__ == "__main__":
    try:
        # Example coordinates that the user might input
        move_to_goal(2.0, 3.0, 0.0)
    except rospy.ROSInterruptException:
        pass

