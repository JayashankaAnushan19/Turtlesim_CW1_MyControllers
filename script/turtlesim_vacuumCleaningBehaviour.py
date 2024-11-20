#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import math
import sys, termios

# Save the existing terminal settings
settings = termios.tcgetattr(sys.stdin)

# Parameters
speed = 0.5  # Linear speed
rad90 = (90 * math.pi) / 180  # 90 degrees in radians

# Define the boundary limits for the turtlesim window
left_boundary = 0
right_boundary = 11
bottom_boundary = 0
upper_boundary = 11

# Keep the positions in global
current_pose = Pose()  # To store the current position of the bot
move_cmd = Twist()  # To store movement commands

# Function to move the bot to the starting point
def move_bot_to_start_point():
    rospy.wait_for_service('/turtle1/teleport_absolute')
    move_bot = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    # Move the turtle to (0.5, 0.5) with 0 orientation to start the process
    move_bot(left_boundary + 0.5, bottom_boundary + 0.5, 0)

# Callback to update the current position
def get_current_position(data):
    global current_pose
    current_pose = data

# Function to check if the bot is about to hit a boundary
def check_boundary_crossed():
    # Calculate the new position after moving one step forward
    new_x = current_pose.x + (speed * math.cos(current_pose.theta))
    new_y = current_pose.y + (speed * math.sin(current_pose.theta))

    rospy.loginfo('-----------------------------')
    rospy.loginfo(f'Currect X : {current_pose.x}')
    rospy.loginfo(f'Current Y : {current_pose.y}')
    rospy.loginfo(f'Expected X : {new_x}')
    rospy.loginfo(f'Expected Y : {new_y}')
    rospy.loginfo('-----------------------------')

    # Check if the new position is outside the boundaries
    if new_x < left_boundary or new_x > right_boundary or new_y < bottom_boundary or new_y > upper_boundary:
        return True
    return False

# Function to move the robot
def move_robot(linear_x, angular_z):
    move_cmd.linear.x = linear_x
    move_cmd.angular.z = angular_z

# Main function
def main():
    rospy.init_node('vacuum_cleaner_bot')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, get_current_position)

    move_bot_to_start_point()  # Move the bot to the starting point
    rate = rospy.Rate(20)  # Loop rate of 10 Hz

    try:
        direction = 1  # Start by moving to the right

        while not rospy.is_shutdown():

            move_robot(speed, 0)
            pub.publish(move_cmd)
            rate.sleep()  # Maintain the loop rate

            # Move forward until a boundary is about to be crossed
            while not rospy.is_shutdown() and not check_boundary_crossed():
                move_robot(speed, 0)
                pub.publish(move_cmd)
                rate.sleep()  # Maintain the loop rate

            # If a boundary is about to be crossed, stop the bot
            move_robot(0, 0)
            pub.publish(move_cmd)

            if direction > 0:
                # Move up to the next row
                rospy.loginfo("------------Turning left - anticlockwise.")
                move_robot(0, rad90)  # Turn 90 degrees anticlockwise
                pub.publish(move_cmd)
                rospy.sleep(2)
                if check_boundary_crossed():
                    break
                else:
                    move_robot(speed, 0)  # Move forward
                    pub.publish(move_cmd)
                    rospy.sleep(1)
                move_robot(0, rad90)  # Turn 90 degrees anticlockwise
                pub.publish(move_cmd)
                rospy.sleep(1)
                move_up = False
                direction *= -1  # Change the horizontal direction

            else:
                # Turn to change direction
                rospy.loginfo("------------Turning right - clockwise..")
                move_robot(0, -rad90)  # Turn 90 degrees clockwise
                pub.publish(move_cmd)
                rospy.sleep(1)
                if check_boundary_crossed():
                    break
                else:
                    move_robot(speed, 0)  # Move forward
                    pub.publish(move_cmd)
                    rospy.sleep(1)
                move_robot(0, -rad90)  # Turn 90 degrees clockwise
                pub.publish(move_cmd)
                rospy.sleep(1)
                direction = 1
                # move_up = True  # Set the flag to move up on the next iteration

            rate.sleep()  # Maintain the loop rate

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down the bot..")

    finally:
        # Stop the turtle when exiting
        move_robot(0, 0)
        pub.publish(move_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
