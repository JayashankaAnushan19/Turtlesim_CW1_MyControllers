#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sin, cos
import sys, select, termios, tty

# Global boundary variables
leftBoundary = None
rightBoundary = None
bottomBoundary = None
upperBoundary = None
speed = None

# Save the existing terminal settings
settings = termios.tcgetattr(sys.stdin)

# Take the keyboard inputs
def getKey():
    tty.setraw(sys.stdin.fileno())  # Set terminal to raw mode
    key = None

    # Take input every 0.1 seconds
    if select.select([sys.stdin], [], [], 0.1)[0]:
        key = sys.stdin.read(1)  # Read one character
        if key == '\x1b':  # Check for arrow key sequence
            key += sys.stdin.read(2)  # Read additional two characters for arrow keys

    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Check if the new position crosses the boundaries
def checkBoundaryCrossed(direction):
    if direction == 'w':
        new_x = current_pose.x + (speed * cos(current_pose.theta))
        new_y = current_pose.y + (speed * sin(current_pose.theta))
    elif direction == 's':
        new_x = current_pose.x - (speed * cos(current_pose.theta))
        new_y = current_pose.y - (speed * sin(current_pose.theta))
    elif direction == 'a' or direction == 'd':  # Left and Right only change orientation
        new_x = current_pose.x
        new_y = current_pose.y
    else:
        return False

    # Log current and expected position for debugging
    rospy.loginfo('-----------------------------')
    rospy.loginfo(f'Current X: {current_pose.x}, Current Y: {current_pose.y}')
    rospy.loginfo(f'Expected X: {new_x}, Expected Y: {new_y}')
    rospy.loginfo('-----------------------------')

    # Check if new position is out of bounds
    if new_x <= leftBoundary or new_x >= rightBoundary or new_y <= bottomBoundary or new_y >= upperBoundary:
        rospy.loginfo('Boundary hit! Cannot move the bot.')
        return True
    return False

# Update current position from the turtle's pose
def getCurrentPosition(data):
    global current_pose
    current_pose = data
    current_pose.x = round(current_pose.x, 4)
    current_pose.y = round(current_pose.y, 4)

# Main function for controlling the turtle
def main():
    rospy.init_node('my_teleop_cmds')  # Initialize ROS node
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # Publisher for movement commands

    # Subscriber to get the current position of the turtle
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, getCurrentPosition)
    rate = rospy.Rate(10)  # 10 Hz rate for ROS loop
    
    # Define initial speed
    global speed
    speed = 1.0
    move_cmd = Twist()

    try:
        rospy.loginfo("--- Set Boundaries ---")

        # Get boundary inputs from user
        global leftBoundary
        leftBoundary = int(input("Enter the left boundary (between 0-11): "))
        while leftBoundary < 0 or leftBoundary > 11:
            rospy.loginfo("Invalid input. Please re-enter the left boundary (0-11).")
            leftBoundary = int(input("Enter the left boundary (between 0-11): "))

        global rightBoundary
        rightBoundary = int(input("Enter the right boundary (larger than left and smaller than 11): "))
        while leftBoundary >= rightBoundary or rightBoundary > 11:
            rospy.loginfo("Invalid input. Please re-enter the right boundary.")
            rightBoundary = int(input("Enter the right boundary: "))

        global bottomBoundary
        bottomBoundary = int(input("Enter the bottom boundary (between 0-11): "))
        while bottomBoundary < 0 or bottomBoundary > 11:
            rospy.loginfo("Invalid input. Please re-enter the bottom boundary (0-11).")
            bottomBoundary = int(input("Enter the bottom boundary (between 0-11): "))

        global upperBoundary
        upperBoundary = int(input("Enter the upper boundary (larger than bottom and smaller than 11): "))
        while bottomBoundary >= upperBoundary or upperBoundary > 11:
            rospy.loginfo("Invalid input. Please re-enter the upper boundary.")
            upperBoundary = int(input("Enter the upper boundary: "))

        rospy.loginfo("--- Control Your Turtle! ---")
        rospy.loginfo("Use 'WASD' or Arrow keys to move the bot.")
        rospy.loginfo("Press 'Q' or 'P' to quit.")
        rospy.loginfo("Press 'R' to reset speed.")
        rospy.loginfo("Press '+' to increase speed, '-' to decrease speed.")

        # Start the control loop
        while not rospy.is_shutdown():
            key = getKey()
            if key:
                # Move up (W or Up Arrow)
                if key in ['w', '\x1b[A'] and not checkBoundaryCrossed('w'):
                    move_cmd.linear.x = speed
                    move_cmd.angular.z = 0
                # Move down (S or Down Arrow)
                elif key in ['s', '\x1b[B'] and not checkBoundaryCrossed('s'):
                    move_cmd.linear.x = -speed
                    move_cmd.angular.z = 0
                # Turn left (A or Left Arrow)
                elif key in ['a', '\x1b[D'] and not checkBoundaryCrossed('a'):
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = speed
                # Turn right (D or Right Arrow)
                elif key in ['d', '\x1b[C'] and not checkBoundaryCrossed('d'):
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = -speed
                # Increase speed
                elif key == '+':
                    speed += 0.2
                    rospy.loginfo(f"Speed increased to {speed:.2f}")
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0
                # Decrease speed
                elif key == '-':
                    speed = max(0, speed - 0.2)
                    rospy.loginfo(f"Speed decreased to {speed:.2f}")
                # Reset speed
                elif key == 'r':
                    speed = 1.0
                    rospy.loginfo("Speed reset to 1.0")
                # Quit
                elif key in ['q', 'p']:
                    rospy.loginfo("Exiting...")
                    break
                # Unrecognized command
                else:
                    if key not in ['w', 'a', 's', 'd', 'r', 'q', 'p', '+', '-', '\x1b[A', '\x1b[B', '\x1b[D', '\x1b[C']:
                        rospy.loginfo(f"Unrecognized command: {key}")
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0

                # Publish the movement command
                pub.publish(move_cmd)

    except Exception as e:
        rospy.loginfo(f"Error: {e}")

    finally:
        # Stop the turtle when exiting
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
