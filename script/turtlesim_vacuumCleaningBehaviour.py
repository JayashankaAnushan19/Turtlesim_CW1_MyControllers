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
    try:
        rospy.wait_for_service('/turtle1/teleport_absolute', timeout=5)
        move_bot = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        move_bot(left_boundary + 0.5, bottom_boundary + 0.5, 0)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        rospy.signal_shutdown("Teleport service unavailable.")


# Callback to update the current position
def get_current_position(data):
    global current_pose
    current_pose = data

def takeTan(newX, newY):
    return math.atan2(newY - current_pose.y, newX - current_pose.x)

# Function to check if the bot is about to hit a boundary
def check_boundary_crossed():
    # Calculate the new position after moving one step forward
    new_x = current_pose.x + (speed * math.cos(current_pose.theta))
    new_y = current_pose.y + (speed * math.sin(current_pose.theta))

    print('-----------------------------')
    print(f'Currect X : {current_pose.x}')
    print(f'Current Y : {current_pose.y}')
    print(f'Current theta : {current_pose.theta}')
    print(f'Expected X : {new_x}')
    print(f'Expected Y : {new_y}')
    print(f'Expected theta : {takeTan(new_x, new_y):}')
    print('-----------------------------')

    # Check if the new position is outside the boundaries
    if new_x < left_boundary or new_x > right_boundary or new_y < bottom_boundary or new_y > upper_boundary:
        return True
    else:
        return False

# Function to move the robot
def move_robot(linear_x, angular_z):
    move_cmd.linear.x = linear_x
    move_cmd.angular.z = angular_z

def turn_to_angle(angle_offset, pub, rate):
    global current_pose  

    # Calculate the target angle and normalize it
    target_angle = current_pose.theta + angle_offset
    # Normalize to [-90, 90]
    target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))  
    

    while not rospy.is_shutdown():
        # Calculate the angular error
        error = target_angle - current_pose.theta
        error = math.atan2(math.sin(error), math.cos(error))  # Normalize to [-90, 90]

        # If the error is within tolerance, stop turning
        if abs(error) < 0.001:  # Tolerance for angle. 0.001 for better accuracy
            break

        # Proportional control for smooth turning
        angular_speed = 2.0 * error  # Adjust gain as necessary
        move_robot(0, angular_speed)
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot after turning
    move_robot(0, 0)
    pub.publish(move_cmd)


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

            # Move forward until a boundary is about to be crossed
            while not rospy.is_shutdown() and not check_boundary_crossed():
                move_robot(speed, 0)
                pub.publish(move_cmd)
                rate.sleep() # Maintain the loop rate

            if direction > 0:
                # Move up to the next row
                print("------------Turning left - anticlockwise.")
                turn_to_angle(rad90, pub, rate)
                if check_boundary_crossed():
                    break
                else:
                    move_robot(speed, 0)  # Move forward
                    pub.publish(move_cmd)
                    rospy.sleep(3)
                turn_to_angle(rad90, pub, rate)
                direction *= -1  # Change the horizontal direction

            else:
                # Turn to change direction
                print("------------Turning right - clockwise..")
                turn_to_angle(-rad90, pub, rate)
                if check_boundary_crossed():
                    break
                else:
                    move_robot(speed, 0)  # Move forward
                    pub.publish(move_cmd)
                    rospy.sleep(3)
                turn_to_angle(-rad90, pub, rate)
                direction = 1

            rate.sleep()  # Maintain the loop rate

    except rospy.ROSInterruptException:
        print("")
        print(">> Program interrupted. Shutting down the bot..")

    finally:
        # Stop the turtle when exiting
        move_robot(0, 0)
        pub.publish(move_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("")
        print(">>> Job Complete. Shutting down the bot..")
        print("")

if __name__ == '__main__':
    main()
