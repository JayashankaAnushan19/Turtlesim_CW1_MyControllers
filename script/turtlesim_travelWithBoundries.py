#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sin, cos
import sys, select, termios, tty

leftBoundry = None
rightBoundry = None
bottomBoundry = None
upperBoundry = None
speed = None

# Save the exixting terminal settings
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

def checkBoundryCrossed(direction):

    if direction == 'w':
        new_x = current_pose.x + (speed * cos(current_pose.theta))
        new_y = current_pose.y + (speed * sin(current_pose.theta))
    elif direction == 's':
        new_x = current_pose.x - (speed * cos(current_pose.theta))
        new_y = current_pose.y - (speed * sin(current_pose.theta))
    elif direction == 'a':
        new_x = current_pose.x
        new_y = current_pose.y
    elif direction == 'd':
        new_x = current_pose.x
        new_y = current_pose.y
    else:
        return False

    rospy.loginfo('-----------------------------')
    rospy.loginfo(f'Currect X : {current_pose.x}')
    rospy.loginfo(f'Current Y : {current_pose.y}')
    rospy.loginfo(f'Expected X : {new_x}')
    rospy.loginfo(f'Expected Y : {new_y}')
    rospy.loginfo('-----------------------------')

    if new_x <= leftBoundry or new_x >= rightBoundry or new_y <= bottomBoundry or new_y >= upperBoundry:
        rospy.loginfo('Wall is going to hit !!!. Can not move the bot.')
        return True
    return False


def getCurrentPosition(data):
    # Get the x and y cordinates of the position
    global current_pose
    current_pose = data
    current_pose.x = round(current_pose.x, 4)
    current_pose.y = round(current_pose.y, 4)

def main():
    rospy.init_node('my_teleop_cmds')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Subscriber to catch the currect position
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, getCurrentPosition)
    rate = rospy.Rate(10)
    
    # Define initial speed and turn speed
    global speed
    speed = 1.0
    move_cmd = Twist()


    try:
        rospy.loginfo("--- First set the boundaries ---")
        
        # Left boundary
        global leftBoundry
        leftBoundry = int(input("Enter the left boundary (between 0-11): "))
        # Validate left boundary
        while leftBoundry < 0 or leftBoundry > 11:
            rospy.loginfo("***Invalid input. Please re-enter the left boundary (between 0-11).")
            leftBoundry = int(input(f"Left boundary is: {leftBoundry}. Enter again: "))


        # Right boundary
        global rightBoundry
        rightBoundry = int(input("Enter the right boundary (bigger than left boundry and smaller than 11): "))
        # Validate right boundary
        while leftBoundry > rightBoundry or rightBoundry > 11 or leftBoundry == rightBoundry:
            rospy.loginfo("***Invalid input. Please re-enter the right boundary (bigger than left boundry and smaller than 11).")
            rightBoundry = int(input(f"Right boundary is: {rightBoundry}. Enter again: "))


        #Bottom boundry
        global bottomBoundry
        bottomBoundry = int(input("Enter the bottom boundary (between 0-11): "))
        # Validate bottom boundary
        while bottomBoundry < 0 or bottomBoundry > 11:
            rospy.loginfo("***Invalid input. Please re-enter the bottom boundary (between 0-11).")
            bottomBoundry = int(input(f"Bottom boundary is: {bottomBoundry}. Enter again: "))


        # Upper boundary
        global upperBoundry
        upperBoundry = int(input("Enter the upper boundary (bigger than bottom boundry and smaller than 11): "))
        # Validate upper boundary
        while bottomBoundry > upperBoundry or upperBoundry > 11 or bottomBoundry == upperBoundry:
            rospy.loginfo("***Invalid input. Please re-enter the upper boundary (bigger than left boundry and smaller than 11).")
            upperBoundry = int(input(f"Upper boundary is: {upperBoundry}. Enter again: "))        
        
        rospy.loginfo("--- Control Your Turtle! ---")
        rospy.loginfo("Use 'WASD' or Arrow keys to move the bot")
        rospy.loginfo("Press 'Q' or 'P' to quit to main menu.")
        rospy.loginfo("Press 'R' to reset speed.")
        rospy.loginfo("Press '+' to increase speed, '-' to decrease speed.")


        while not rospy.is_shutdown():
            key = getKey()
            if not key == None:
                # Up (W or Up Arrow)
                if key in ['w', '\x1b[A'] and not checkBoundryCrossed('w'):
                        move_cmd.linear.x = speed
                        move_cmd.angular.z = 0
                # Down (S or Down Arrow)
                elif key in ['s', '\x1b[B'] and not checkBoundryCrossed('s'):
                        move_cmd.linear.x = -speed
                        move_cmd.angular.z = 0
                # Left (A or Left Arrow)
                elif key in ['a', '\x1b[D'] and not checkBoundryCrossed('a'):
                        move_cmd.linear.x = 0
                        move_cmd.angular.z = speed
                # Right (D or Right Arrow)
                elif key in ['d', '\x1b[C'] and not checkBoundryCrossed('d'):
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
                elif key in ['q' , 'p']:  
                    rospy.loginfo("Exiting...")
                    break
                else:
                    if not key in ['w', 'a', 's', 'd', 'r', 'q', 'p', '+', '-', '\x1b[A', '\x1b[B', '\x1b[D', '\x1b[C']:
                        rospy.loginfo("Unrecongnized command")
                        rospy.loginfo(key)
                    # If an unrecognized key is pressed, stop movement
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0

                # Publish the movement command
                pub.publish(move_cmd)

    except Exception as e:
        rospy.loginfo(e)

    finally:
        # Stop the turtle when exiting
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
