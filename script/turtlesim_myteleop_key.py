#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

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

def main():
    rospy.init_node('my_teleop_cmds')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Define initial speed and turn speed
    speed = 1.0
    move_cmd = Twist()

    print("--- Control Your Turtle! ---")
    print("Use 'WASD' or Arrow keys to move the bot")
    print("Press 'Q' or 'P' to quit to main menu.")
    print("Press 'R' to reset speed.")
    print("Press '+' to increase speed, '-' to decrease speed.")

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if not key == None:
                if key in ['w', '\x1b[A']:  # Up (W or Up Arrow)
                    move_cmd.linear.x = speed
                    move_cmd.angular.z = 0
                elif key in ['s', '\x1b[B']:  # Down (S or Down Arrow)
                    move_cmd.linear.x = -speed
                    move_cmd.angular.z = 0
                elif key in ['a', '\x1b[D']:  # Left (A or Left Arrow)
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = speed
                elif key in ['d', '\x1b[C']:  # Right (D or Right Arrow)
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = -speed
                elif key == '+':  # Increase speed
                    speed += 0.2
                    print(f"Speed increased to {speed:.2f}")
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0
                elif key == '-':  # Decrease speed
                    speed = max(0, speed - 0.2)
                    print(f"Speed decreased to {speed:.2f}")
                elif key == 'r':  # Reset speed
                    speed = 1.0
                    print("Speed reset to 1.0")
                elif key in ['q' , 'p']:  # Quit
                    print("Exiting...")
                    break
                else:
                    print("unrecongnized command")
                    # If an unrecognized key is pressed, stop movement
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0

                # Display movement and speed details for clarity
                print(f"Key: {key} | Linear Speed: {move_cmd.linear.x:.2f} | Angular Speed: {move_cmd.angular.z:.2f}")

                # Publish the movement command
                pub.publish(move_cmd)

    except Exception as e:
        print(e)

    finally:
        # Stop the turtle when exiting
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
