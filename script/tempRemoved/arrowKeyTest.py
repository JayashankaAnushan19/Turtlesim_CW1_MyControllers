#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Save the terminal settings to revert after exiting
settings = termios.tcgetattr(sys.stdin)

# Function to get a single key press
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    
    # Check for escape character (prefix for arrow keys)
    if key == '\x1b':
        key += sys.stdin.read(2)  # Read two more characters for arrow keys
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    rospy.init_node('key_listener')
    
    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == "q":
                print("Exiting...")
                break
            elif key == '\x1b[A':  # Up arrow
                print("Up arrow pressed")
            elif key == '\x1b[B':  # Down arrow
                print("Down arrow pressed")
            elif key == '\x1b[C':  # Right arrow
                print("Right arrow pressed")
            elif key == '\x1b[D':  # Left arrow
                print("Left arrow pressed")
            else:
                print(f"Key pressed: {key}")
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
