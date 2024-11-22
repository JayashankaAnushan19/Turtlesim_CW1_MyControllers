#!/usr/bin/env python3
import rospy
from turtlesim.srv import TeleportAbsolute, Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


# Parameters
speed = 0.5  # Linear speed
rad90 = (90 * math.pi) / 180  # 90 degrees in radians

# Keep the positions in global
current_pose = Pose()  # To store the current position of the bot
move_cmd = Twist()  # To store movement commands

# Callback to update the current position
def get_current_position(data):
    global current_pose
    current_pose = data

# Function to check if the bot is about to hit a boundary
def check_boundary_crossed(left_boundary, right_boundary, bottom_boundary, upper_boundary):
    # Calculate the new position after moving one step forward
    new_x = current_pose.x + (speed * math.cos(current_pose.theta))
    new_y = current_pose.y + (speed * math.sin(current_pose.theta))

    # Check if the new position is outside the boundaries
    if new_x < left_boundary or new_x > right_boundary or new_y < bottom_boundary or new_y > upper_boundary:
        return True
    return False

# Function to move the robot
def move_robot(linear_x, angular_z):
    move_cmd.linear.x = linear_x
    move_cmd.angular.z = angular_z

# Generate a random float between 0 and 11
def clearner_positioning(turtle_name, turtle_x, turtle_y):

    # Generate random x and y positions for the turtle
    turtle_x = turtle_x 
    turtle_y = turtle_y 

    print(f"{turtle_name} Bot at X : {turtle_x} | Y : {turtle_y}")

    # Ensure the x and y are floats
    turtle_x = float(turtle_x)
    turtle_y = float(turtle_y)

    # Wait for the Teleport service to be available
    rospy.wait_for_service('/turtle1/teleport_absolute')

    try:
        if not turtle_name == "turtle1":
            # Create a service proxy for 'spawn'# Create a service proxy for 'spawn'
            spawn_service = rospy.ServiceProxy('/spawn', Spawn)
            # Spawn the turtle at the random coordinates
            spawn_service(turtle_x, turtle_y, 0, turtle_name)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def turn_to_angle(angle_offset, pub, rate):
    global current_pose  

    # Calculate the target angle and normalize it
    target_angle = current_pose.theta + angle_offset
    # Normalize to [-90, 90]
    target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))  
    

    while not rospy.is_shutdown():
        # Calculate the angular error
        error = target_angle - current_pose.theta
        error = math.atan2(math.sin(error), math.cos(error))  # Normalize to [-π, π]

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

def clearniner_behavior(turtle_name, turtle_x, turtle_y, left_boundary, right_boundary, bottom_boundary, upper_boundary):
    # rospy.init_node(f'vacuum_cleaner_bot + {turtle_name}')
    pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber(f'/{turtle_name}/pose', Pose, get_current_position)
    
    print(f"{turtle_name} Bot Started cleaning.")

    # Create a service proxy for 'teleport_absolute'
    teleport_service = rospy.ServiceProxy(f'/{turtle_name}/teleport_absolute', TeleportAbsolute)
    # Teleport the turtle to the random coordinates
    teleport_service(left_boundary + 0.5, bottom_boundary + 0.5, 0)

    rate = rospy.Rate(10)  # Loop rate of 10 Hz

    try:
        direction = 1  # Start by moving to the right

        while not rospy.is_shutdown():

            move_robot(speed, 0)
            pub.publish(move_cmd)
            rate.sleep()  # Maintain the loop rate

            # Move forward until a boundary is about to be crossed
            while not rospy.is_shutdown() and not check_boundary_crossed(left_boundary, right_boundary, bottom_boundary, upper_boundary):
                move_robot(speed, 0)
                pub.publish(move_cmd)
                rate.sleep()  # Maintain the loop rate

            if direction > 0:
                # Move up to the next row
                print(f"----------[{turtle_name}] :: Turning left - anticlockwise.")
                turn_to_angle(rad90, pub, rate)
                if check_boundary_crossed(left_boundary, right_boundary, bottom_boundary, upper_boundary):
                    break
                else:
                    move_robot(speed, 0)  # Move forward
                    pub.publish(move_cmd)
                    rospy.sleep(1)
                turn_to_angle(rad90, pub, rate)
                direction *= -1  # Change the horizontal direction

            else:
                # Turn to change direction
                print(f"----------[{turtle_name}] :: Turning right - clockwise..")
                turn_to_angle(-rad90, pub, rate)
                if check_boundary_crossed(left_boundary, right_boundary, bottom_boundary, upper_boundary):
                    break
                else:
                    move_robot(speed, 0)  # Move forward
                    pub.publish(move_cmd)
                    rospy.sleep(1)
                turn_to_angle(-rad90, pub, rate)
                direction = 1
        print("")
        print(f">>>>> Job Complete!!!. [{turtle_name}] bot Shutting down!!!.")

    except rospy.ROSInterruptException:
        print("")
        print(f">> Program interrupted !!!!!. Shutting down the [{turtle_name}] bot..")

    finally:
        # Stop the turtle when exiting
        move_robot(0, 0)
        pub.publish(move_cmd)


def main():
    if rospy.has_param("~turtle_name"):
        turtle_name = rospy.get_param('~turtle_name', 'turtle1')
        turtle_x = rospy.get_param('~turtle_x', 2.0)
        turtle_y = rospy.get_param('~turtle_y', 2.0)
        left_boundary = rospy.get_param('~left_boundary', 0)
        right_boundary = rospy.get_param('~right_boundary', 11)
        bottom_boundary = rospy.get_param('~bottom_boundary', 0)
        upper_boundary = rospy.get_param('~upper_boundary', 11)
        print(f"Another {turtle_name} :::::")
    else:
        turtle_name = "turtle1"
        turtle_x = 0
        turtle_y = 0
        left_boundary = 0
        right_boundary = 11
        bottom_boundary = 0
        upper_boundary = 11
        print("Turtle 01 :::::")

    clearner_positioning(turtle_name, turtle_x, turtle_y)
    clearniner_behavior(turtle_name, turtle_x, turtle_y, left_boundary, right_boundary, bottom_boundary, upper_boundary)

if __name__ == '__main__':
    rospy.init_node('vacuum_cleaner_node', anonymous=True)
    main()
